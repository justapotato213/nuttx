/****************************************************************************
 * drivers/wireless/lpwan/lr1121/lr1121.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "lr1121.h"

#include <nuttx/arch.h>
#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/ioctl.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/endian.h>
#include <unistd.h>
#include <nuttx/wireless/lpwan/lr1121.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Private prototypes for file operations
 ****************************************************************************/

static int lr1121_open(FAR struct file *filep);

static int lr1121_close(FAR struct file *filep);

static ssize_t lr1121_read(FAR struct file *filep,
                           FAR char *buffer,
                           size_t buflen);

static ssize_t lr1121_write(FAR struct file *filep,
                            FAR const char *buf,
                            size_t buflen);

static int lr1121_ioctl(FAR struct file *filep,
                        int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private data types
 ****************************************************************************/

struct lr1121_dev_s
{
    struct spi_dev_s *spi;
    const struct lr1121_lower_s *lower;
    uint8_t times_opened;
    mutex_t lock; /* Only let one user in at a time */
    sem_t rx_sem;
    sem_t tx_sem;
    uint16_t irqbits;

    /* Hardware settings */

    bool invert_iq;

    /* Common settings */

    uint8_t payload_len;
    enum lr1121_packet_type_e packet_type; /* This will decide what modulation to use */
    uint32_t frequency_hz;
    uint8_t power;
    uint16_t preambles;

    /* LoRa settings */

    enum lr1121_lora_sf_e lora_sf;
    enum lr1121_lora_bw_e lora_bw;
    enum lr1121_lora_cr_e lora_cr;
    bool lora_crc;
    bool lora_fixed_header;
    bool low_datarate_optimization;
    uint8_t syncword[LR1121_REG_SYNCWORD_LEN];

    /* Interrupt handling */

    uint16_t irq_mask;
    struct work_s irq0_work;
};

enum lr1121_cmd_status
{
    LR1121_STATUS_RESERVED,
    LR1121_STATUS_RFU,
    LR1121_STATUS_DATA_AVAILABLE,
    LR1121_STATUS_TIMEOUT,
    LR1121_STATUS_ERROR,
    LR1121_STATUS_EXECUTE_FAIL,
    LR1121_STATUS_TX_DONE
};

enum lr1121_chip_mode
{
    LR1121_MODE_UNUSED,
    LR1121_MODE_RFU,
    LR1121_MODE_STBY_RC,
    LR1121_MODE_STBY_XOSC,
    LR1121_MODE_FS,
    LR1121_MODE_RX,
    LR1121_MODE_TX
};

struct lr1121_status_s
{
    enum lr1121_cmd_status cmd;
    enum lr1121_chip_mode mode;
};

static const struct file_operations lr1121_ops =
    {
        lr1121_open,
        lr1121_close,
        lr1121_read,
        lr1121_write,
        NULL,
        lr1121_ioctl,
        NULL,
        NULL};

/****************************************************************************
 * Globals
 ****************************************************************************/

FAR struct lr1121_dev_s g_lr1121_devices[LR1121_MAX_DEVICES];

/****************************************************************************
 * Private prototypes
 ****************************************************************************/

/* SPI and control **********************************************************/

static void lr1121_command(FAR struct lr1121_dev_s *dev,
                           uint8_t cmd,
                           FAR const uint8_t *params,
                           size_t paramslen,
                           FAR uint8_t *returns);

static void lr1121_reset(FAR struct lr1121_dev_s *dev);

static void lr1121_get_status(FAR struct lr1121_dev_s *dev,
                              FAR struct lr1121_status_s *status);

static void lr1121_spi_lock(FAR struct lr1121_dev_s *dev);

static void lr1121_spi_unlock(FAR struct lr1121_dev_s *dev);

static void lr1121_write_register(FAR struct lr1121_dev_s *dev,
                                  uint16_t address,
                                  uint8_t *data,
                                  size_t data_length);

/* Operational modes functions **********************************************/

static void lr1121_set_standby(FAR struct lr1121_dev_s *dev,
                               enum lr1121_standby_mode_e mode);

static void lr1121_set_tx(FAR struct lr1121_dev_s *dev,
                          uint32_t timeout);

static void lr1121_set_rx(FAR struct lr1121_dev_s *dev, uint32_t timeout);

static void lr1121_set_cad(struct lr1121_dev_s *dev);

static void lr1121_set_tx_continuous_wave(FAR struct lr1121_dev_s *dev);

static void lr1121_set_regulator_mode(FAR struct lr1121_dev_s *dev,
                                      enum lr1121_regulator_mode_e mode);

static void lr1121_set_pa_config(FAR struct lr1121_dev_s *dev,
                                 enum lr1121_device_e model,
                                 uint8_t hpmax,
                                 uint8_t padutycycle);

static void lr1121_set_tx_infinite_preamble(FAR struct lr1121_dev_s *dev);

/* DIO and IRQ control functions ********************************************/

static void lr1121_set_dio_irq_params(FAR struct lr1121_dev_s *dev,
                                      uint16_t irq_mask,
                                      uint16_t dio1_mask,
                                      uint16_t dio2_mask,
                                      uint16_t dio3_mask);

static void lr1121_set_dio2_as_rf_switch(FAR struct lr1121_dev_s *dev,
                                         bool enable);

static void lr1121_set_dio3_as_tcxo(FAR struct lr1121_dev_s *dev,
                                    enum lr1121_tcxo_voltage_e voltage,
                                    uint32_t delay);

static void lr1121_get_irq_status(FAR struct lr1121_dev_s *dev,
                                  FAR uint16_t *irqstatus);

static void lr1121_clear_irq_status(FAR struct lr1121_dev_s *dev,
                                    uint16_t clearbits);

/* RF Modulation and Packet-Related Functions *******************************/

static void lr1121_set_packet_params_lora(FAR struct lr1121_dev_s *dev,
                                          FAR struct
                                          lr1121_packetparams_lora_s *
                                              pktparams);

static void lr1121_set_modulation_params_lora(FAR struct lr1121_dev_s *dev,
                                              FAR struct
                                              lr1121_modparams_lora_s *
                                                  modparams);

static void lr1121_set_buffer_base_address(FAR struct lr1121_dev_s *dev,
                                           uint8_t tx,
                                           uint8_t rx);

static void lr1121_set_tx_params(FAR struct lr1121_dev_s *dev, uint8_t power,
                                 enum lr1121_ramp_time_e ramp_time);

static void lr1121_set_packet_type(FAR struct lr1121_dev_s *dev,
                                   enum lr1121_packet_type_e type);

static void lr1121_set_rf_frequency(FAR struct lr1121_dev_s *dev,
                                    uint32_t frequency_hz);

/* Communication status information *****************************************/

static void lr1121_get_rssi_inst(FAR struct lr1121_dev_s *dev,
                                 FAR int32_t *dbm);

static void lr1121_get_rx_buffer_status(FAR struct lr1121_dev_s *dev,
                                        uint8_t *status,
                                        uint8_t *payload_len,
                                        uint8_t *rx_buff_offset);

/* Registers and buffer *****************************************************/

static void lr1121_write_register(FAR struct lr1121_dev_s *dev,
                                  uint16_t address,
                                  uint8_t *data,
                                  size_t data_length);

static void lr1121_write_buffer(FAR struct lr1121_dev_s *dev,
                                uint8_t offset,
                                FAR const uint8_t *payload,
                                uint8_t len);

static void lr1121_read_buffer(FAR struct lr1121_dev_s *dev,
                               uint8_t offset,
                               FAR uint8_t *payload,
                               uint8_t len);

/* Register settings ********************************************************/

static void lr1121_set_syncword(FAR struct lr1121_dev_s *dev,
                                uint8_t *syncword,
                                uint8_t syncword_length);

/* Driver specific **********************************************************/

static int lr1121_init(FAR struct lr1121_dev_s *dev);

static int lr1121_deinit(FAR struct lr1121_dev_s *dev);

static int lr1121_setup_radio(FAR struct lr1121_dev_s *dev);

static void lr1121_set_defaults(FAR struct lr1121_dev_s *dev);

/* Interrupt handlers *******************************************************/

static int lr1121_irq0handler(int irq, FAR void *context, FAR void *arg);

static inline int lr1121_attachirq0(FAR struct lr1121_dev_s *dev, xcpt_t isr,
                                    FAR void *arg);

static void lr1121_isr0_process(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* File operations **********************************************************/

static int lr1121_open(FAR struct file *filep)
{
    int ret = 0;

    /* Get device */

    struct lr1121_dev_s *dev;
    dev = filep->f_inode->i_private;
    wlinfo("Opening Lr1121 %d", dev->lower->dev_number);

    /* Lock dev */

    ret = nxmutex_lock(&dev->lock);
    if (ret < 0)
    {
        return ret;
    }

    /* Only one can open this dev at a time */

    if (dev->times_opened > 0)
    {
        ret = -EBUSY;
        goto exit_err;
    }

    /* Initialize */

    ret = lr1121_init(dev);
    if (ret != 0)
    {
        goto exit_err;
    }

    /* Success */

    dev->times_opened++;
    ret = OK;

exit_err:
    nxmutex_unlock(&dev->lock);
    return ret;
}

static int lr1121_close(FAR struct file *filep)
{
    int ret = 0;

    /* Get device */

    struct lr1121_dev_s *dev;
    dev = filep->f_inode->i_private;
    wlinfo("Closing Lr1121 %d", dev->lower->dev_number);

    /* Lock */

    ret = nxmutex_lock(&dev->lock);
    if (ret < 0)
    {
        goto exit_err;
    }

    /* De-init */

    ret = lr1121_deinit(dev);
    if (ret != 0)
    {
        goto exit_err;
    }

    /* Success */

    if (dev->times_opened > 0)
    {
        dev->times_opened--; /* Do not let this wrap around. */
        ret = OK;
    }

exit_err:
    nxmutex_unlock(&dev->lock);
    return ret;
}

static ssize_t lr1121_read(FAR struct file *filep,
                           FAR char *buf,
                           size_t buflen)
{
    int ret = 0;
    if (buf == NULL || buflen < 1)
    {
        return -EINVAL;
    }

    /* Get device */

    struct lr1121_dev_s *dev;
    dev = filep->f_inode->i_private;

    nxmutex_lock(&dev->lock);

    printf("Reading\n");

    /* Get header */

    struct lr1121_read_header_s *header = (struct lr1121_read_header_s *)buf;

    /* Pre-RX setup */

    lr1121_spi_lock(dev);
    dev->irq_mask = LR1121_IRQ_RXDONE_MASK | LR1121_IRQ_CRCERR_MASK;
    ret = lr1121_setup_radio(dev);
    if (ret != 0)
    {
        goto lr1121_rx_abort;
    }

    /* RX mode */

    lr1121_set_rx(dev, LR1121_NO_TIMEOUT);
    lr1121_spi_unlock(dev);

    /* Wait for a packet */

    nxsem_wait(&dev->rx_sem);

    /* Get payload */

    uint8_t status = 0;
    uint8_t offset = 0;

    lr1121_spi_lock(dev);
    lr1121_get_rx_buffer_status(dev, &status,
                                &header->payload_length,
                                &offset);
    lr1121_read_buffer(dev, offset, header->payload,
                       header->payload_length);
    lr1121_spi_unlock(dev);

    /* Get CRC check */

    header->crc_error = dev->irqbits & LR1121_IRQ_CRCERR_MASK;

    /* Exit */

lr1121_rx_abort:

    nxmutex_unlock(&dev->lock);

    return 1;
}

static ssize_t lr1121_write(FAR struct file *filep,
                            FAR const char *buf,
                            size_t buflen)
{
    int ret = 0;

    /* Get device */

    struct lr1121_dev_s *dev;
    dev = filep->f_inode->i_private;

    if (buf == NULL || buflen < 1)
    {
        return -EINVAL;
    }

    nxmutex_lock(&dev->lock);
    lr1121_spi_lock(dev);

    /* Data */

    dev->payload_len = buflen;
    lr1121_write_buffer(dev, 0, (uint8_t *)buf, buflen);

    /* Pre-TX setup */

    dev->irq_mask = LR1121_IRQ_TXDONE_MASK;
    ret = lr1121_setup_radio(dev);
    if (ret != 0)
    {
        lr1121_spi_unlock(dev);
        goto lr1121_tx_abort;
    }

    /* TX */

    lr1121_set_tx(dev, 0);

    lr1121_spi_unlock(dev);

    /* Wait for transmitting operations to be finished */

    wlinfo("TXing");
    ret = nxsem_wait(&dev->tx_sem);

lr1121_tx_abort:

    nxmutex_unlock(&dev->lock);
    return ret;
}

static int lr1121_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
    int ret = 0;

    /* Get device */

    struct lr1121_dev_s *dev;
    dev = filep->f_inode->i_private;
    wlinfo("IOCTL cmd %d arg %u Lr1121 dev_number %d",
           cmd,
           *(FAR uint32_t *)((uintptr_t)arg),
           dev->lower->dev_number);

    /* Lock */

    ret = nxmutex_lock(&dev->lock);
    if (ret < 0)
    {
        goto exit_err;
    }

    /* Do thing */

    switch (cmd)
    {
        /* Set radio freq. Takes uint32_t *frequency in Hz */

    case WLIOC_SETRADIOFREQ:
    {
        FAR uint32_t *freq_ptr = (FAR uint32_t *)((uintptr_t)arg);
        DEBUGASSERT(freq_ptr != NULL);

        dev->frequency_hz = *freq_ptr;
        break;
    }

        /* Get radio freq. Sets uint32_t *frequency in Hz */

    case WLIOC_GETRADIOFREQ:
    {
        FAR uint32_t *freq_ptr = (FAR uint32_t *)((uintptr_t)arg);
        DEBUGASSERT(freq_ptr != NULL);

        *freq_ptr = dev->frequency_hz;
        break;
    }

        /* Set TX power. arg: Pointer to int8_t power value */

    case WLIOC_SETTXPOWER:
    {
        FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);
        DEBUGASSERT(ptr != NULL);

        dev->power = *ptr;
        break;
    }

        /* Get current TX power. arg: Pointer to int8_t power value */

    case WLIOC_GETTXPOWER:
    {
        FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);
        DEBUGASSERT(ptr != NULL);

        *ptr = dev->power;
        break;
    }

        /* TODO: Integration with new common IOCTL API */

        /* Driver specific IOCTL */

        /* Lora config */

    case LR1121IOC_LORACONFIGSET:
    {
        FAR struct lr1121_lora_config_s *ptr =
            (FAR struct lr1121_lora_config_s *)((uintptr_t)arg);
        DEBUGASSERT(ptr != NULL);

        /* Modulation params */

        dev->lora_sf = ptr->modulation.spreading_factor;
        dev->lora_bw = ptr->modulation.bandwidth;
        dev->lora_cr = ptr->modulation.coding_rate;
        dev->low_datarate_optimization =
            ptr->modulation.low_datarate_optimization;

        /* Packet params */

        dev->lora_crc = ptr->packet.crc_enable;
        dev->lora_fixed_header =
            ptr->packet.fixed_length_header;
        dev->payload_len = ptr->packet.payload_length;
        dev->invert_iq = ptr->packet.invert_iq;
        dev->preambles = ptr->packet.preambles;

        break;
    }
    }

    /* Success */

    ret = OK;

exit_err:
    nxmutex_unlock(&dev->lock);
    return ret;
}

uint32_t lr1121_convert_freq_in_hz_to_pll_step(uint32_t freq_in_hz)
{
    uint32_t steps_int;
    uint32_t steps_frac;

    steps_int = freq_in_hz / LR1121_PLL_STEP_SCALED;
    steps_frac = freq_in_hz - (steps_int * LR1121_PLL_STEP_SCALED);

    return (steps_int << LR1121_PLL_STEP_SHIFT_AMOUNT) + (((steps_frac << LR1121_PLL_STEP_SHIFT_AMOUNT) + (LR1121_PLL_STEP_SCALED >>
                                                                                                           1)) /
                                                          LR1121_PLL_STEP_SCALED);
}

/* Operational modes functions **********************************************/

static void lr1121_set_standby(FAR struct lr1121_dev_s *dev,
                               enum lr1121_standby_mode_e mode)
{
    lr1121_command(dev, LR1121_SETSTANDBY, (uint8_t *)&mode,
                   LR1121_SETSTANDBY_PARAMS, NULL);
}

static void lr1121_set_tx(FAR struct lr1121_dev_s *dev, uint32_t timeout)
{
    /* Convert timeout to BE24 */

    timeout = htobe32(timeout << 8);

    lr1121_command(dev, LR1121_SETTX, (uint8_t *)&timeout, LR1121_SETTX_PARAMS,
                   NULL);
}

static void lr1121_set_rx(FAR struct lr1121_dev_s *dev, uint32_t timeout)
{
    /* Convert timeout to BE24 */

    timeout = htobe32(timeout << 8);

    lr1121_command(dev, LR1121_SETRX, (uint8_t *)&timeout, LR1121_SETRX_PARAMS,
                   NULL);
}

static void lr1121_stop_timer_on_preamble(FAR struct lr1121_dev_s *dev,
                                          bool enable)
{
    lr1121_command(dev, LR1121_STOPTIMERONPREAMBLE, (uint8_t *)&enable,
                   LR1121_STOPTIMERONPREAMBLE_PARAMS, NULL);
}

static void lr1121_set_rx_duty_cycle(FAR struct lr1121_dev_s *dev,
                                     uint32_t rx_period,
                                     uint32_t sleep_period)
{
    uint8_t params[LR1121_SETRXDUTYCYCLE_PARAMS];

    rx_period = htobe32(rx_period << 8);
    sleep_period = htobe32(sleep_period << 8);

    memcpy(params + LR1121_SETRXDUTYCYCLE_RXPERIOD_PARAM,
           (uint8_t *)&rx_period,
           LR1121_SETRXDUTYCYCLE_RXPERIOD_PARAMS);
    memcpy(params + LR1121_SETRXDUTYCYCLE_SLEEPPERIOD_PARAM,
           (uint8_t *)&sleep_period,
           LR1121_SETRXDUTYCYCLE_SLEEPPERIOD_PARAMS);

    lr1121_command(dev, LR1121_SETRXDUTYCYCLE, params,
                   LR1121_SETRXDUTYCYCLE_PARAMS, NULL);
}

static void lr1121_set_cad(FAR struct lr1121_dev_s *dev)
{
    lr1121_command(dev, LR1121_SETCAD, NULL, 0, NULL);
}

static void lr1121_set_tx_continuous_wave(FAR struct lr1121_dev_s *dev)
{
    lr1121_command(dev, LR1121_SETTXCONTINUOUSWAVE, NULL, 0, NULL);
}

static void lr1121_set_regulator_mode(FAR struct lr1121_dev_s *dev,
                                      enum lr1121_regulator_mode_e mode)
{
    lr1121_command(dev, LR1121_SETREGULATORMODE, (uint8_t *)&mode,
                   LR1121_SETREGULATORMODE_PARAMS, NULL);
}

/* Caution! Exceeding the limits listed in DS_SX1261/2 V2.1
 * 13.1.14.1 may cause irreversible damage to the device
 */

static void lr1121_set_pa_config(FAR struct lr1121_dev_s *dev,
                                 enum lr1121_device_e model, uint8_t hpmax,
                                 uint8_t padutycycle)
{
    uint8_t params[LR1121_SETPACONFIG_PARMS];

    memset(params, 0, LR1121_SETPACONFIG_PARMS);

    params[LR1121_SETPACONFIG_PADUTYCYCLE_PARAM] = padutycycle;
    params[LR1121_SETPACONFIG_HPMAX_PARAM] = hpmax;
    params[LR1121_SETPACONFIG_DEVICESEL_PARAM] = model;
    params[LR1121_SETPACONFIG_PALUT_PARAM] = 0x01;

    lr1121_command(dev, LR1121_SETPACONFIG, params, LR1121_SETPACONFIG_PARMS,
                   NULL);
}

static void lr1121_set_tx_infinite_preamble(FAR struct lr1121_dev_s *dev)
{
    lr1121_command(dev, LR1121_SETTXINFINITEPREAMBLE, NULL, 0, NULL);
}

static void lr1121_set_rx_tx_fallback_mode(FAR struct lr1121_dev_s *dev,
                                           enum lr1121_fallback_mode_e
                                               fallback)
{
    lr1121_command(dev, LR1121_SETRXTXFALLBACKMODE, (uint8_t *)&fallback,
                   LR1121_SETRXTXFALLBACKMODE_PARAMS, NULL);
}

/* DIO and IRQ control functions */

static void lr1121_set_dio_irq_params(FAR struct lr1121_dev_s *dev,
                                      uint16_t irq_mask, uint16_t dio1_mask,
                                      uint16_t dio2_mask, uint16_t dio3_mask)
{
    irq_mask = htobe16(irq_mask);
    dio1_mask = htobe16(dio1_mask);
    dio2_mask = htobe16(dio2_mask);
    dio3_mask = htobe16(dio3_mask);

    uint8_t params[LR1121_SETDIOIRQPARAMS_PARAMS];

    memcpy(params + LR1121_SETDIOIRQPARAMS_IRQMASK_PARAM, &irq_mask,
           LR1121_SETDIOIRQPARAMS_IRQMASK_PARAMS);

    memcpy(params + LR1121_SETDIOIRQPARAMS_DIO1MASK_PARAM, &dio1_mask,
           LR1121_SETDIOIRQPARAMS_DIO1MASK_PARAMS);

    memcpy(params + LR1121_SETDIOIRQPARAMS_DIO2MASK_PARAM, &dio2_mask,
           LR1121_SETDIOIRQPARAMS_DIO2MASK_PARAMS);

    memcpy(params + LR1121_SETDIOIRQPARAMS_DIO3MASK_PARAM, &dio3_mask,
           LR1121_SETDIOIRQPARAMS_DIO3MASK_PARAMS);

    lr1121_command(dev, LR1121_SETDIOIRQPARAMS, params,
                   LR1121_SETDIOIRQPARAMS_PARAMS, NULL);
}

static void lr1121_set_dio2_as_rf_switch(FAR struct lr1121_dev_s *dev,
                                         bool enable)
{
    lr1121_command(dev, LR1121_SETDIO2RFSWCTRL, (uint8_t *)&enable,
                   LR1121_SETDIO2RFSWCTRL_PARAMS, NULL);
}

static void lr1121_set_dio3_as_tcxo(FAR struct lr1121_dev_s *dev,
                                    enum lr1121_tcxo_voltage_e voltage,
                                    uint32_t delay)
{
    uint8_t params[LR1121_SETDIO3TCXOCTRL_PARAMS];

    params[LR1121_SETDIO3TCXOCTRL_TCXO_V_PARAM] = voltage;

    /* Convert delay to 24 bit and convert to BE */

    delay = htobe32(delay << 8);
    memcpy(params + LR1121_SETDIO3TCXOCTRL_DELAY_PARAM, &delay,
           LR1121_SETDIO3TCXOCTRL_DELAY_PARAMS);

    lr1121_command(dev, LR1121_SETDIO3TCXOCTRL, params,
                   LR1121_SETDIO3TCXOCTRL_PARAMS, NULL);
}

static void lr1121_get_irq_status(FAR struct lr1121_dev_s *dev,
                                  FAR uint16_t *irqstatus)
{
    uint8_t returns[LR1121_GETIRQSTATUS_RETURNS];

    lr1121_command(dev, LR1121_GETIRQSTATUS,
                   NULL, LR1121_GETIRQSTATUS_RETURNS,
                   returns);

    uint16_t bits;
    memcpy(&bits, returns + LR1121_GETIRQSTATUS_IRQSTATUS_RETURN,
           LR1121_GETIRQSTATUS_IRQSTATUS_RETURNS);

    *irqstatus = be16toh(bits);
}

static void lr1121_clear_irq_status(FAR struct lr1121_dev_s *dev,
                                    uint16_t clearbits)
{
    uint8_t params[LR1121_CLEARIRQSTATUS_PARAMS];

    clearbits = htobe16(clearbits);
    memcpy(params + LR1121_CLEARIRQSTATUS_CLEAR_PARAM,
           &clearbits,
           LR1121_CLEARIRQSTATUS_CLEAR_PARAMS);

    lr1121_command(dev, LR1121_CLEARIRQSTATUS, params,
                   LR1121_CLEARIRQSTATUS_PARAMS,
                   NULL);
}

/* RF Modulation and Packet-Related Functions *******************************/

static void lr1121_set_packet_params_lora(FAR struct lr1121_dev_s *dev,
                                          FAR struct
                                          lr1121_packetparams_lora_s *
                                              pktparams)
{
    uint8_t params[LR1121_SETPACKETPARMS_PARAMS];

    memset(params, 0, LR1121_SETPACKETPARMS_PARAMS);

    uint16_t preambles = htobe16(pktparams->preambles);
    memcpy(params + LR1121_PKTPARAM1_LORA_PREAMBLELEN_PARAM, &preambles,
           LR1121_PKTPARAM1_LORA_PREAMBLELEN_PARAMS);

    params[LR1121_PKTPARAM3_LORA_HEADERTYPE_PARAM] =
        pktparams->fixed_length_header;
    params[LR1121_PKTPARAM4_LORA_PAYLOADLEN_PARAM] =
        pktparams->payload_length;
    params[LR1121_PKTPARAM5_LORA_CRCTYPE_PARAM] = pktparams->crc_enable;
    params[LR1121_PKTPARAM6_LORA_INVERTIQ_PARAM] = pktparams->invert_iq;

    lr1121_command(dev, LR1121_SETPACKETPARMS, params,
                   LR1121_SETPACKETPARMS_PARAMS, NULL);
}

static void lr1121_set_modulation_params_lora(FAR struct lr1121_dev_s *dev,
                                              FAR struct
                                              lr1121_modparams_lora_s *
                                                  modparams)
{
    uint8_t params[LR1121_SETMODULATIONPARAMS_PARAMS];

    memset(params, 0, LR1121_SETMODULATIONPARAMS_PARAMS);

    params[LR1121_MODPARAM1_LORA_SF_PARAM] =
        modparams->spreading_factor;
    params[LR1121_MODPARAM2_LORA_BW_PARAM] =
        modparams->bandwidth;
    params[LR1121_MODPARAM3_LORA_CR_PARAM] =
        modparams->coding_rate;
    params[LR1121_MODPARAM4_LORA_LOWDATRATE_OPTI_PARAM] =
        modparams->low_datarate_optimization;

    lr1121_command(dev, LR1121_SETMODULATIONPARAMS, params,
                   LR1121_SETMODULATIONPARAMS_PARAMS, NULL);
}

static void lr1121_set_buffer_base_address(FAR struct lr1121_dev_s *dev,
                                           uint8_t tx, uint8_t rx)
{
    uint8_t params[LR1121_SETBUFFERBASEADDRESS_PARAMS];

    memset(params, 0, LR1121_SETBUFFERBASEADDRESS_PARAMS);

    params[LR1121_SETBUFFERBASEADDRESS_TX_PARAM] = tx;
    params[LR1121_SETBUFFERBASEADDRESS_RX_PARAM] = rx;

    lr1121_command(dev, LR1121_SETBUFFERBASEADDRESS, params,
                   LR1121_SETBUFFERBASEADDRESS_PARAMS, NULL);
}

static void lr1121_set_tx_params(FAR struct lr1121_dev_s *dev, uint8_t power,
                                 enum lr1121_ramp_time_e ramp_time)
{
    uint8_t params[LR1121_SETTXPARMS_PARAMS];

    memset(params, 0, LR1121_SETTXPARMS_PARAMS);

    params[LR1121_SETTXPARMS_RAMPTIME_PARAM] = ramp_time;
    params[LR1121_SETTXPARMS_POWER_PARAM] = power;

    lr1121_command(dev, LR1121_SETTXPARMS, params, LR1121_SETTXPARMS_PARAMS,
                   NULL);
}

static void lr1121_set_packet_type(FAR struct lr1121_dev_s *dev,
                                   enum lr1121_packet_type_e type)
{
    lr1121_command(dev, LR1121_SETPACKETTYPE, (uint8_t *)&type,
                   LR1121_SETPACKETTYPE_PARAMS, NULL);
}

static void lr1121_set_rf_frequency(FAR struct lr1121_dev_s *dev,
                                    uint32_t frequency_hz)
{
    uint32_t corrected_freq =
        lr1121_convert_freq_in_hz_to_pll_step(frequency_hz);

    corrected_freq = htobe32(corrected_freq);

    lr1121_command(dev, LR1121_SETRFFREQUENCY, (uint8_t *)&corrected_freq,
                   LR1121_SETRFFREQUENCY_PARAMS, NULL);
}

static void lr1121_set_lora_symb_num_timout(FAR struct lr1121_dev_s *dev,
                                            uint8_t symbnum)
{
    lr1121_command(dev, LR1121_SETLORASYMBNUMTIMEOUT, &symbnum,
                   LR1121_SETLORASYMBNUMTIMEOUT_PARAMS, NULL);
}

/* Communication status information *****************************************/

static void lr1121_get_status(FAR struct lr1121_dev_s *dev,
                              FAR struct lr1121_status_s *status)
{
    /* NOP param to shift out result */

    uint8_t parms[1] = {
        0x00};

    uint8_t rets[1];

    /* Get, mask and shift result into readable mode numbers */

    lr1121_command(dev, LR1121_CMD_GETSTATUS, parms, sizeof(parms), rets);
    status->mode = (rets[0] & LR1121_STATUS_CHIPMODE_MASK) >>
                   LR1121_STATUS_CHIPMODE_SHIFT;
    status->cmd = (rets[0] & LR1121_STATUS_CMD_MASK) >>
                  LR1121_STATUS_CMD_SHIFT;
}

static void lr1121_get_rssi_inst(FAR struct lr1121_dev_s *dev,
                                 FAR int32_t *dbm)
{
    uint8_t rets[LR1121_GETRSSIINST_RETURNS];

    lr1121_command(dev, LR1121_GETRSSIINST, NULL, LR1121_GETRSSIINST_RETURNS,
                   rets);

    /* Calculate dBm from returns */

    int32_t rssi = rets[LR1121_GETRSSIINST_RSSI_RETURN];
    (*dbm) = -rssi / 2.0;
}

static void lr1121_get_rx_buffer_status(FAR struct lr1121_dev_s *dev,
                                        uint8_t *status,
                                        uint8_t *payload_len,
                                        uint8_t *rx_buff_offset)
{
    uint8_t returns[LR1121_GETRXBUFFERSTATUS_RETURNS];

    lr1121_command(dev, LR1121_GETRXBUFFERSTATUS,
                   NULL,
                   LR1121_GETRXBUFFERSTATUS_RETURNS,
                   returns);

    *status = returns[LR1121_GETRXBUFFERSTATUS_STATUS_RETURN];
    *payload_len = returns[LR1121_GETRXBUFFERSTATUS_PAYLOAD_LEN_RETURN];
    *rx_buff_offset = returns[LR1121_GETRXBUFFERSTATUS_RX_START_PTR_RETURN];
}

/* Lower hardware control ***************************************************/

static void lr1121_reset(FAR struct lr1121_dev_s *dev)
{
    dev->lower->reset();
}

/* SPI Communication ********************************************************/

static void lr1121_select(FAR struct lr1121_dev_s *dev)
{
    SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->dev_number), true);
}

static void lr1121_deselect(FAR struct lr1121_dev_s *dev)
{
    SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->dev_number), false);
}

static void lr1121_spi_lock(FAR struct lr1121_dev_s *dev)
{
    struct spi_dev_s *spi = dev->spi;

    SPI_LOCK(spi, true);
    SPI_SETBITS(spi, 8);
    SPI_SETMODE(spi, SPIDEV_MODE0);
    SPI_SETFREQUENCY(spi, LR1121_SPI_SPEED);
}

static void lr1121_spi_unlock(FAR struct lr1121_dev_s *dev)
{
    SPI_LOCK(dev->spi, false);
}

static void lr1121_command(FAR struct lr1121_dev_s *dev, uint8_t cmd,
                           const FAR uint8_t *params, size_t paramslen,
                           FAR uint8_t *returns)
{
    lr1121_select(dev);

    /* First send the command. This does not return anything.
     * "RFU" according the manual
     */

    SPI_SEND(dev->spi, cmd);

    /* Send all the params and record the returning bytes */

    for (size_t i = 0; i < paramslen; i++)
    {
        uint8_t param = LR1121_NOP;
        if (params != NULL)
        {
            param = params[i];
        }

        uint8_t ret = SPI_SEND(dev->spi, param);

        if (returns != NULL)
        {
            returns[i] = ret;
        }
    }

    lr1121_deselect(dev);
}

/* Registers and buffer *****************************************************/

static void lr1121_write_register(FAR struct lr1121_dev_s *dev,
                                  uint16_t address,
                                  uint8_t *data,
                                  size_t data_length)
{
    lr1121_select(dev);

    /* Send the opcode and address */

    SPI_SEND(dev->spi, LR1121_WRITEREGISTER);
    SPI_SEND(dev->spi, (uint8_t)(address >> 8));
    SPI_SEND(dev->spi, (uint8_t)address);

    /* Send data */

    for (size_t i = 0; i < data_length; i++)
    {
        SPI_SEND(dev->spi, data[i]);
    }

    lr1121_deselect(dev);
}

static void lr1121_read_register(FAR struct lr1121_dev_s *dev,
                                 uint16_t address,
                                 uint8_t *data,
                                 size_t data_length)
{
    lr1121_select(dev);

    /* Send the opcode and address */

    SPI_SEND(dev->spi, LR1121_WRITEREGISTER);
    SPI_SEND(dev->spi, (uint8_t)(address >> 8));
    SPI_SEND(dev->spi, (uint8_t)address);

    /* Send data */

    for (size_t i = 0; i < data_length; i++)
    {
        data[i] = SPI_SEND(dev->spi, LR1121_NOP);
    }

    lr1121_deselect(dev);
}

static void lr1121_write_buffer(FAR struct lr1121_dev_s *dev,
                                uint8_t offset,
                                FAR const uint8_t *payload,
                                uint8_t len)
{
    lr1121_select(dev);

    /* Command */

    SPI_SEND(dev->spi, LR1121_WRITEBUFFER);

    /* Offset */

    SPI_SEND(dev->spi, offset);

    /* Data */

    for (size_t i = 0; i < len; i++)
    {
        SPI_SEND(dev->spi, payload[i]);
    }

    lr1121_deselect(dev);
}

static void lr1121_read_buffer(FAR struct lr1121_dev_s *dev,
                               uint8_t offset,
                               FAR uint8_t *payload,
                               uint8_t len)
{
    lr1121_select(dev);

    /* Command */

    SPI_SEND(dev->spi, LR1121_READBUFFER);

    /* Offset */

    SPI_SEND(dev->spi, offset);

    /* NOP */

    SPI_SEND(dev->spi, LR1121_NOP);

    /* Data */

    for (size_t i = 0; i < len; i++)
    {
        payload[i] = SPI_SEND(dev->spi, LR1121_NOP);
    }

    lr1121_deselect(dev);
}

/* Register settings ********************************************************/

static void lr1121_set_syncword(FAR struct lr1121_dev_s *dev,
                                uint8_t *syncword,
                                uint8_t syncword_length)
{
    if (syncword_length > LR1121_REG_SYNCWORD_LEN)
    {
        syncword_length = LR1121_REG_SYNCWORD_LEN;
        wlerr("Syncword length was limited to the maximum 8 bytes");
    }

    lr1121_write_register(dev, LR1121_REG_SYNCWORD, syncword, syncword_length);
}

/* Driver specific **********************************************************/

static int lr1121_init(FAR struct lr1121_dev_s *dev)
{
    lr1121_reset(dev);
    lr1121_set_defaults(dev);
    return 0;
}

static int lr1121_deinit(FAR struct lr1121_dev_s *dev)
{
    return 0;
}

static void lr1121_set_defaults(FAR struct lr1121_dev_s *dev)
{
    /* Hardware defaults */

    dev->invert_iq = LR1121_DEFAULT_INVERT_IQ;

    /* Common defaults */

    dev->packet_type = LR1121_DEFAULT_PACKET_TYPE;
    dev->frequency_hz = LR1121_DEFAULT_FREQ;
    dev->power = LR1121_DEFAULT_POWER;
    dev->preambles = LR1121_DEFAULT_LORA_PREAMBLES;

    /* LoRa defaults */

    dev->lora_sf = LR1121_DEFAULT_LORA_SF;
    dev->lora_bw = LR1121_DEFAULT_LORA_BW;
    dev->lora_cr = LR1121_DEFAULT_LORA_CR;
    dev->lora_fixed_header = LR1121_DEFAULT_LORA_FIXED_HEADER;
    dev->lora_crc = LR1121_DEFAULT_LORA_CRC_EN;
    dev->low_datarate_optimization = LR1121_DEFAULT_LORA_LDO;

    uint8_t newsyncword[] = LR1121_DEFAULT_SYNCWORD;
    memcpy(dev->syncword, newsyncword, sizeof(dev->syncword));

    /* GFSK defaults */
}

static int lr1121_setup_radio(FAR struct lr1121_dev_s *dev)
{
    /* Clear IRQ status */

    lr1121_clear_irq_status(dev, 0xffff);

    /* Set regulator */

    lr1121_set_regulator_mode(dev, dev->lower->regulator_mode);

    /* Set packet type */

    lr1121_set_packet_type(dev, dev->packet_type);

    /* Set RF frequency */

    int illegal_freq = dev->lower->check_frequency(dev->frequency_hz);

    if (illegal_freq)
    {
        wlerr("Board does not support %dHz", dev->frequency_hz);
        return -1;
    }

    lr1121_set_rf_frequency(dev, dev->frequency_hz);

    /* Set PA settings from lower */

    uint8_t hp;
    uint8_t dc;
    enum lr1121_device_e model;
    dev->lower->get_pa_values(&model, &hp, &dc);
    lr1121_set_pa_config(dev, model, hp, dc);

    /* Set TX params */

    dev->lower->limit_tx_power(&dev->power); /* Limited by board */
    lr1121_set_tx_params(dev, dev->power, dev->lower->tx_ramp_time);

    /* Set base */

    lr1121_set_buffer_base_address(dev, 0, 0);

    /* Set params depending on packet type */

    switch (dev->packet_type)
    {
    case (LR1121_PACKETTYPE_LORA):
    {
        /* Mod params */

        struct lr1121_modparams_lora_s modparams = {
            .spreading_factor = dev->lora_sf,
            .bandwidth = dev->lora_bw,
            .coding_rate = dev->lora_cr,
            .low_datarate_optimization = dev->low_datarate_optimization};

        lr1121_set_modulation_params_lora(dev, &modparams);

        /* Packet params */

        struct lr1121_packetparams_lora_s pktparams = {
            .crc_enable = dev->lora_crc,
            .fixed_length_header = dev->lora_fixed_header,
            .payload_length = dev->payload_len,
            .invert_iq = dev->invert_iq,
            .preambles = dev->preambles};

        lr1121_set_packet_params_lora(dev, &pktparams);
        break;
    }

    default:
        break;
    }

    /* Sync word */

    lr1121_set_syncword(dev, dev->syncword, sizeof(dev->syncword));

    /* IRQ MASK */

    lr1121_set_dio_irq_params(dev, dev->irq_mask,
                              dev->lower->masks.dio1_mask,
                              dev->lower->masks.dio2_mask,
                              dev->lower->masks.dio3_mask);

    /* DIO 2 */

    lr1121_set_dio2_as_rf_switch(dev, dev->lower->use_dio2_as_rf_sw);

    /* DIO 3 */

    lr1121_set_dio3_as_tcxo(dev, dev->lower->dio3_voltage,
                            dev->lower->dio3_delay);
    return 0;
}

/* Interrupt handling *******************************************************/

static int lr1121_irq0handler(int irq, FAR void *context, FAR void *arg)
{
    FAR struct lr1121_dev_s *dev = (FAR struct lr1121_dev_s *)arg;

    DEBUGASSERT(dev != NULL);

    DEBUGASSERT(work_available(&dev->irq0_work));

    return work_queue(HPWORK, &dev->irq0_work, lr1121_isr0_process, arg, 0);
}

static inline int lr1121_attachirq0(FAR struct lr1121_dev_s *dev, xcpt_t isr,
                                    FAR void *arg)
{
    DEBUGASSERT(dev->lower->irq0attach != NULL);

    return dev->lower->irq0attach(isr, arg);
}

static void lr1121_isr0_process(FAR void *arg)
{
    DEBUGASSERT(arg);

    FAR struct lr1121_dev_s *dev = (FAR struct lr1121_dev_s *)arg;

    wlinfo("Lr1121 ISR0 process triggered");

    /* Get and clear IRQ bits */

    lr1121_spi_lock(dev);
    lr1121_get_irq_status(dev, &dev->irqbits);
    lr1121_spi_unlock(dev);

    wlinfo("IRQ status 0x%X", dev->irqbits);

    /* On TX done */

    if (dev->irqbits & LR1121_IRQ_TXDONE_MASK)
    {
        wlinfo("TX done");

        /* Release writing threads */

        nxsem_post(&dev->tx_sem);
    }

    /* On RX done */

    if (dev->irqbits & LR1121_IRQ_RXDONE_MASK)
    {
        wlinfo("RX done");

        nxsem_post(&dev->rx_sem);
    }

    /* On CAD done */

    if (dev->irqbits & LR1121_IRQ_CADDONE_MASK)
    {
        wlinfo("CAD done");
    }

    /* On CAD detect */

    if (dev->irqbits & LR1121_IRQ_CADDETECTED_MASK)
    {
        wlinfo("CAD detect");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void lr1121_register(FAR struct spi_dev_s *spi,
                     FAR const struct lr1121_lower_s *lower,
                     const char *path)
{
    /* Register the dev using an unique dev_number,
     * so multiple radios can be registered at once
     */

    if (lower->dev_number >= LR1121_MAX_DEVICES)
    {
        wlerr("Lr1121 dev_number %d is greater than \
            allowed amount of Lr1121 devices",
              lower->dev_number);
        return;
    }

    struct lr1121_dev_s *dev;
    dev = &g_lr1121_devices[lower->dev_number];
    dev->lower = lower;
    dev->spi = spi;

    /* Initialize locks and semaphores */

    nxmutex_init(&dev->lock);
    nxsem_init(&dev->rx_sem, 0, 0);
    nxsem_init(&dev->tx_sem, 0, 0);

    lr1121_attachirq0(dev, lr1121_irq0handler, dev);

    (void)register_driver(path, &lr1121_ops, 0666,
                          dev);
}
