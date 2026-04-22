/****************************************************************************
 * include/nuttx/wireless/lpwan/lr1121.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_LR1121_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_LR1121_H

/* This version is currently experimental.
 * Breaking changes might happen in the near future.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define LR1121_RX_PAYLOAD_SIZE 0xff

/* IOCTL commands ***********************************************************/

/* arg: lr1121_packet_type_e */

#define LR1121IOC_PACKETTYPESET _WLCIOC(LR1121_FIRST + 0) //

/* Sets lora parameters. arg: lr1121_lora_config_s *config */

#define LR1121IOC_LORACONFIGSET _WLCIOC(LR1121_FIRST + 1)

/* IRQ Register bits ********************************************************/

#define LR1121_IRQ_TXDONE_MASK (1 << 2)
#define LR1121_IRQ_RXDONE_MASK (1 << 3)
#define LR1121_IRQ_PREAMBLEDETECTED_MASK (1 << 4)
#define LR1121_IRQ_SYNCWORDVALID_MASK (1 << 5)
#define LR1121_IRQ_HEADERVALID_MASK (1 << 5)
#define LR1121_IRQ_HEADERERR_MASK (1 << 6)
#define LR1121_IRQ_CRCERR_MASK (1 << 7)
#define LR1121_IRQ_CADDONE_MASK (1 << 8)
#define LR1121_IRQ_CADDETECTED_MASK (1 << 9)
#define LR1121_IRQ_TIMEOUT_MASK (1 << 10)
#define LR1121_IRQ_LRFHSSHOP_MASK (1 << 11)
#define LR1121_IRQ_LBD_MASK (1 << 21)
#define LR1121_IRQ_CMDERROR_MASK (1 << 22)
#define LR1121_IRQ_ERROR_MASK (1 << 23)
#define LR1121_IRQ_FSKLENERROR_MASK (1 << 24)
#define LR1121_IRQ_FSKADDRERROR_MASK (1 << 25)

/* Others */

#define LR1121_NOP 0
#define LR1121_NO_TIMEOUT 0
#define LR1121_NO_DELAY 0

/* Oscillators and PLLs */

#define LR1121_OSC_MAIN_HZ (32000000)
#define LR1121_FXTAL LR1121_OSC_MAIN_HZ
#define LR1121_PLL_STEP_SHIFT_AMOUNT (14)
#define LR1121_PLL_STEP_SCALED (LR1121_FXTAL >> (25 - LR1121_PLL_STEP_SHIFT_AMOUNT))

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/* Standby config */

enum lr1121_standby_mode_e
{
  LR1121_STDBY_RC = 0x00,
  LR1121_STDBY_XOSC = 0x01
};

/* Packet Types */

enum lr1121_packet_type_e
{
  LR1121_PACKETTYPE_GFSK = 0x00,
  LR1121_PACKETTYPE_LORA = 0x01,
  LR1121_PACKETTYPE_LR_FHSS = 0x13
};

/* Ramp times */

enum lr1121_ramp_time_e
{
  LR1121_SET_RAMP_10U = 0x00,
  LR1121_SET_RAMP_20U = 0x01,
  LR1121_SET_RAMP_40U = 0x02,
  LR1121_SET_RAMP_80U = 0x03,
  LR1121_SET_RAMP_200U = 0x04,
  LR1121_SET_RAMP_800U = 0x05,
  LR1121_SET_RAMP_1700U = 0x06,
  LR1121_SET_RAMP_3400U = 0x07
};

/* GFSK Pulse shapes */

enum lr1121_gfsk_pulseshape_e
{
  LR1121_GFSK_PULSESHAPE_NONE = 0x00,
  LR1121_GFSK_PULSESHAPE_GAUSSIAN_BT_0_3 = 0x08,
  LR1121_GFSK_PULSESHAPE_GAUSSIAN_BT_0_5 = 0x09,
  LR1121_GFSK_PULSESHAPE_GAUSSIAN_BT_0_7 = 0x0a,
  LR1121_GFSK_PULSESHAPE_GAUSSIAN_BT_1 = 0x0b
};

/* GFSK Bandwidths in Hz */

enum lr1121_gfsk_bandwidth_e
{
  LR1121_GFSK_BANDWIDTH_4800HZ = 0x1f,
  LR1121_GFSK_BANDWIDTH_5800HZ = 0x17,
  LR1121_GFSK_BANDWIDTH_7300HZ = 0x0f,
  LR1121_GFSK_BANDWIDTH_9700HZ = 0x1e,
  LR1121_GFSK_BANDWIDTH_11700HZ = 0x16,
  LR1121_GFSK_BANDWIDTH_14600HZ = 0x0e,
  LR1121_GFSK_BANDWIDTH_19500HZ = 0x1d,
  LR1121_GFSK_BANDWIDTH_23400HZ = 0x15,
  LR1121_GFSK_BANDWIDTH_29300HZ = 0x0d,
  LR1121_GFSK_BANDWIDTH_39000HZ = 0x1c,
  LR1121_GFSK_BANDWIDTH_46900HZ = 0x14,
  LR1121_GFSK_BANDWIDTH_58600HZ = 0x0c,
  LR1121_GFSK_BANDWIDTH_78200HZ = 0x1b,
  LR1121_GFSK_BANDWIDTH_93800HZ = 0x13,
  LR1121_GFSK_BANDWIDTH_117300HZ = 0x0b,
  LR1121_GFSK_BANDWIDTH_156200HZ = 0x1a,
  LR1121_GFSK_BANDWIDTH_187200HZ = 0x12,
  LR1121_GFSK_BANDWIDTH_234300HZ = 0x0a,
  LR1121_GFSK_BANDWIDTH_312000HZ = 0x19,
  LR1121_GFSK_BANDWIDTH_373600HZ = 0x11,
  LR1121_GFSK_BANDWIDTH_467000HZ = 0x09
};

/* LoRa Spreading Factors */

enum lr1121_lora_sf_e
{
  LR1121_LORA_SF5 = 0x05,
  LR1121_LORA_SF6 = 0x06,
  LR1121_LORA_SF7 = 0x07,
  LR1121_LORA_SF8 = 0x08,
  LR1121_LORA_SF9 = 0x09,
  LR1121_LORA_SF10 = 0x0a,
  LR1121_LORA_SF11 = 0x0b,
  LR1121_LORA_SF12 = 0x0c
};

/* LoRa Bandwidths */

enum lr1121_lora_bw_e
{
  LR1121_LORA_BW_7   = 0x00,
  LR1121_LORA_BW_10  = 0x08,
  LR1121_LORA_BW_15  = 0x01,
  LR1121_LORA_BW_20  = 0x09,
  LR1121_LORA_BW_31  = 0x02,
  LR1121_LORA_BW_41  = 0x0a,
  LR1121_LORA_BW_62  = 0x03,
  LR1121_LORA_BW_125 = 0x04,
  LR1121_LORA_BW_250 = 0x05,
  LR1121_LORA_BW_500 = 0x06,
  LR1121_LORA_BW203  = 0x0D,
  LR1121_LORA_BW406  = 0x0E,
  LR1121_LORA_BW812  = 0x0F
};

/* LoRa Coding Rates */

enum lr1121_lora_cr_e
{
  LR1121_LORA_CR_4_5 = 0x01,
  LR1121_LORA_CR_4_6 = 0x02,
  LR1121_LORA_CR_4_7 = 0x03,
  LR1121_LORA_CR_4_8 = 0x04
};

/* CAD Exit modes */

enum lr1121_cad_exit_mode_e
{
  LR1121_CAD_ONLY = 0x00,
  LR1121_CAD_RX = 0x01,
  LR1121_CAD_LBT = 0x10
};

/* TCXO voltages */

enum lr1121_tcxo_voltage_e
{
  LR1121_TCXO_1_6V = 0x00,
  LR1121_TCXO_1_7V = 0x01,
  LR1121_TCXO_1_8V = 0x02,
  LR1121_TCXO_2_2V = 0x03,
  LR1121_TCXO_2_4V = 0x04,
  LR1121_TCXO_2_7V = 0x05,
  LR1121_TCXO_3_0V = 0x06,
  LR1121_TCXO_3_3V = 0x07
};

/* Fallback modes */

enum lr1121_fallback_mode_e
{
  LR1121_FALLBACK_FS = 0x03,
  LR1121_FALLBACK_STDBY_XOSC = 0x02,
  LR1121_FALLBACK_STDBY_RC = 0x01
};

/* Regulator modes */

enum lr1121_regulator_mode_e
{
  LR1121_LDO = 0x00,
  LR1121_DC_DC_LDO = 0x01
};

/* Device */

enum lr1121_device_e
{
  SX1261 = 0x01,
  LR1121 = 0x00
};

enum lr1121_gfsk_preamble_detect_e
{
  LR1121_GFSK_PREAMBLE_DETECT_OFF,
  LR1121_GFSK_PREAMBLE_DETECT_8B,
  LR1121_GFSK_PREAMBLE_DETECT_16B,
  LR1121_GFSK_PREAMBLE_DETECT_24B,
  LR1121_GFSK_PREAMBLE_DETECT_32B
};

/* Addr comp */

enum lr1121_address_filtering_e
{
  LR1121_ADDR_FILT_DISABLED,
  LR1121_ADDR_FILT_NODE,
  LR1121_ADDR_FILT_NODE_BROADCAST
};

/* GFSK CRC types */

enum lr1121_gfsk_crc_type_e
{
  LR1121_GFSK_CRCTYPE_OFF = 0x01,
  LR1121_GFSK_CRCTYPE_1_BYTE = 0x00,
  LR1121_GFSK_CRCTYPE_2_BYTE = 0x02,
  LR1121_GFSK_CRCTYPE_1_BYTE_INV = 0x04,
  LR1121_GFSK_CRCTYPE_2_BYTE_INV = 0x06
};

/* PA Selection */
enum lr1121_pa_sel_e
{
  LR1121_PA_LOW,
  LR1121_PA_HIGH,
  LR1121_PA_HIGH_FREQ
};

/* PA Power Source */
enum lr1121_pa_reg_supply_e
{
  LR1121_PA_REG_SUPPLY_INTERNAL,
  LR1121_PA_REG_SUPPLY_VBAT
};

/* LoRa mod params */

struct lr1121_modparams_lora_s
{
  enum lr1121_lora_sf_e spreading_factor;
  enum lr1121_lora_bw_e bandwidth;
  enum lr1121_lora_cr_e coding_rate;
  bool low_datarate_optimization;
};

/* GFSK mod params */

struct lr1121_modparams_gfsk_s
{
  uint32_t bitrate;
  enum lr1121_gfsk_pulseshape_e pulseshape;
  enum lr1121_gfsk_bandwidth_e bandwidth;
  uint32_t frequency_deviation;
};

/* LoRa packet params */

struct lr1121_packetparams_lora_s
{
  uint16_t preambles;
  bool fixed_length_header;
  uint8_t payload_length;
  bool crc_enable;
  bool invert_iq;
};

/* GFSK packet params */

struct lr1121_packetparams_gfsk_s
{
  uint16_t preambles;
  enum lr1121_gfsk_preamble_detect_e preamble_detect;
  uint8_t syncword_length;
  enum lr1121_address_filtering_e address_filtering;
  bool include_packet_size;
  uint8_t packet_length;
  enum lr1121_gfsk_crc_type_e crc_type;
  bool whitening_enable;
};

/* Config */

struct lr1121_lora_config_s
{
  struct lr1121_modparams_lora_s modulation;
  struct lr1121_packetparams_lora_s packet;
};

/* Lower driver *************************************************************/

struct lr1121_irq_masks
{
  uint16_t dio1_mask;
  uint16_t dio2_mask;
  uint16_t dio3_mask;
};

struct lr1121_lower_s
{
  /* Index of radio to register.
   * ex: 0 is the primary radio, 1 is the secondary.
   * Must be within the maximum configured radios.
   */

  unsigned int dev_number;
  CODE void (*reset)(void);

  /* This controls which DIO reacts to interrupts
   * Depended on the pinout of the board / module.
   * Note that DIO 2 and DIO 3 can be already in use
   * by the module and setting them might interfere
   * with the operation or even damage them.
   */

  struct lr1121_irq_masks masks;
  enum lr1121_tcxo_voltage_e dio3_voltage;
  uint32_t dio3_delay;
  uint8_t use_dio2_as_rf_sw;

  /* Interrupt attachments. These should be
   * connected to one of the DIOx pins
   */

  CODE int (*irq0attach)(xcpt_t handler, FAR void *arg);

  /* The regulator mode is board / module depended */

  enum lr1121_regulator_mode_e regulator_mode;

  /* Power amplifier control. DO NOT exceeds the
   * limits listed in SX1261-2 V2 datasheet.
   * 13.1.14 SetPaConfig
   * This can cause damage to the device.
   */

  CODE int (*get_pa_values)(uint8_t *dutyCycle,
                            enum lr1121_pa_sel_e *pa_sel,
                            enum lr1121_pa_reg_supply_e *reg_supply,
                            uint8_t *hp_sel);

  /* TX power control. Depending on the local RF regulations,
   * power might have to be limited.
   * Also depending on board or module,
   * power values have different charactersitics.
   * More info in sx1261-2 V2 datasheet 13.4.4 SetTxParams.
   * uint8_t *power is set and this function may limit it.
   */

  CODE int (*limit_tx_power)(uint8_t *current_power);

  enum lr1121_ramp_time_e tx_ramp_time;

  /* Frequency control
   * Typically boards have a limited range of frequencies.
   * Exceeding these can damage the radio.
   * Also depending on regulations, some frequencies are restricted.
   * This must return non zero in case a frequency is denied.
   */

  CODE int (*check_frequency)(uint32_t frequency);
};

/* Upper ********************************************************************/

struct lr1121_read_header_s
{
  uint8_t payload_length;
  int32_t snr;
  int16_t rssi_db;
  uint8_t payload[LR1121_RX_PAYLOAD_SIZE];
  uint8_t crc_error;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void lr1121_register(FAR struct spi_dev_s *spi,
                     FAR const struct lr1121_lower_s *lower,
                     const char *path);

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_LR1121_H */ 
/**************************************************************************** \
* include/nuttx/wireless/lpwan/lr1121.h                                      \
*                                                                            \
* SPDX-License-Identifier: Apache-2.0                                        \
*                                                                            \
* Licensed to the Apache Software Foundation (ASF) under one or more         \
* contributor license agreements.  See the NOTICE file distributed with      \
* this work for additional information regarding copyright ownership.  The   \
* ASF licenses this file to you under the Apache License, Version 2.0 (the   \
* "License"); you may not use this file except in compliance with the        \
* License.  You may obtain a copy of the License at                          \
*                                                                            \
*   http://www.apache.org/licenses/LICENSE-2.0                               \
*                                                                            \
* Unless required by applicable law or agreed to in writing, software        \
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT  \
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the  \
* License for the specific language governing permissions and limitations    \
* under the License.                                                         \
*                                                                            \
****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_LR1121_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_LR1121_H

/* This version is currently experimental.
 * Breaking changes might happen in the near future.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define LR1121_RX_PAYLOAD_SIZE 0xff

/* IOCTL commands ***********************************************************/

/* arg: lr1121_packet_type_e */

#define LR1121IOC_PACKETTYPESET _WLCIOC(LR1121_FIRST + 0) //

/* Sets lora parameters. arg: lr1121_lora_config_s *config */

#define LR1121IOC_LORACONFIGSET _WLCIOC(LR1121_FIRST + 1)

/* IRQ Register bits ********************************************************/

#define LR1121_IRQ_TXDONE_MASK (1 << 0)
#define LR1121_IRQ_RXDONE_MASK (1 << 1)
#define LR1121_IRQ_PREAMBLEDETECTED_MASK (1 << 2)
#define LR1121_IRQ_SYNCWORDVALID_MASK (1 << 3)
#define LR1121_IRQ_HEADERVALID_MASK (1 << 4)
#define LR1121_IRQ_HEADERERR_MASK (1 << 5)
#define LR1121_IRQ_CRCERR_MASK (1 << 6)
#define LR1121_IRQ_CADDONE_MASK (1 << 7)
#define LR1121_IRQ_CADDETECTED_MASK (1 << 8)
#define LR1121_IRQ_TIMEOUT_MASK (1 << 9)
#define LR1121_IRQ_LRFHSSHOP_MASK (1 << 14)

/* Others */

#define LR1121_NOP 0
#define LR1121_NO_TIMEOUT 0
#define LR1121_NO_DELAY 0

/* Oscillators and PLLs */

#define LR1121_OSC_MAIN_HZ (32000000)
#define LR1121_FXTAL LR1121_OSC_MAIN_HZ
#define LR1121_PLL_STEP_SHIFT_AMOUNT (14)
#define LR1121_PLL_STEP_SCALED (LR1121_FXTAL >> (25 - LR1121_PLL_STEP_SHIFT_AMOUNT))

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/* Standby config */

enum lr1121_standby_mode_e
{
  LR1121_STDBY_RC = 0x00,
  LR1121_STDBY_XOSC = 0x01
};

/* Packet Types */

enum lr1121_packet_type_e
{
  LR1121_PACKETTYPE_NONE = 0x00,
  LR1121_PACKETTYPE_GFSK = 0x01,
  LR1121_PACKETTYPE_LORA = 0x02,
  LR1121_PACKETTYPE_LR_FHSS = 0x04
};

/* Ramp times */

enum lr1121_ramp_time_e
{
  LR1121_SET_RAMP_16U = 0x00,
  LR1121_SET_RAMP_32U = 0x01,
  LR1121_SET_RAMP_48U = 0x02,
  LR1121_SET_RAMP_64U = 0x03,
  LR1121_SET_RAMP_80U = 0x04,
  LR1121_SET_RAMP_96U = 0x05,
  LR1121_SET_RAMP_112U = 0x06,
  LR1121_SET_RAMP_128U = 0x07,
  LR1121_SET_RAMP_144U = 0x08,
  LR1121_SET_RAMP_160U = 0x09,
  LR1121_SET_RAMP_176U = 0x0A,
  LR1121_SET_RAMP_192U = 0x0B,
  LR1121_SET_RAMP_208U = 0x0C,
  LR1121_SET_RAMP_240U = 0x0D,
  LR1121_SET_RAMP_272U = 0x0E,
  LR1121_SET_RAMP_304U = 0x0F
};

/* GFSK Pulse shapes */

enum lr1121_gfsk_pulseshape_e
{
  LR1121_GFSK_PULSESHAPE_NONE = 0x00,
  LR1121_GFSK_PULSESHAPE_GAUSSIAN_BT_0_3 = 0x08,
  LR1121_GFSK_PULSESHAPE_GAUSSIAN_BT_0_5 = 0x09,
  LR1121_GFSK_PULSESHAPE_GAUSSIAN_BT_0_7 = 0x0a,
  LR1121_GFSK_PULSESHAPE_GAUSSIAN_BT_1 = 0x0b
};

/* GFSK Bandwidths in Hz */

enum lr1121_gfsk_bandwidth_e
{
  LR1121_GFSK_BANDWIDTH_4800HZ = 0x1f,
  LR1121_GFSK_BANDWIDTH_5800HZ = 0x17,
  LR1121_GFSK_BANDWIDTH_7300HZ = 0x0f,
  LR1121_GFSK_BANDWIDTH_9700HZ = 0x1e,
  LR1121_GFSK_BANDWIDTH_11700HZ = 0x16,
  LR1121_GFSK_BANDWIDTH_14600HZ = 0x0e,
  LR1121_GFSK_BANDWIDTH_19500HZ = 0x1d,
  LR1121_GFSK_BANDWIDTH_23400HZ = 0x15,
  LR1121_GFSK_BANDWIDTH_29300HZ = 0x0d,
  LR1121_GFSK_BANDWIDTH_39000HZ = 0x1c,
  LR1121_GFSK_BANDWIDTH_46900HZ = 0x14,
  LR1121_GFSK_BANDWIDTH_58600HZ = 0x0c,
  LR1121_GFSK_BANDWIDTH_78200HZ = 0x1b,
  LR1121_GFSK_BANDWIDTH_93800HZ = 0x13,
  LR1121_GFSK_BANDWIDTH_117300HZ = 0x0b,
  LR1121_GFSK_BANDWIDTH_156200HZ = 0x1a,
  LR1121_GFSK_BANDWIDTH_187200HZ = 0x12,
  LR1121_GFSK_BANDWIDTH_234300HZ = 0x0a,
  LR1121_GFSK_BANDWIDTH_312000HZ = 0x19,
  LR1121_GFSK_BANDWIDTH_373600HZ = 0x11,
  LR1121_GFSK_BANDWIDTH_467000HZ = 0x09
};

/* LoRa Spreading Factors */

enum lr1121_lora_sf_e
{
  LR1121_LORA_SF5 = 0x05,
  LR1121_LORA_SF6 = 0x06,
  LR1121_LORA_SF7 = 0x07,
  LR1121_LORA_SF8 = 0x08,
  LR1121_LORA_SF9 = 0x09,
  LR1121_LORA_SF10 = 0x0a,
  LR1121_LORA_SF11 = 0x0b,
  LR1121_LORA_SF12 = 0x0c
};

/* LoRa Bandwidths */

enum lr1121_lora_bw_e
{
  LR1121_LORA_BW_7 = 0x00,
  LR1121_LORA_BW_10 = 0x08,
  LR1121_LORA_BW_15 = 0x01,
  LR1121_LORA_BW_20 = 0x09,
  LR1121_LORA_BW_31 = 0x02,
  LR1121_LORA_BW_41 = 0x0a,
  LR1121_LORA_BW_62 = 0x03,
  LR1121_LORA_BW_125 = 0x04,
  LR1121_LORA_BW_250 = 0x05,
  LR1121_LORA_BW_500 = 0x06
};

/* LoRa Coding Rates */

enum lr1121_lora_cr_e
{
  LR1121_LORA_CR_4_5 = 0x01,
  LR1121_LORA_CR_4_6 = 0x02,
  LR1121_LORA_CR_4_7 = 0x03,
  LR1121_LORA_CR_4_8 = 0x04
};

/* CAD Exit modes */

enum lr1121_cad_exit_mode_e
{
  LR1121_CAD_ONLY = 0x00,
  LR1121_CAD_RX = 0x01
};

/* TCXO voltages */

enum lr1121_tcxo_voltage_e
{
  LR1121_TCXO_1_6V = 0x00,
  LR1121_TCXO_1_7V = 0x01,
  LR1121_TCXO_1_8V = 0x02,
  LR1121_TCXO_2_2V = 0x03,
  LR1121_TCXO_2_4V = 0x04,
  LR1121_TCXO_2_7V = 0x05,
  LR1121_TCXO_3_0V = 0x06,
  LR1121_TCXO_3_3V = 0x07
};

/* Fallback modes */

enum lr1121_fallback_mode_e
{
  LR1121_FALLBACK_FS = 0x40,
  LR1121_FALLBACK_STDBY_XOSC = 0x30,
  LR1121_FALLBACK_STDBY_RC = 0x20
};

/* Regulator modes */

enum lr1121_regulator_mode_e
{
  LR1121_LDO = 0x00,
  LR1121_DC_DC_LDO = 0x01
};

/* Device */

enum lr1121_device_e
{
  SX1261 = 0x01,
  SX1262 = 0x00
};

enum lr1121_gfsk_preamble_detect_e
{
  LR1121_GFSK_PREAMBLE_DETECT_OFF = 0x00,
  LR1121_GFSK_PREAMBLE_DETECT_8B = 0x04,
  LR1121_GFSK_PREAMBLE_DETECT_16B = 0x05,
  LR1121_GFSK_PREAMBLE_DETECT_24B = 0x06,
  LR1121_GFSK_PREAMBLE_DETECT_32B = 0x07
};

/* Addr comp */

enum lr1121_address_filtering_e
{
  LR1121_ADDR_FILT_DISABLED,
  LR1121_ADDR_FILT_NODE,
  LR1121_ADDR_FILT_NODE_BROADCAST
};

/* GFSK CRC types */

enum lr1121_gfsk_crc_type_e
{
  LR1121_GFSK_CRCTYPE_OFF = 0x01,
  LR1121_GFSK_CRCTYPE_1_BYTE = 0x00,
  LR1121_GFSK_CRCTYPE_2_BYTE = 0x02,
  LR1121_GFSK_CRCTYPE_1_BYTE_INV = 0x04,
  LR1121_GFSK_CRCTYPE_2_BYTE_INV = 0x06
};

/* LoRa mod params */

struct lr1121_modparams_lora_s
{
  enum lr1121_lora_sf_e spreading_factor;
  enum lr1121_lora_bw_e bandwidth;
  enum lr1121_lora_cr_e coding_rate;
  bool low_datarate_optimization;
};

/* GFSK mod params */

struct lr1121_modparams_gfsk_s
{
  uint32_t bitrate;
  enum lr1121_gfsk_pulseshape_e pulseshape;
  enum lr1121_gfsk_bandwidth_e bandwidth;
  uint32_t frequency_deviation;
};

/* LoRa packet params */

struct lr1121_packetparams_lora_s
{
  uint16_t preambles;
  bool fixed_length_header;
  uint8_t payload_length;
  bool crc_enable;
  bool invert_iq;
};

/* GFSK packet params */

struct lr1121_packetparams_gfsk_s
{
  uint16_t preambles;
  enum lr1121_gfsk_preamble_detect_e preamble_detect;
  uint8_t syncword_length;
  enum lr1121_address_filtering_e address_filtering;
  bool include_packet_size;
  uint8_t packet_length;
  enum lr1121_gfsk_crc_type_e crc_type;
  bool whitening_enable;
};

/* Config */

struct lr1121_lora_config_s
{
  struct lr1121_modparams_lora_s modulation;
  struct lr1121_packetparams_lora_s packet;
};

/* Lower driver *************************************************************/

struct lr1121_irq_masks
{
  uint16_t dio1_mask;
  uint16_t dio2_mask;
  uint16_t dio3_mask;
};

struct lr1121_lower_s
{
  /* Index of radio to register.
   * ex: 0 is the primary radio, 1 is the secondary.
   * Must be within the maximum configured radios.
   */

  unsigned int dev_number;
  CODE void (*reset)(void);

  /* This controls which DIO reacts to interrupts
   * Depended on the pinout of the board / module.
   * Note that DIO 2 and DIO 3 can be already in use
   * by the module and setting them might interfere
   * with the operation or even damage them.
   */

  struct lr1121_irq_masks masks;
  enum lr1121_tcxo_voltage_e dio3_voltage;
  uint32_t dio3_delay;
  uint8_t use_dio2_as_rf_sw;

  /* Interrupt attachments. These should be
   * connected to one of the DIOx pins
   */

  CODE int (*irq0attach)(xcpt_t handler, FAR void *arg);

  /* The regulator mode is board / module depended */

  enum lr1121_regulator_mode_e regulator_mode;

  /* Power amplifier control. DO NOT exceeds the
   * limits listed in SX1261-2 V2 datasheet.
   * 13.1.14 SetPaConfig
   * This can cause damage to the device.
   */

  CODE int (*get_pa_values)(enum lr1121_device_e *model,
                            uint8_t *hpmax, uint8_t *padutycycle);

  /* TX power control. Depending on the local RF regulations,
   * power might have to be limited.
   * Also depending on board or module,
   * power values have different charactersitics.
   * More info in sx1261-2 V2 datasheet 13.4.4 SetTxParams.
   * uint8_t *power is set and this function may limit it.
   */

  CODE int (*limit_tx_power)(uint8_t *current_power);

  enum lr1121_ramp_time_e tx_ramp_time;

  /* Frequency control
   * Typically boards have a limited range of frequencies.
   * Exceeding these can damage the radio.
   * Also depending on regulations, some frequencies are restricted.
   * This must return non zero in case a frequency is denied.
   */

  CODE int (*check_frequency)(uint32_t frequency);
};

/* Upper ********************************************************************/

struct lr1121_read_header_s
{
  uint8_t payload_length;
  int32_t snr;
  int16_t rssi_db;
  uint8_t payload[LR1121_RX_PAYLOAD_SIZE];
  uint8_t crc_error;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void lr1121_register(FAR struct spi_dev_s *spi,
                     FAR const struct lr1121_lower_s *lower,
                     const char *path);

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_LR1121_H */