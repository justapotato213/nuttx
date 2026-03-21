/****************************************************************************
 * drivers/wireless/lpwan/LR1121/LR1121.h
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

/* Currently this is an experimental version.
 * Breaking changes might happen in the near future.
 * All functions and definitions are accurately recreated
 * from the official datasheet "DS_SX1261-2_V2_1"
 */

#ifndef __DRIVERS_WIRELESS_LPWAN_LR1121_LR1121_H
#define __DRIVERS_WIRELESS_LPWAN_LR1121_LR1121_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <endian.h>

/****************************************************************************
 * Settings
 ****************************************************************************/

/* Driver settings */

#define LR1121_MAX_DEVICES 2
#define LR1121_SPI_SPEED 500000

/* LoRa defaults */

#define LR1121_DEFAULT_LORA_SF LR1121_LORA_SF10
#define LR1121_DEFAULT_LORA_BW LR1121_LORA_BW_125
#define LR1121_DEFAULT_LORA_CR LR1121_LORA_CR_4_8
#define LR1121_DEFAULT_LORA_CRC_EN true
#define LR1121_DEFAULT_LORA_FIXED_HEADER false
#define LR1121_DEFAULT_LORA_PREAMBLES 12
#define LR1121_DEFAULT_LORA_LDO false

/* Common defaults */

#define LR1121_DEFAULT_FREQ 869525000
#define LR1121_DEFAULT_POWER 0x0e
#define LR1121_DEFAULT_PACKET_TYPE LR1121_PACKETTYPE_LORA
#define LR1121_DEFAULT_SYNCWORD {0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

/* Hardware defaults */

#define LR1121_DEFAULT_INVERT_IQ false

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register addresses *******************************************************/

#define LR1121_REG_SYNCWORD 0x06C0      /* Start of Byte 0 - Byte 7 */
#define LR1121_REG_SYNCWORD_LEN 8       /* Bytes */
#define LR1121_REG_NODEADDR 0x06CD      /* Node address to filter. Default 0x00 */
#define LR1121_REG_BRDCASTADDR 0x06CE   /* Broadcast address to filter. Default 0x00 */
#define LR1121_REG_CRC_INIT_MSB 0x06BC  /* Default 0x1D */
#define LR1121_REG_CRC_INIT_LSB 0x06BD  /* Default 0x0F */
#define LR1121_REG_CRC_POLY_MSB 0x06BE  /* Default 0x10 */
#define LR1121_REG_CRC_POLY_LSB 0x06BF  /* Default 0x21 */
#define LR1121_REG_WHITENING_MSB 0x06B8 /* Default 0x01 */
#define LR1121_REG_WHITENING_LSB 0x06B9 /* Default 0x00 */

/* Enum and constant definitions ********************************************/

/* Packet parameters    *****************************************************/

#define LR1121_PKTPARAM1_GFSK_PREAMBLELEN_PARAM 0 /* Takes 0x0001 to 0xFFFF preamble bits */
#define LR1121_PKTPARAM1_GFSK_PREAMBLELEN_PARAMS 2
#define LR1121_PKTPARAM3_GFSK_PREAMBLEDETECTORLEN_PARAM 2 /* Takes LR1121_GFSK_PREABMLE_DETECT_x */
#define LR1121_PKTPARAM4_GFSK_SYNCWORDLEN_PARAM 3         /* 0x00 to 0x40, 0 to 8 bytes from LR1121_SYNCWORD register */
#define LR1121_PKTPARAM5_GFSK_ADDRCOMP_PARAM 4            /* Takes LR1121_ADDR_FILT_x */
#define LR1121_PKTPARAM6_GFSK_PKTTYPE_PARAM 5             /* Packet length unknown? false/true */
#define LR1121_PKTPARAM7_GFSK_PAYLOADLEN_PARAM 6          /* 0x00 to 0xFF size of tx&rx payload in bytes */
#define LR1121_PKTPARAM8_GFSK_CRCTYPE_PARAM 7             /* Takes LR1121_GFSK_CRCTYPE_x */
#define LR1121_PKTPARAM9_GFSK_WHITENING_PARAM 8           /* Enable false/true */

#define LR1121_PKTPARAM1_LORA_PREAMBLELEN_PARAM 0 /* 0x0001 to 0xFFFF preamble symbols */
#define LR1121_PKTPARAM1_LORA_PREAMBLELEN_PARAMS 2
#define LR1121_PKTPARAM3_LORA_HEADERTYPE_PARAM 2 /* Variable/Fixed length false/true */
#define LR1121_PKTPARAM4_LORA_PAYLOADLEN_PARAM 3 /* 0x00 to 0xFF number of tx and rx bytes */
#define LR1121_PKTPARAM5_LORA_CRCTYPE_PARAM 4    /* CRC OFF/ON false/true */
#define LR1121_PKTPARAM6_LORA_INVERTIQ_PARAM 5   /* Standard/Inverted false/true */

/* Modulation parameters ****************************************************/

/* GFSK */

#define LR1121_MODPARAM1_GFSK_BR_PARAM 0 /* Takes br = 32 * Fxtal / bit_rate */
#define LR1121_MODPARAM1_GFSK_BR_PARAMS 3
#define LR1121_MODPARAM4_GFSK_PULSESHAPE_PARAM 3 /* Takes LR1121_GFSK_PULSESHAPE_x */
#define LR1121_MODPARAM5_GFSK_BANDWIDTH_PARAM 4  /* Takes LR1121_GFSK_BANDWIDTH_xHZ */
#define LR1121_MODPARAM6_GFSK_FDEV_PARAM 5       /* Takes Fdev = (frequency_deviation * 2^25) / Fxtal */
#define LR1121_MODPARAM6_GFSK_FDEV_PARAMS 3

/* LoRa */

#define LR1121_MODPARAM1_LORA_SF_PARAM 0              /* Takes SF between LR1121_LORA_SF_MIN and MAX */
#define LR1121_MODPARAM2_LORA_BW_PARAM 1              /* Takes LR1121_LORA_BW_x */
#define LR1121_MODPARAM3_LORA_CR_PARAM 2              /* Takes LR1121_LORA_CR_x */
#define LR1121_MODPARAM4_LORA_LOWDATRATE_OPTI_PARAM 3 /* Takes true/false*/

/* Operational modes functions **********************************************/

/* SetSleep */

#define LR1121_SETSLEEP 0x011B /* Opcode */
#define LR1121_SETSLEEP_PARAMS 1
#define LR1121_SETSLEEP_CONF_PARAM 0
#define LR1121_SETSLEEP_CONF_RTC_SHIFT 0
#define LR1121_SETSLEEP_CONF_RTC_DISABLE (0 << LR1121_SETSLEEP_CONF_RTC_SHIFT)
#define LR1121_SETSLEEP_CONF_RTC_ENABLE (1 << LR1121_SETSLEEP_CONF_RTC_SHIFT)
#define LR1121_SETSLEEP_CONF_START_SHIFT 2
#define LR1121_SETSLEEP_CONF_START_COLD (0 << LR1121_SETSLEEP_CONF_START_SHIFT)
#define LR1121_SETSLEEP_CONF_START_WARM (1 << LR1121_SETSLEEP_CONF_START_SHIFT)

/* SetStandby */

#define LR1121_SETSTANDBY 0x011C /* Opcode */
#define LR1121_SETSTANDBY_PARAMS 1
#define LR1121_SETSTANDBY_CONF_PARAM 0
#define LR1121_SETSTANDBY_CONF_SHIFT 0 /* Bit 0-1: STDBY mode */
#define LR1121_SETSTANDBY_CONF_RC (0 << LR1121_SETSTANDBY_CONF_SHIFT)
#define LR1121_SETSTANDBY_CONF_XOSC (1 << LR1121_SETSTANDBY_CONF_SHIFT)

/* SetFS */

#define LR1121_SETFS 0x011D /* Opcode */

/* SetTX */

#define LR1121_SETTX 0x020A /* Opcode */
#define LR1121_SETTX_PARAMS 2
#define LR1121_SETTX_TIMEOUT_PARAM 0
#define LR1121_SETTX_TIMEOUT_PARAMS 2
#define LR1121_SETTX_NO_TIMEOUT 0x000000 /* Constant */

/* SetRX */

#define LR1121_SETRX 0x0209 /* Opcode */
#define LR1121_SETRX_PARAMS 2
#define LR1121_SETRX_TIMEOUT_PARAM 0
#define LR1121_SETRX_TIMEOUT_PARAMS 2
#define LR1121_SETRX_NO_TIMEOUT 0x000000 /* Constant */
#define LR1121_SETRX_CONTINUOUS 0xFFFFFF /* Constant */

/* StopTimerOnPreamble */

#define LR1121_STOPTIMERONPREAMBLE 0x127 /* Opcode */
#define LR1121_STOPTIMERONPREAMBLE_PARAMS 1
#define LR1121_STOPTIMERONPREAMBLE_ENABLE (1 << 0) /* Bit 0 disabled=stop on syncword/header, enabled=stop on preamble */

/* SetRxDutyCycle */

#define LR1121_SETRXDUTYCYCLE 0x0214 /* Opcode */
#define LR1121_SETRXDUTYCYCLE_PARAMS 6
#define LR1121_SETRXDUTYCYCLE_RXPERIOD_PARAM 0
#define LR1121_SETRXDUTYCYCLE_RXPERIOD_PARAMS 3
#define LR1121_SETRXDUTYCYCLE_SLEEPPERIOD_PARAM 3
#define LR1121_SETRXDUTYCYCLE_SLEEPPERIOD_PARAMS 3

/* SetCAD */

#define LR1121_SETCAD 0x0218

/* SetTXContinuousWave */

#define LR1121_SETTXCONTINUOUSWAVE 0x0219

/* SetTXInfinitePreamble */

#define LR1121_SETTXINFINITEPREAMBLE 0x021A

/* SetRegulatorMode */

#define LR1121_SETREGULATORMODE 0x0110
#define LR1121_SETREGULATORMODE_PARAMS 1
#define LR1121_SETREGULATORMODE_PARAM 0

/* Calibrate Function */

#define LR1121_CALIBRATE 0x010F
#define LR1121_CALIBRATE_RC64K_EN (1 << 0)
#define LR1121_CALIBRATE_RC13M_EN (1 << 1)
#define LR1121_CALIBRATE_PLL_EN (1 << 2)
#define LR1121_CALIBRATE_ADC_PULSE_EN (1 << 3)
#define LR1121_CALIBRATE_ADC_BULK_N_EN (1 << 4)
#define LR1121_CALIBRATE_ADC_BULK_P_EN (1 << 5)
#define LR1121_CALIBRATE_IMAGE_EN (1 << 6)

/* CalibrateImage */

#define LR1121_CALIBRATEIMAGE 0x0111
#define LR1121_CALIBRATEIMAGE_FREQ1_PARAM 0
#define LR1121_CALIBRATEIMAGE_FREQ2_PARAM 1

/* SetPAConfig */

#define LR1121_SETPACONFIG 0x0215
#define LR1121_SETPACONFIG_PARMS 4
#define LR1121_SETPACONFIG_PADUTYCYCLE_PARAM 0
#define LR1121_SETPACONFIG_HPMAX_PARAM 1
#define LR1121_SETPACONFIG_DEVICESEL_PARAM 2
#define LR1121_SETPACONFIG_PALUT_PARAM 3

/* SetRXTXFallbackMode */

#define LR1121_SETRXTXFALLBACKMODE 0x0213
#define LR1121_SETRXTXFALLBACKMODE_PARAMS 1 /* Takes LR1121_FALLBACK_x*/

/* Registers and buffer access **********************************************/

/* WriteRegister Function */

#define LR1121_WRITEREGISTER 0x0105
#define LR1121_WRITEREGISTER_PARAMS 2
#define LR1121_WRITEREGISTER_ADDRESS_PARAM 0
#define LR1121_WRITEREGISTER_ADDRESS_PARAMS 2
#define LR1121_WRITEREGISTER_DATA_PARAM 2    /* Data extends, address is auto incremented */
#define LR1121_WRITEREGISTER_STATUS_RETURN 0 /* Gets returned every byte sent */

/* ReadRegister Function */

#define LR1121_READREGISTER 0x0106
#define LR1121_READREGISTER_ADDRESS_PARAM 0
#define LR1121_READREGISTER_ADDRESS_PARAMS 2
#define LR1121_READREGISTER_STATUS_RETURN 0
#define LR1121_READREGISTER_STATUS_RETURNS 3
#define LR1121_READREGISTER_DATA_RETURN 3 /* Data extends, address is auto incremented */

/* WriteBuffer Function */

#define LR1121_WRITEBUFFER 0x0109
#define LR1121_WRITEBUFFER_PARAMS_MIN 2
#define LR1121_WRITEBUFFER_OFFSET_PARAM 0
#define LR1121_WRITEBUFFER_DATA_PARAM 1    /* Data extends, offset(address) is auto incremented */
#define LR1121_WRITEBUFFER_STATUS_RETURN 0 /* Gets returned every byte sent */

/* ReadBuffer Function */

#define LR1121_READBUFFER 0x010A
#define LR1121_READBUFFER_OFFSET_PARAM 0
#define LR1121_READBUFFER_STATUS_RETURN 0
#define LR1121_READBUFFER_STATUS_RETURNS 2
#define LR1121_READBUFFER_DATA_RETURN 2 /* Data extends, offset(address) is auto incremented */

/* DIO and IRQ Control Functions ********************************************/

/* SetDioIrqParams */

#define LR1121_SETDIOIRQPARAMS 0x0113
#define LR1121_SETDIOIRQPARAMS_PARAMS 8 /* Takes LR1121_IRQ_x bit masks */
#define LR1121_SETDIOIRQPARAMS_IRQMASK_PARAM 0
#define LR1121_SETDIOIRQPARAMS_IRQMASK_PARAMS 2
#define LR1121_SETDIOIRQPARAMS_DIO1MASK_PARAM 2
#define LR1121_SETDIOIRQPARAMS_DIO1MASK_PARAMS 2
#define LR1121_SETDIOIRQPARAMS_DIO2MASK_PARAM 4
#define LR1121_SETDIOIRQPARAMS_DIO2MASK_PARAMS 2
#define LR1121_SETDIOIRQPARAMS_DIO3MASK_PARAM 6
#define LR1121_SETDIOIRQPARAMS_DIO3MASK_PARAMS 2

/* SetDIO2AsRfSwitchCtrl */

#define LR1121_SETDIO2RFSWCTRL 0x9D
#define LR1121_SETDIO2RFSWCTRL_PARAMS 1
#define LR1121_SETDIO2RFSWCTRL_ENABLE_PARAM 0 /* true/false */

/* SetDIO3AsTCXOCtrl */

#define LR1121_SETDIO3TCXOCTRL 0x97
#define LR1121_SETDIO3TCXOCTRL_PARAMS 4
#define LR1121_SETDIO3TCXOCTRL_TCXO_V_PARAM 0 /* Takes LR1121_TCXO_xV */
#define LR1121_SETDIO3TCXOCTRL_DELAY_PARAM 1  /* time = delay(23:0) * 15.625 uS */
#define LR1121_SETDIO3TCXOCTRL_DELAY_PARAMS 3

/* GetIrqStatus */

#define LR1121_GETIRQSTATUS 0x12
#define LR1121_GETIRQSTATUS_RETURNS 3
#define LR1121_GETIRQSTATUS_STATUS_RETURN 0
#define LR1121_GETIRQSTATUS_IRQSTATUS_RETURN 1
#define LR1121_GETIRQSTATUS_IRQSTATUS_RETURNS 2

/* ClearIrqStatus */

#define LR1121_CLEARIRQSTATUS 0x02
#define LR1121_CLEARIRQSTATUS_PARAMS 2
#define LR1121_CLEARIRQSTATUS_CLEAR_PARAM 0
#define LR1121_CLEARIRQSTATUS_CLEAR_PARAMS 2

/* RF Modulation and Packet-Related Functions *******************************/

/* SetRfFrequency */

#define LR1121_SETRFFREQUENCY 0x020B
#define LR1121_SETRFFREQUENCY_PARAMS 4
#define LR1121_SETRFFREQUENCY_RFFREQ_PARAM 0 /* Takes (freq * 2 ^ 25) / xtal */
#define LR1121_SETRFFREQUENCY_RFFREQ_PARAMS 4

/* SetPacketType */

#define LR1121_SETPACKETTYPE 0x020E
#define LR1121_SETPACKETTYPE_PARAMS 1
#define LR1121_SETPACKETTYPE_PACKETTYPE_PARAM 0 /* Takes LR1121_PACKETTYPE_x */

/* GetPacketType */

#define LR1121_GETPACKETTYPE 0x0202
#define LR1121_GETPACKETTYPE_RETURNS 2
#define LR1121_GETPACKETTYPE_STATUS_RETURN 0
#define LR1121_GETPACKETTYPE_PACKETTYPE_RETURN 1 /* Gives LR1121_PACKETTYPE_x */

/* SetTxParms */

#define LR1121_SETTXPARMS 0x0211
#define LR1121_SETTXPARMS_PARAMS 2
#define LR1121_SETTXPARMS_POWER_PARAM 0
#define LR1121_SETTXPARMS_RAMPTIME_PARAM 1 /* Takes LR1121_SET_RAMP_xU*/

/* SetModulationParams */

#define LR1121_SETMODULATIONPARAMS 0x020F
#define LR1121_SETMODULATIONPARAMS_PARAMS 8 /* Takes the corresponding LR1121_MODPARAMx_y */
#define LR1121_SETMODULATIONPARAMS_PARAM1_PARAM 0
#define LR1121_SETMODULATIONPARAMS_PARAM2_PARAM 1
#define LR1121_SETMODULATIONPARAMS_PARAM3_PARAM 2
#define LR1121_SETMODULATIONPARAMS_PARAM4_PARAM 3
#define LR1121_SETMODULATIONPARAMS_PARAM5_PARAM 4
#define LR1121_SETMODULATIONPARAMS_PARAM6_PARAM 5
#define LR1121_SETMODULATIONPARAMS_PARAM7_PARAM 6
#define LR1121_SETMODULATIONPARAMS_PARAM8_PARAM 7

/* SetPacketParms */

#define LR1121_SETPACKETPARMS 0x0210
#define LR1121_SETPACKETPARMS_PARAMS 9 /* Takes the corresponding LR1121_PKTPARAMx_y */
#define LR1121_SETPACKETPARMS_PARAM1_PARAM 0
#define LR1121_SETPACKETPARMS_PARAM2_PARAM 1
#define LR1121_SETPACKETPARMS_PARAM3_PARAM 2
#define LR1121_SETPACKETPARMS_PARAM4_PARAM 3
#define LR1121_SETPACKETPARMS_PARAM5_PARAM 4
#define LR1121_SETPACKETPARMS_PARAM6_PARAM 5
#define LR1121_SETPACKETPARMS_PARAM7_PARAM 6
#define LR1121_SETPACKETPARMS_PARAM8_PARAM 7
#define LR1121_SETPACKETPARMS_PARAM9_PARAM 8

/* SetCadParams */

#define LR1121_SETCADPARAMS 0x020D
#define LR1121_SETCADPARAMS_PARAMS 7
#define LR1121_SETCADPARAMS_CADSYMNUM_PARAM 0 /* Takes LR1121_CAD_ON_x_SYMB */
#define LR1121_SETCADPARAMS_CADDETPEAK_PARAM 1
#define LR1121_SETCADPARAMS_CADDETMIN_PARAM 2
#define LR1121_SETCADPARAMS_CADEXITMODE_PARAM 3 /* Takes LR1121_CAD_x */
#define LR1121_SETCADPARAMS_CADTIMEOUT_PARAM 4  /* RxTimeout = cadTimeout * 15.625 */
#define LR1121_SETCADPARAMS_CADTIMEOUI_PARAMS 3

/* SetBufferBaseAddress TODO */

#define LR1121_SETBUFFERBASEADDRESS 0x8F
#define LR1121_SETBUFFERBASEADDRESS_PARAMS 2
#define LR1121_SETBUFFERBASEADDRESS_TX_PARAM 0
#define LR1121_SETBUFFERBASEADDRESS_RX_PARAM 1

/* SetLoRaSymbNumTimeout */

#define LR1121_SETLORASYMBNUMTIMEOUT 0x021B
#define LR1121_SETLORASYMBNUMTIMEOUT_PARAMS 1
#define LR1121_SETLORASYMBNUMTIMEOUT_NUM_PARAM 0

/* Communication status information *****************************************/

/* GetStatus */

#define LR1121_CMD_GETSTATUS 0x0100 /* Opcode */
#define LR1121_STATUS_CMD_SHIFT (1)
#define LR1121_STATUS_CMD_MASK (0b111 << LR1121_STATUS_CMD_SHIFT)
#define LR1121_STATUS_CHIPMODE_SHIFT (4)
#define LR1121_STATUS_CHIPMODE_MASK (0b111 << LR1121_STATUS_CHIPMODE_SHIFT)

/* GetRSSIInst */

#define LR1121_GETRSSIINST 0x0205
#define LR1121_GETRSSIINST_RETURNS 2
#define LR1121_GETRSSIINST_STAT_RETURN 0
#define LR1121_GETRSSIINST_RSSI_RETURN 1

/* GetRxBufferStatus */

#define LR1121_GETRXBUFFERSTATUS 0x0203
#define LR1121_GETRXBUFFERSTATUS_RETURNS 3
#define LR1121_GETRXBUFFERSTATUS_STATUS_RETURN 0
#define LR1121_GETRXBUFFERSTATUS_PAYLOAD_LEN_RETURN 1
#define LR1121_GETRXBUFFERSTATUS_RX_START_PTR_RETURN 2

/* Misc *********************************************************************/

/* GetDeviceErrors */

#define LR1121_GETDEVIrx_booCEERRORS 0x010D
#define LR1121_GETDEVICEERRORS_RETURNS 3
#define LR1121_GETDEVICEERRORS_STATUS_RETURN 0
#define LR1121_GETDEVICEERRORS_OPERROR_RETURN 1
#define LR1121_GETDEVICEERRORS_OPERROR_RETURNS 2

/* ClearDeviceErrors */

#define LR1121_CLEARDEVICEERRORS 0x010E
#define LR1121_CLEARDEVICEERRORS_NOPS 2

#endif /* __DRIVERS_WIRELESS_LPWAN_LR1121_LR1121_H */
