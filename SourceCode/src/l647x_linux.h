/**************************************************************************//**
  * @file    l647x_linux.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 7, 2015
  * @brief   L647X library for Linux
  * @note    (C) COPYRIGHT 2015 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef _L647X_LINUX_H
#define _L647X_LINUX_H


#include <sys/ioctl.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <unistd.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <sys/ioctl.h>



#include "l647x_target_config.h"

/******************************************************************************/
/* Choice of host platform                                                    */
/******************************************************************************/
#ifdef USE_RASPBERRY_PI
#include "l647x_RPi_host_config.h"
#include "gpio.h"
#endif


/* Constant ------------------------------------------------------------------*/
#define MAX_SPI_INTERFACE_NAME  20

/// The maximum number of devices in the daisy chain
#define MAX_NUMBER_OF_DEVICES                 (8)



/* Exported Constants --------------------------------------------------------*/

/// boolean for false condition
#ifndef FALSE
#define FALSE (0)
#endif
/// boolean for true condition
#ifndef TRUE
#define TRUE  (1)
#endif

/// Current FW version
#define L647X_FW_VERSION (1)

/// L647X max number of bytes of command & arguments to set a parameter
#define L647X_CMD_ARG_MAX_NB_BYTES              (4)

/// L647X command + argument bytes number for NOP command
#define L647X_CMD_ARG_NB_BYTES_NOP              (1)
/// L647X command + argument bytes number for RUN command
#define L647X_CMD_ARG_NB_BYTES_RUN              (4)
/// L647X command + argument bytes number for STEP_CLOCK command
#define L647X_CMD_ARG_NB_BYTES_STEP_CLOCK       (1)
/// L647X command + argument bytes number for MOVE command
#define L647X_CMD_ARG_NB_BYTES_MOVE             (4)
/// L647X command + argument bytes number for GO_TO command
#define L647X_CMD_ARG_NB_BYTES_GO_TO            (4)
/// L647X command + argument bytes number for GO_TO_DIR command
#define L647X_CMD_ARG_NB_BYTES_GO_TO_DIR        (4)
/// L647X command + argument bytes number for GO_UNTIL command
#define L647X_CMD_ARG_NB_BYTES_GO_UNTIL         (4)
/// L647X command + argument bytes number for RELEASE_SW command
#define L647X_CMD_ARG_NB_BYTES_RELEASE_SW       (1)
/// L647X command + argument bytes number for GO_HOME command
#define L647X_CMD_ARG_NB_BYTES_GO_HOME          (1)
/// L647X command + argument bytes number for GO_MARK command
#define L647X_CMD_ARG_NB_BYTES_GO_MARK          (1)
/// L647X command + argument bytes number for RESET_POS command
#define L647X_CMD_ARG_NB_BYTES_RESET_POS        (1)
/// L647X command + argument bytes number for RESET_DEVICE command
#define L647X_CMD_ARG_NB_BYTES_RESET_DEVICE     (1)
/// L647X command + argument bytes number for NOP command
#define L647X_CMD_ARG_NB_BYTES_SOFT_STOP        (1)
/// L647X command + argument bytes number for HARD_STOP command
#define L647X_CMD_ARG_NB_BYTES_HARD_STOP        (1)
/// L647X command + argument bytes number for SOFT_HIZ command
#define L647X_CMD_ARG_NB_BYTES_SOFT_HIZ         (1)
/// L647X command + argument bytes number for HARD_HIZ command
#define L647X_CMD_ARG_NB_BYTES_HARD_HIZ         (1)
/// L647X command + argument bytes number for GET_STATUS command
#define L647X_CMD_ARG_NB_BYTES_GET_STATUS       (1)

/// L647X response bytes number
#define L647X_RSP_NB_BYTES_GET_STATUS           (2)

/// Daisy chain command mask
#define DAISY_CHAIN_COMMAND_MASK (0xFA)

/* Exported Types  -------------------------------------------------------*/

/// Boolean type
typedef uint8_t  bool;


/******************************************************************************/
/******************************************************************************/
/* Private part                                                               */
/******************************************************************************/
/******************************************************************************/

/*-------------------------------------------------------------*/
/*----------- L6470 -------------------------------------------*/
/*-------------------------------------------------------------*/
#ifdef L6470

/// masks for ABS_POS register of L647X
typedef enum {
  L647X_ABS_POS_VALUE_MASK        = ((uint32_t) 0x003FFFFF),
  L647X_ABS_POS_SIGN_BIT_MASK     = ((uint32_t) 0x00200000)
} L647X_AbsPosMasks_t;

/// masks for EL_POS register of L647X
typedef enum {
  L647X_ELPOS_STEP_MASK       = ((uint8_t)0xC0),
  L647X_ELPOS_MICROSTEP_MASK  = ((uint8_t)0x3F)
} L647X_ElPosMasks_t;

/// masks for MIN_SPEED register of L647X
typedef enum {
  L647X_LSPD_OPT        = ((uint16_t) ((0x1) << 12)),
  L647X_MIN_SPEED_MASK  = ((uint16_t)0x0FFF)
} L647X_MinSpeedMasks_t;

/// Low speed optimization (MIN_SPEED register of L647X)
typedef enum {
  L647X_LSPD_OPT_OFF    = ((uint16_t)0x0000),
  L647X_LSPD_OPT_ON     = ((uint16_t)L647X_LSPD_OPT)
} L647X_LspdOpt_t;

/// Overcurrent threshold options (OCD register of L647X)
typedef enum {
	L647X_OCD_TH_375mA		=((uint8_t)0x00),
	L647X_OCD_TH_750mA		=((uint8_t)0x01),
	L647X_OCD_TH_1125mA		=((uint8_t)0x02),
	L647X_OCD_TH_1500mA		=((uint8_t)0x03),
	L647X_OCD_TH_1875mA		=((uint8_t)0x04),
	L647X_OCD_TH_2250mA		=((uint8_t)0x05),
	L647X_OCD_TH_2625mA		=((uint8_t)0x06),
	L647X_OCD_TH_3000mA		=((uint8_t)0x07),
	L647X_OCD_TH_3375mA		=((uint8_t)0x08),
	L647X_OCD_TH_3750mA		=((uint8_t)0x09),
	L647X_OCD_TH_4125mA		=((uint8_t)0x0A),
	L647X_OCD_TH_4500mA		=((uint8_t)0x0B),
	L647X_OCD_TH_4875mA		=((uint8_t)0x0C),
	L647X_OCD_TH_5250mA		=((uint8_t)0x0D),
	L647X_OCD_TH_5625mA		=((uint8_t)0x0E),
	L647X_OCD_TH_6000mA		=((uint8_t)0x0F)
} L647X_OCD_TH_TypeDef;

/// masks for STEP_MODE register of L647X
typedef enum {
  L647X_STEP_MODE_STEP_SEL    = ((uint8_t)0x07),
  L647X_STEP_MODE_SYNC_SEL    = ((uint8_t)0x70),
  L647X_STEP_MODE_SYNC_EN     = ((uint8_t)0x80)
} L647X_StepModeMasks_t;

 /// Stepping options (field STEP_SEL of STEP_MODE register of L647X)
typedef enum {
  L647X_STEP_SEL_1      = ((uint8_t)0x00),
  L647X_STEP_SEL_1_2    = ((uint8_t)0x01),
  L647X_STEP_SEL_1_4    = ((uint8_t)0x02),
  L647X_STEP_SEL_1_8    = ((uint8_t)0x03),
  L647X_STEP_SEL_1_16   = ((uint8_t)0x04),
  L647X_STEP_SEL_1_32   = ((uint8_t)0x05),
  L647X_STEP_SEL_1_64   = ((uint8_t)0x06),
  L647X_STEP_SEL_1_128  = ((uint8_t)0x07)
} L647X_StepSel_t;

/// L647X Sync Output frequency enabling bitw
#define L647X_SYNC_EN   ((0x1) << 7)

/// SYNC_SEL options (STEP_MODE register of L647X)
typedef enum {
  L647X_SYNC_SEL_DISABLED   = ((uint8_t)0x00),
  L647X_SYNC_SEL_1_2        = ((uint8_t)(L647X_SYNC_EN|0x00)),
  L647X_SYNC_SEL_1          = ((uint8_t)(L647X_SYNC_EN|0x10)),
  L647X_SYNC_SEL_2          = ((uint8_t)(L647X_SYNC_EN|0x20)),
  L647X_SYNC_SEL_4          = ((uint8_t)(L647X_SYNC_EN|0x30)),
  L647X_SYNC_SEL_8          = ((uint8_t)(L647X_SYNC_EN|0x40)),
  L647X_SYNC_SEL_16         = ((uint8_t)(L647X_SYNC_EN|0x50)),
  L647X_SYNC_SEL_32         = ((uint8_t)(L647X_SYNC_EN|0x60)),
  L647X_SYNC_SEL_64         = ((uint8_t)(L647X_SYNC_EN|0x70))
} L647X_SyncSel_t;

/// Alarms conditions (ALARM_EN register of L647X)
typedef enum {
	L647X_ALARM_EN_OVERCURRENT			  =((uint8_t)0x01),
	L647X_ALARM_EN_THERMAL_SHUTDOWN		=((uint8_t)0x02),
	L647X_ALARM_EN_THERMAL_WARNING		=((uint8_t)0x04),
	L647X_ALARM_EN_UNDER_VOLTAGE		  =((uint8_t)0x08),
	L647X_ALARM_EN_STALL_DET_A			  =((uint8_t)0x10),
	L647X_ALARM_EN_STALL_DET_B			  =((uint8_t)0x20),
	L647X_ALARM_EN_SW_TURN_ON			    =((uint8_t)0x40),
	L647X_ALARM_EN_WRONG_NPERF_CMD		=((uint8_t)0x80)
} L647X_AlarmEn_t;

/// Masks for CONFIG register of L647X
typedef enum {
	L647X_CONFIG_OSC_SEL					=((uint16_t)0x0007),
	L647X_CONFIG_EXT_CLK					=((uint16_t)0x0008),
	L647X_CONFIG_SW_MODE					=((uint16_t)0x0010),
	L647X_CONFIG_EN_VSCOMP				=((uint16_t)0x0020),
	L647X_CONFIG_OC_SD						=((uint16_t)0x0080),
	L647X_CONFIG_POW_SR						=((uint16_t)0x0300),
	L647X_CONFIG_F_PWM_DEC				=((uint16_t)0x1C00),
	L647X_CONFIG_F_PWM_INT				=((uint16_t)0xE000)
} L647X_ConfigMasks_t;

/// Oscillator management (EXT_CLK and OSC_SEL fields of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_INT_16MHZ					      =((uint16_t)0x0000),
	L647X_CONFIG_INT_16MHZ_OSCOUT_2MHZ		=((uint16_t)0x0008),
	L647X_CONFIG_INT_16MHZ_OSCOUT_4MHZ		=((uint16_t)0x0009),
	L647X_CONFIG_INT_16MHZ_OSCOUT_8MHZ		=((uint16_t)0x000A),
	L647X_CONFIG_INT_16MHZ_OSCOUT_16MHZ		=((uint16_t)0x000B),
	L647X_CONFIG_EXT_8MHZ_XTAL_DRIVE		  =((uint16_t)0x0004),
	L647X_CONFIG_EXT_16MHZ_XTAL_DRIVE		  =((uint16_t)0x0005),
	L647X_CONFIG_EXT_24MHZ_XTAL_DRIVE		  =((uint16_t)0x0006),
	L647X_CONFIG_EXT_32MHZ_XTAL_DRIVE		  =((uint16_t)0x0007),
	L647X_CONFIG_EXT_8MHZ_OSCOUT_INVERT		=((uint16_t)0x000C),
	L647X_CONFIG_EXT_16MHZ_OSCOUT_INVERT	=((uint16_t)0x000D),
	L647X_CONFIG_EXT_24MHZ_OSCOUT_INVERT	=((uint16_t)0x000E),
	L647X_CONFIG_EXT_32MHZ_OSCOUT_INVERT	=((uint16_t)0x000F)
} L647X_ConfigOscMgmt_t;

/// Oscillator management (EXT_CLK and OSC_SEL fields of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_SW_HARD_STOP		=((uint16_t)0x0000),
	L647X_CONFIG_SW_USER			=((uint16_t)0x0010)
} L647X_ConfigSwMode_t;

/// Voltage supply compensation enabling (EN_VSCOMP field of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_VS_COMP_DISABLE	=((uint16_t)0x0000),
	L647X_CONFIG_VS_COMP_ENABLE		=((uint16_t)0x0020)
} L647X_ConfigEnVscomp_t;

/// Overcurrent shutdown (OC_SD field of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_OC_SD_DISABLE		=((uint16_t)0x0000),
	L647X_CONFIG_OC_SD_ENABLE		  =((uint16_t)0x0080)
} L647X_ConfigOcSd_t;

typedef enum {
	L647X_CCONFIG_POW_SR_320V_us		=((uint16_t)0x0000),
  L647X_CCONFIG_POW_SR_075V_us		=((uint16_t)0x0100),
	L647X_CCONFIG_POW_SR_110V_us		=((uint16_t)0x0200),
	L647X_CONFIG_POW_SR_260V_us		  =((uint16_t)0x0300)
} L647X_ConfigPowSr_t;

/// PWM frequency division factor (F_PWM_INT field of CONFIG register of L647X)
typedef enum {
  L647X_CONFIG_PWM_DIV_1		=(((uint16_t)0x00)<<13),
	L647X_CONFIG_PWM_DIV_2		=(((uint16_t)0x01)<<13),
	L647X_CONFIG_PWM_DIV_3		=(((uint16_t)0x02)<<13),
	L647X_CONFIG_PWM_DIV_4		=(((uint16_t)0x03)<<13),
	L647X_CONFIG_PWM_DIV_5		=(((uint16_t)0x04)<<13),
	L647X_CONFIG_PWM_DIV_6		=(((uint16_t)0x05)<<13),
	L647X_CONFIG_PWM_DIV_7		=(((uint16_t)0x06)<<13)
} L647X_ConfigFPwmInt_t;

/// PWM frequency multiplication factor (F_PWM_DEC field of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_PWM_MUL_0_625	=(((uint16_t)0x00)<<10),
	L647X_CONFIG_PWM_MUL_0_75		=(((uint16_t)0x01)<<10),
	L647X_CONFIG_PWM_MUL_0_875	=(((uint16_t)0x02)<<10),
	L647X_CONFIG_PWM_MUL_1			=(((uint16_t)0x03)<<10),
	L647X_CONFIG_PWM_MUL_1_25		=(((uint16_t)0x04)<<10),
	L647X_CONFIG_PWM_MUL_1_5		=(((uint16_t)0x05)<<10),
	L647X_CONFIG_PWM_MUL_1_75		=(((uint16_t)0x06)<<10),
	L647X_CONFIG_PWM_MUL_2			=(((uint16_t)0x07)<<10)
} L647X_ConfigFPwmDec_t;

/// Bit mask for STATUS Register of L647X
typedef enum {
  L647X_STATUS_HIZ			=(((uint16_t)0x0001)),
	L647X_STATUS_BUSY			=(((uint16_t)0x0002)),
	L647X_STATUS_SW_F			=(((uint16_t)0x0004)),
	L647X_STATUS_SW_EVN			=(((uint16_t)0x0008)),
	L647X_STATUS_DIR			=(((uint16_t)0x0010)),
	L647X_STATUS_MOT_STATUS		=(((uint16_t)0x0060)),
	L647X_STATUS_NOTPERF_CMD	=(((uint16_t)0x0080)),
	L647X_STATUS_WRONG_CMD		=(((uint16_t)0x0100)),
	L647X_STATUS_UVLO			=(((uint16_t)0x0200)),
	L647X_STATUS_TH_WRN			=(((uint16_t)0x0400)),
	L647X_STATUS_TH_SD			=(((uint16_t)0x0800)),
	L647X_STATUS_OCD			=(((uint16_t)0x1000)),
	L647X_STATUS_STEP_LOSS_A	=(((uint16_t)0x2000)),
	L647X_STATUS_STEP_LOSS_B	=(((uint16_t)0x4000)),
	L647X_STATUS_SCK_MOD		=(((uint16_t)0x8000))
} L647X_StatusMasks_t;

/// Motor state (MOT_STATUS filed of STATUS register of L647X)
typedef enum {
	L647X_STATUS_MOT_STATUS_STOPPED			=(((uint16_t)0x0000)<<5),
	L647X_STATUS_MOT_STATUS_ACCELERATION	=(((uint16_t)0x0001)<<5),
	L647X_STATUS_MOT_STATUS_DECELERATION	=(((uint16_t)0x0002)<<5),
	L647X_STATUS_MOT_STATUS_CONST_SPD		=(((uint16_t)0x0003)<<5)
} L647X_Status_t;

/// L647X internal register addresses
typedef enum {
	L647X_ABS_POS			    =((uint8_t)0x01),
	L647X_EL_POS			    =((uint8_t)0x02),
	L647X_MARK				    =((uint8_t)0x03),
	L647X_SPEED				    =((uint8_t)0x04),
	L647X_ACC				      =((uint8_t)0x05),
	L647X_DEC				      =((uint8_t)0x06),
	L647X_MAX_SPEED		    =((uint8_t)0x07),
	L647X_MIN_SPEED		    =((uint8_t)0x08),
	L647X_FS_SPD			    =((uint8_t)0x15),
	L647X_KVAL_HOLD		    =((uint8_t)0x09),
	L647X_KVAL_RUN		    =((uint8_t)0x0A),
	L647X_KVAL_ACC		    =((uint8_t)0x0B),
	L647X_KVAL_DEC		    =((uint8_t)0x0C),
	L647X_INT_SPD			    =((uint8_t)0x0D),
	L647X_ST_SLP			    =((uint8_t)0x0E),
	L647X_FN_SLP_ACC	    =((uint8_t)0x0F),
	L647X_FN_SLP_DEC	    =((uint8_t)0x10),
	L647X_K_THERM			    =((uint8_t)0x11),
	L647X_ADC_OUT			    =((uint8_t)0x12),
	L647X_OCD_TH			    =((uint8_t)0x13),
	L647X_STALL_TH		    =((uint8_t)0x14),
	L647X_STEP_MODE		    =((uint8_t)0x16),
	L647X_ALARM_EN		    =((uint8_t)0x17),
	L647X_CONFIG		    	=((uint8_t)0x18),
	L647X_STATUS			    =((uint8_t)0x19),
	L647X_RESERVED_REG2		=((uint8_t)0x1A),
	L647X_RESERVED_REG1		=((uint8_t)0x1B)
} L647X_Registers_t;

/// L647X application commands
typedef enum {
	L647X_NOP			=((uint8_t)0x00),
	L647X_SET_PARAM		=((uint8_t)0x00),
	L647X_GET_PARAM		=((uint8_t)0x20),
	L647X_RUN			=((uint8_t)0x50),
	L647X_STEP_CLOCK	=((uint8_t)0x58),
	L647X_MOVE			=((uint8_t)0x40),
	L647X_GO_TO			=((uint8_t)0x60),
	L647X_GO_TO_DIR		=((uint8_t)0x68),
	L647X_GO_UNTIL		=((uint8_t)0x82),
  L647X_GO_UNTIL_ACT_CPY  =((uint8_t)0x8A),
	L647X_RELEASE_SW	=((uint8_t)0x92),
	L647X_GO_HOME		=((uint8_t)0x70),
	L647X_GO_MARK		=((uint8_t)0x78),
	L647X_RESET_POS		=((uint8_t)0xD8),
	L647X_RESET_DEVICE	=((uint8_t)0xC0),
	L647X_SOFT_STOP		=((uint8_t)0xB0),
	L647X_HARD_STOP		=((uint8_t)0xB8),
	L647X_SOFT_HIZ		=((uint8_t)0xA0),
	L647X_HARD_HIZ		=((uint8_t)0xA8),
	L647X_GET_STATUS	=((uint8_t)0xD0),
	L647X_RESERVED_CMD2	=((uint8_t)0xEB),
	L647X_RESERVED_CMD1	=((uint8_t)0xF8)
} L647X_Commands_t;

/// Direction options
typedef enum {
  FORWARD   = ((uint8_t)0x01),
  BACKWARD   = ((uint8_t)0x00)
} L647X_Direction_t;

/// Action options
typedef enum {
  ACTION_RESET = ((uint8_t)0x00),
  ACTION_COPY  = ((uint8_t)0x08)
} L647X_Action_t;

#endif /* #ifdef L6470 */

////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------*/
/*----------- L6470 -------------------------------------------*/
/*-------------------------------------------------------------*/

#ifdef L6472

/// masks for ABS_POS register of L647X
typedef enum {
  L647X_ABS_POS_VALUE_MASK        = ((uint32_t) 0x003FFFFF),
  L647X_ABS_POS_SIGN_BIT_MASK     = ((uint32_t) 0x00200000)
} L647X_AbsPosMasks_t;

/// masks for EL_POS register of L647X
typedef enum {
  L647X_ELPOS_STEP_MASK       = ((uint8_t)0xC0),
  L647X_ELPOS_MICROSTEP_MASK  = ((uint8_t)0x3F)
} L647X_ElPosMasks_t;

/// masks for MIN_SPEED register of L647X
typedef enum {
  L647X_LSPD_OPT        = ((uint16_t) ((0x1) << 12)),
  L647X_MIN_SPEED_MASK  = ((uint16_t)0x0FFF)
} L647X_MinSpeedMasks_t;

/// Low speed optimization (MIN_SPEED register of L647X)
typedef enum {
  L647X_LSPD_OPT_OFF    = ((uint16_t)0x0000),
  L647X_LSPD_OPT_ON     = ((uint16_t)L647X_LSPD_OPT)
} L647X_LspdOpt_t;

/// Maximum fall step times (T_FAST register of L647X)
typedef enum {
  L647X_FAST_STEP_2us     = ((uint8_t)0x00),
  L647X_FAST_STEP_4us     = ((uint8_t)0x01),
  L647X_FAST_STEP_6us     = ((uint8_t)0x02),
  L647X_FAST_STEP_8us     = ((uint8_t)0x03),
  L647X_FAST_STEP_10us    = ((uint8_t)0x04),
  L647X_FAST_STEP_12us    = ((uint8_t)0x05),
  L647X_FAST_STEP_14us    = ((uint8_t)0x06),
  L647X_FAST_STEP_16us    = ((uint8_t)0x07),
  L647X_FAST_STEP_18us    = ((uint8_t)0x08),
  L647X_FAST_STEP_20us    = ((uint8_t)0x09),
  L647X_FAST_STEP_22us    = ((uint8_t)0x0A),
  L647X_FAST_STEP_24s     = ((uint8_t)0x0B),
  L647X_FAST_STEP_26us    = ((uint8_t)0x0C),
  L647X_FAST_STEP_28us    = ((uint8_t)0x0D),
  L647X_FAST_STEP_30us    = ((uint8_t)0x0E),
  L647X_FAST_STEP_32us    = ((uint8_t)0x0F)
} L647X_FastStep_t;


/// Maximum fast decay times (T_FAST register of L647X)
typedef enum {
  L647X_TOFF_FAST_2us     = (((uint8_t)0x00)<<4),
  L647X_TOFF_FAST_4us     = (((uint8_t)0x01)<<4),
  L647X_TOFF_FAST_6us     = (((uint8_t)0x02)<<4),
  L647X_TOFF_FAST_8us     = (((uint8_t)0x03)<<4),
  L647X_TOFF_FAST_10us    = (((uint8_t)0x04)<<4),
  L647X_TOFF_FAST_12us    = (((uint8_t)0x05)<<4),
  L647X_TOFF_FAST_14us    = (((uint8_t)0x06)<<4),
  L647X_TOFF_FAST_16us    = (((uint8_t)0x07)<<4),
  L647X_TOFF_FAST_18us    = (((uint8_t)0x08)<<4),
  L647X_TOFF_FAST_20us    = (((uint8_t)0x09)<<4),
  L647X_TOFF_FAST_22us    = (((uint8_t)0x0A)<<4),
  L647X_TOFF_FAST_24us    = (((uint8_t)0x0B)<<4),
  L647X_TOFF_FAST_26us    = (((uint8_t)0x0C)<<4),
  L647X_TOFF_FAST_28us    = (((uint8_t)0x0D)<<4),
  L647X_TOFF_FAST_30us    = (((uint8_t)0x0E)<<4),
  L647X_TOFF_FAST_32us    = (((uint8_t)0x0F)<<4)
} L647X_ToffFast_t;

/// Overcurrent threshold options (OCD register of L647X)
typedef enum {
	L647X_OCD_TH_375mA		=((uint8_t)0x00),
	L647X_OCD_TH_750mA		=((uint8_t)0x01),
	L647X_OCD_TH_1125mA		=((uint8_t)0x02),
	L647X_OCD_TH_1500mA		=((uint8_t)0x03),
	L647X_OCD_TH_1875mA		=((uint8_t)0x04),
	L647X_OCD_TH_2250mA		=((uint8_t)0x05),
	L647X_OCD_TH_2625mA		=((uint8_t)0x06),
	L647X_OCD_TH_3000mA		=((uint8_t)0x07),
	L647X_OCD_TH_3375mA		=((uint8_t)0x08),
	L647X_OCD_TH_3750mA		=((uint8_t)0x09),
	L647X_OCD_TH_4125mA		=((uint8_t)0x0A),
	L647X_OCD_TH_4500mA		=((uint8_t)0x0B),
	L647X_OCD_TH_4875mA		=((uint8_t)0x0C),
	L647X_OCD_TH_5250mA		=((uint8_t)0x0D),
	L647X_OCD_TH_5625mA		=((uint8_t)0x0E),
	L647X_OCD_TH_6000mA		=((uint8_t)0x0F)
} L647X_OCD_TH_TypeDef;

/// masks for STEP_MODE register of L647X
typedef enum {
  L647X_STEP_MODE_STEP_SEL    = ((uint8_t)0x07),
  L647X_STEP_MODE_SYNC_SEL    = ((uint8_t)0x70),
  L647X_STEP_MODE_SYNC_EN     = ((uint8_t)0x80)
} L647X_StepModeMasks_t;

 /// Stepping options (field STEP_SEL of STEP_MODE register of L647X)
typedef enum {
  L647X_STEP_SEL_1      = ((uint8_t)0x00),
  L647X_STEP_SEL_1_2    = ((uint8_t)0x01),
  L647X_STEP_SEL_1_4    = ((uint8_t)0x02),
  L647X_STEP_SEL_1_8    = ((uint8_t)0x03),
  L647X_STEP_SEL_1_16   = (AccDec_Steps_to_Par(uint8_t)0x04),
} L647X_StepSel_t;

/// L647X Sync Output frequency enabling bitw
#define L647X_SYNC_EN   ((0x1) << 7)

/// SYNC_SEL options (STEP_MODE register of L647X)
typedef enum {
  L647X_SYNC_SEL_DISABLED   = ((uint8_t)0x00),
  L647X_SYNC_SEL_1_2        = ((uint8_t)(L647X_SYNC_EN|0x00)),
  L647X_SYNC_SEL_1          = ((uint8_t)(L647X_SYNC_EN|0x10)),
  L647X_SYNC_SEL_2          = ((uint8_t)(L647X_SYNC_EN|0x20)),
  L647X_SYNC_SEL_4          = ((uint8_t)(L647X_SYNC_EN|0x30)),
  L647X_SYNC_SEL_8          = ((uint8_t)(L647X_SYNC_EN|0x40))
} L647X_SyncSel_t;

/// Alarms conditions (ALARM_EN register of L647X)
typedef enum {
	L647X_ALARM_EN_OVERCURRENT			  =((uint8_t)0x01),
	L647X_ALARM_EN_THERMAL_SHUTDOWN		=((uint8_t)0x02),
	L647X_ALARM_EN_THERMAL_WARNING		=((uint8_t)0x04),
	L647X_ALARM_EN_UNDER_VOLTAGE		  =((uint8_t)0x08),
	L647X_ALARM_EN_SW_TURN_ON			    =((uint8_t)0x40),
	L647X_ALARM_EN_WRONG_NPERF_CMD		=((uint8_t)0x80)
} L647X_AlarmEn_t;

/// Masks for CONFIG register of L647X
typedef enum {
	L647X_CONFIG_OSC_SEL					=((uint16_t)0x0007),
	L647X_CONFIG_EXT_CLK					=((uint16_t)0x0008),
	L647X_CONFIG_SW_MODE					=((uint16_t)0x0010),
	L647X_CONFIG_EN_VSCOMP				=((uint16_t)0x0020),
	L647X_CONFIG_OC_SD					  =((uint16_t)0x0080),
	L647X_CONFIG_POW_SR					  =((uint16_t)0x0300),
	L647X_CONFIG_TSW					    =((uint16_t)0x7C00),
	L647X_CONFIG_PRED_EN					=((uint16_t)0x8000)
} L647X_ConfigMasks_t;

/// Oscillator management (EXT_CLK and OSC_SEL fields of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_INT_16MHZ					      =((uint16_t)0x0000),
	L647X_CONFIG_INT_16MHZ_OSCOUT_2MHZ		=((uint16_t)0x0008),
	L647X_CONFIG_INT_16MHZ_OSCOUT_4MHZ		=((uint16_t)0x0009),
	L647X_CONFIG_INT_16MHZ_OSCOUT_8MHZ		=((uint16_t)0x000A),
	L647X_CONFIG_INT_16MHZ_OSCOUT_16MHZ		=((uint16_t)0x000B),
	L647X_CONFIG_EXT_8MHZ_XTAL_DRIVE		  =((uint16_t)0x0004),
	L647X_CONFIG_EXT_16MHZ_XTAL_DRIVE		  =((uint16_t)0x0005),
	L647X_CONFIG_EXT_24MHZ_XTAL_DRIVE		  =((uint16_t)0x0006),
	L647X_CONFIG_EXT_32MHZ_XTAL_DRIVE		  =((uint16_t)0x0007),
	L647X_CONFIG_EXT_8MHZ_OSCOUT_INVERT		=((uint16_t)0x000C),
	L647X_CONFIG_EXT_16MHZ_OSCOUT_INVERT	=((uint16_t)0x000D),
	L647X_CONFIG_EXT_24MHZ_OSCOUT_INVERT	=((uint16_t)0x000E),
	L647X_CONFIG_EXT_32MHZ_OSCOUT_INVERT	=((uint16_t)0x000F)
} L647X_ConfigOscMgmt_t;

/// Oscillator management (EXT_CLK and OSC_SEL fields of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_SW_HARD_STOP		=((uint16_t)0x0000),
	L647X_CONFIG_SW_USER			=((uint16_t)0x0010)
} L647X_ConfigSwMode_t;

/// External torque regulation enabling (EN_TQREG field of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_TQ_REG_TVAL_USED	    =((uint16_t)0x0000),
	L647X_CONFIG_TQ_REG_ADC_OUT		    =((uint16_t)0x0020)
} L647X_ConfigEnTqReg_t;

/// Overcurrent shutdown (OC_SD field of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_OC_SD_DISABLE		=((uint16_t)0x0000),
	L647X_CONFIG_OC_SD_ENABLE		  =((uint16_t)0x0080)
} L647X_ConfigOcSd_t;

typedef enum {
	L647X_CCONFIG_POW_SR_320V_us		=((uint16_t)0x0000),
  L647X_CCONFIG_POW_SR_075V_us		=((uint16_t)0x0100),
	L647X_CCONFIG_POW_SR_110V_us		=((uint16_t)0x0200),
	L647X_CONFIG_POW_SR_260V_us		  =((uint16_t)0x0300)
} L647X_ConfigPowSr_t;

/// Switching period  (TSW field of CONFIG register of L647X)
typedef enum {
	L647X_CONFIG_TSW_004us		=(((uint16_t)0x01)<<10),
	L647X_CONFIG_TSW_008us		=(((uint16_t)0x02)<<10),
	L647X_CONFIG_TSW_012us		=(((uint16_t)0x03)<<10),
	L647X_CONFIG_TSW_016us		=(((uint16_t)0x04)<<10),
	L647X_CONFIG_TSW_020us		=(((uint16_t)0x05)<<10),
	L647X_CONFIG_TSW_024us		=(((uint16_t)0x06)<<10),
	L647X_CONFIG_TSW_028us		=(((uint16_t)0x07)<<10),
	L647X_CONFIG_TSW_032us		=(((uint16_t)0x08)<<10),
	L647X_CONFIG_TSW_036us		=(((uint16_t)0x09)<<10),
	L647X_CONFIG_TSW_040us		=(((uint16_t)0x0A)<<10),
	L647X_CONFIG_TSW_044us		=(((uint16_t)0x0B)<<10),
	L647X_CONFIG_TSW_048us		=(((uint16_t)0x0C)<<10),
	L647X_CONFIG_TSW_052us		=(((uint16_t)0x0D)<<10),
	L647X_CONFIG_TSW_056us		=(((uint16_t)0x0E)<<10),
	L647X_CONFIG_TSW_060us		=(((uint16_t)0x0F)<<10),
	L647X_CONFIG_TSW_064us		=(((uint16_t)0x10)<<10),
	L647X_CONFIG_TSW_068us		=(((uint16_t)0x11)<<10),
	L647X_CONFIG_TSW_072us		=(((uint16_t)0x12)<<10),
	L647X_CONFIG_TSW_076us		=(((uint16_t)0x13)<<10),
	L647X_CONFIG_TSW_080us		=(((uint16_t)0x14)<<10),
	L647X_CONFIG_TSW_084us		=(((uint16_t)0x15)<<10),
	L647X_CONFIG_TSW_088us		=(((uint16_t)0x16)<<10),
	L647X_CONFIG_TSW_092us		=(((uint16_t)0x17)<<10),
	L647X_CONFIG_TSW_096us		=(((uint16_t)0x18)<<10),
	L647X_CONFIG_TSW_100us		=(((uint16_t)0x19)<<10),
	L647X_CONFIG_TSW_104us		=(((uint16_t)0x1A)<<10),
	L647X_CONFIG_TSW_108us		=(((uint16_t)0x1B)<<10),
	L647X_CONFIG_TSW_112us		=(((uint16_t)0x1C)<<10),
	L647X_CONFIG_TSW_116us		=(((uint16_t)0x1D)<<10),
	L647X_CONFIG_TSW_120us		=(((uint16_t)0x1E)<<10),
  L647X_CONFIG_TSW_124us		=(((uint16_t)0x1F)<<10)
} L647X_ConfigTsw_t;


/// Voltage supply compensation enabling (EN_PRED field of CONFIG register of L647X)
typedef enum {
  L647X_CONFIG_PRED_DISABLE =((uint16_t)0x0000),
  L647X_CONFIG_PRED_ENABLE  =((uint16_t)0x8000)
} L647X_ConfigPredEn_t;

/// Bit mask for STATUS Register of L647X
typedef enum {
  L647X_STATUS_HIZ			=(((uint16_t)0x0001)),
	L647X_STATUS_BUSY			=(((uint16_t)0x0002)),
	L647X_STATUS_SW_F			=(((uint16_t)0x0004)),
	L647X_STATUS_SW_EVN			=(((uint16_t)0x0008)),
	L647X_STATUS_DIR			=(((uint16_t)0x0010)),
	L647X_STATUS_MOT_STATUS		=(((uint16_t)0x0060)),
	L647X_STATUS_NOTPERF_CMD	=(((uint16_t)0x0080)),
	L647X_STATUS_WRONG_CMD		=(((uint16_t)0x0100)),
	L647X_STATUS_UVLO			=(((uint16_t)0x0200)),
	L647X_STATUS_TH_WRN			=(((uint16_t)0x0400)),
	L647X_STATUS_TH_SD			=(((uint16_t)0x0800)),
	L647X_STATUS_OCD			=(((uint16_t)0x1000)),
	L647X_STATUS_SCK_MOD		=(((uint16_t)0x8000))
} L647X_StatusMasks_t;

/// Motor state (MOT_STATUS filed of STATUS register of L647X)
typedef enum {
	L647X_STATUS_MOT_STATUS_STOPPED			=(((uint16_t)0x0000)<<5),
	L647X_STATUS_MOT_STATUS_ACCELERATION	=(((uint16_t)0x0001)<<5),
	L647X_STATUS_MOT_STATUS_DECELERATION	=(((uint16_t)0x0002)<<5),
	L647X_STATUS_MOT_STATUS_CONST_SPD		=(((uint16_t)0x0003)<<5)
} L647X_Status_t;

/// L647X internal register addresses
typedef enum {
	L647X_ABS_POS			    =((uint8_t)0x01),
	L647X_EL_POS			    =((uint8_t)0x02),
	L647X_MARK			      =((uint8_t)0x03),
	L647X_SPEED			      =((uint8_t)0x04),
	L647X_ACC			        =((uint8_t)0x05),
	L647X_DEC			        =((uint8_t)0x06),
	L647X_MAX_SPEED			  =((uint8_t)0x07),
	L647X_MIN_SPEED			  =((uint8_t)0x08),
	L647X_FS_SPD			    =((uint8_t)0x15),
	L647X_TVAL_HOLD			  =((uint8_t)0x09),
	L647X_TVAL_RUN			  =((uint8_t)0x0A),
	L647X_TVAL_ACC			  =((uint8_t)0x0B),
	L647X_TVAL_DEC			  =((uint8_t)0x0C),
	L647X_RESERVED_REG5		=((uint8_t)0x0D),
	L647X_T_FAST			    =((uint8_t)0x0E),
	L647X_TON_MIN		      =((uint8_t)0x0F),
	L647X_TOFF_MIN		    =((uint8_t)0x10),
	L647X_RESERVED_REG4		=((uint8_t)0x11),
	L647X_ADC_OUT			    =((uint8_t)0x12),
	L647X_OCD_TH			    =((uint8_t)0x13),
	L647X_RESERVED_REG3		=((uint8_t)0x14),
	L647X_STEP_MODE			  =((uint8_t)0x16),
	L647X_ALARM_EN			  =((uint8_t)0x17),
	L647X_CONFIG			    =((uint8_t)0x18),
	L647X_STATUS			    =((uint8_t)0x19),
	L647X_RESERVED_REG2		=((uint8_t)0x1A),
	L647X_RESERVED_REG1		=((uint8_t)0x1B)
} L647X_Registers_t;

/// L647X application commands
typedef enum {
	L647X_NOP			=((uint8_t)0x00),
	L647X_SET_PARAM		=((uint8_t)0x00),
	L647X_GET_PARAM		=((uint8_t)0x20),
	L647X_RUN			=((uint8_t)0x50),
	L647X_STEP_CLOCK	=((uint8_t)0x58),
	L647X_MOVE			=((uint8_t)0x40),
	L647X_GO_TO			=((uint8_t)0x60),
	L647X_GO_TO_DIR		=((uint8_t)0x68),
	L647X_GO_UNTIL		=((uint8_t)0x82),
  L647X_GO_UNTIL_ACT_CPY  =((uint8_t)0x8A),
	L647X_RELEASE_SW	=((uint8_t)0x92),
	L647X_GO_HOME		=((uint8_t)0x70),
	L647X_GO_MARK		=((uint8_t)0x78),
	L647X_RESET_POS		=((uint8_t)0xD8),
	L647X_RESET_DEVICE	=((uint8_t)0xC0),
	L647X_SOFT_STOP		=((uint8_t)0xB0),
	L647X_HARD_STOP		=((uint8_t)0xB8),
	L647X_SOFT_HIZ		=((uint8_t)0xA0),
	L647X_HARD_HIZ		=((uint8_t)0xA8),
	L647X_GET_STATUS	=((uint8_t)0xD0),
	L647X_RESERVED_CMD2	=((uint8_t)0xEB),
	L647X_RESERVED_CMD1	=((uint8_t)0xF8)
} L647X_Commands_t;

/// Direction options
typedef enum {
  FORWARD   = ((uint8_t)0x01),
  BACKWARD   = ((uint8_t)0x00)
} L647X_Direction_t;

/// Action options
typedef enum {
  ACTION_RESET = ((uint8_t)0x00),
  ACTION_COPY  = ((uint8_t)0x08)
} L647X_Action_t;

#endif /* #ifdef L6472 */



/* Structure definition ------------------------------------------------------*/

typedef struct {
	char name[MAX_SPI_INTERFACE_NAME];
    int  file_desc;
    unsigned int spi_mode;
    unsigned int spi_speed;
    unsigned int spi_bitsPerWord;
} spi_interface_struct_t;

typedef struct {
	char spi_interface_name[MAX_SPI_INTERFACE_NAME];
} interface_configuration_struct_t;

typedef struct {
	char name[15];
	uint8_t lacation;
} config_entry_t;

/* Exported macro ------------------------------------------------------------*/

/// Macro for Speed conversion (step/s to parameter) range 0 to 15625 steps/s
#define Speed_Steps_to_Par(steps) ((uint32_t)(((steps)*67.108864)+0.5))
/// Macro for Speed conversion (parameter to step/s), range 0 to 15625 steps/s
#define Speed_Par_to_Steps(Par) ((uint32_t)((Par)*0.01490116119))

/// Macro for acc/Dec rates conversion (step/s^2 to parameter), range 14.55 to 59590 steps/s2
#define AccDec_Steps_to_Par(steps) ((uint16_t)(((steps)*0.068719476736)+0.5))
/// Macro for acc/Dec rates conversion (parameter to step/s^2)
#define AccDec_Par_to_Steps(Par) ((uint16_t)((Par)*14.5519152283))


/// Macro for max Speed conversion (step/s to parameter), range 15.25 to 15610 steps/s
#define MaxSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*0.065536)+0.5))
/// Macro for max Speed conversion (parameter to step/s)
#define MaxSpd_Par_to_Steps(Par) ((uint16_t)((Par)*15,258789))

/// Macro for min Speed conversion (step/s to parameter), range 0 to 976.3 steps/s */
#define MinSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*4.194304)+0.5))
/// Macro for min Speed conversion (parameter to step/s)
#define MinSpd_Par_to_Steps(Par) ((uint16_t)((steps)*0.238418579))

/// Macro for full Step Speed conversion (step/s to parameter), range 7.63 to 15625 steps/s
#define FSSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*0.065536)+0.5))
/// Macro for full Step Speed conversion (parameter to step/s)
#define FSSpd_Par_to_Steps(Par) ((uint16_t)((Par)*15,258789))

//
// Specific for L6470
//
/// Macro for intersect Speed conversion, range 0 to 3906 steps/s
#define IntSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*16.777216)+0.5))
/// Macro for KVAL conversions, range 0.4% to 99.6%
#define Kval_Perc_to_Par(perc) ((uint8_t)(((perc)/0.390625)+0.5))
/// Macro for BEMF compensation slopes, range 0 to 0.4% s/step
#define BEMF_Slope_Perc_to_Par(perc) ((uint8_t)(((perc)/0.00156862745098)+0.5))
/// Macro for K_THERM compensation conversion, range 1 to 1.46875
#define KTherm_to_Par(KTherm) ((uint8_t)(((KTherm - 1)/0.03125)+0.5))
/// Macro for stall Threshold conversion, range 31.25mV to 1000mV
#define StallTh_to_Par(StallTh) ((uint8_t)(((StallTh - 31.25)/31.25)+0.5))

//
// Specific for L6472
//
/// Macro for torque regulation DAC current conversion, range 7.8mV to 1000mV
#define Tval_Current_to_Par(Tval) ((uint8_t)(((Tval - 7.8)/7.8)+0.5))
/// Macro for minimum time conversion, range 0.5us to 64us
#define Tmin_Time_to_Par(Tmin) ((uint8_t)(((Tmin - 0.5)*2)+0.5))




/* Exported functions --------------------------------------------------------*/


/// @defgroup group1 L647X library functions
///@{

int L647X_SPIOpenPort( char spi_interface_name[] );
void L647X_AttachBusyInterrupt(void (*callback)(void));   //Attach a user callback to the busy interrupt
void L647X_AttachFlagInterrupt(void (*callback)(void));  //Attach a user callback to the flag Interrupt
void L647X_Begin(uint8_t nbDevices);                     //Start the L647X library

void L647X_LoadMotorConfigValuesFromFile(FILE *ptr);
void L647X_SetMotorConfigToPredefinedValues( void);
void L647X_SetRegisterValues( uint8_t deviceId);




//uint8_t L647X_CheckBusyHw(void);
//uint8_t L647X_CheckStatusHw(void);
void L647X_Close(void);
uint32_t L647X_CmdGetParam(uint8_t deviceId, L647X_Registers_t param);
uint16_t L647X_CmdGetStatus(uint8_t deviceId);
void L647X_CmdGoHome(uint8_t deviceId);
void L647X_CmdGoMark(uint8_t deviceId);
void L647X_CmdGoTo(uint8_t deviceId, int32_t abs_pos);
void L647X_CmdGoToDir(uint8_t deviceId, L647X_Direction_t direction, int32_t abs_pos);
void L647X_CmdGoUntil(uint8_t deviceId, L647X_Action_t action, L647X_Direction_t direction, uint32_t speed);
void L647X_CmdHardHiZ(uint8_t deviceId);
void L647X_CmdHardStop(uint8_t deviceId);
void L647X_CmdMove(uint8_t deviceId, L647X_Direction_t direction, uint32_t n_step);
void L647X_CmdNop(uint8_t deviceId);
void L647X_CmdReleaseSw(uint8_t deviceId, L647X_Action_t action, L647X_Direction_t direction);
void L647X_CmdResetDevice(uint8_t deviceId);
void L647X_CmdResetPos(uint8_t deviceId);
void L647X_CmdRun(uint8_t deviceId, L647X_Direction_t direction, uint32_t speed);
void L647X_CmdSetParam(uint8_t deviceId, L647X_Registers_t param, uint32_t value);
void L647X_CmdSoftHiZ(uint8_t deviceId);
void L647X_CmdSoftStop(uint8_t deviceId);
void L647X_CmdStepClock(uint8_t deviceId, L647X_Direction_t direction);
void L647X_FetchAndClearAllStatus(void);                 //Fetch Status register and clear flags
uint16_t L647X_GetFetchedStatus(uint8_t deviceId);       //Get the fetched value of the STATUS registeruint8_t L647X_GetFwVersion(void);                        //Return the FW version
int32_t L647X_GetMark(uint8_t deviceId);                 //Return the mark position
uint8_t L647X_GetNbDevices(void);                        //Get the number of devices
int32_t L647X_GetPosition(uint8_t deviceId);             //Return the ABS_POSITION (32b signed)
bool L647X_IsDeviceBusy(uint8_t deviceId);
void L647X_QueueCommands(uint8_t deviceId, uint8_t param, uint32_t value);
void L647X_ReleaseReset(void);                        //Release the L647X reset pin
void L647X_Reset(void);                               //Set the L647X reset pin
void L647X_SelectStepMode(uint8_t deviceId, L647X_StepSel_t stepMod);
void L647X_SendQueuedCommands(void);
void L647X_SetMark(uint8_t deviceId);
void L647X_StartStepClock(uint8_t step_clock_pin, uint32_t newFreq);
void L647X_StopStepClock(void);
void L647X_WaitWhileActive(uint8_t deviceId);
void L647X_WaitForAllDevicesNotBusy(void);
///@}


#endif /* #ifndef _L647X_LINUX_H_INCLUDED */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
