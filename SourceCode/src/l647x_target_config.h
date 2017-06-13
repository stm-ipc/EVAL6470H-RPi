/**************************************************************************//**
  * @file    l647x_target_config.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    July 17, 2014
  * @brief   Predefined values for the L647X registers
  * and for the devices parameters
  * @note    (C) COPYRIGHT 2014 STMicroelectronics
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

#ifndef __L647X_TARGET_CONFIG_H
#define __L647X_TARGET_CONFIG_H

/* Exported constants --------------------------------------------------------*/    
    



/****************************************************************************/
/* Device 0                                                                 */
/****************************************************************************/  
    
/**************************** Speed Profile *********************************/

/// Register : ACC
/// Acceleration rate in step/s2, range 14.55 to 59590 steps/s2
#define L647X_CONF_PARAM_ACC (2008.164)

/// Register : DEC 
/// Deceleration rate in step/s2, range 14.55 to 59590 steps/s2
#define L647X_CONF_PARAM_DEC (2008.164)

/// Register : MAX_SPEED 
/// Maximum speed in step/s, range 15.25 to 15610 steps/s 
#define L647X_CONF_PARAM_MAX_SPEED (991.821)

/// Register : MIN_SPEED */
/// Minimum speed in step/s, range 0 to 976.3 steps/s
#define L647X_CONF_PARAM_MIN_SPEED (0)

/// Register : FS_SPD
/// Full step speed in step/s, range 7.63 to 15625 steps/s
#define L647X_CONF_PARAM_FS_SPD (595.093)

/************************ Phase Current Control *****************************/

#ifdef L6470    
/// Register : KVAL_HOLD
/// Hold duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_HOLD (16.02)

/// Register : KVAL_RUN
///  Run duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_RUN (16.02)

/// Register : KVAL_ACC
/// Acceleration duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_ACC (16.02)

/// Register : KVAL_DEC
/// Deceleration duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_DEC (16.02)

/// Register : CONFIG - field : EN_VSCOMP
/// Motor Supply Voltage Compensation enabling , enum L647X_CONFIG_EN_VSCOMP_TypeDef
#define L647X_CONF_PARAM_VS_COMP (L647X_CONFIG_VS_COMP_DISABLE)

/// Register : MIN_SPEED - field : LSPD_OPT
/// Low speed optimization bit, enum L647X_LSPD_OPT_TypeDef
#define L647X_CONF_PARAM_LSPD_BIT (L647X_LSPD_OPT_OFF)

/// Register : K_THERM
///  Thermal compensation param, range 1 to 1.46875
#define L647X_CONF_PARAM_K_THERM (1)

/// Register : INT_SPEED
///  Intersect speed settings for BEMF compensation in steps/s, range 0 to 3906 steps/s
#define L647X_CONF_PARAM_INT_SPD (61.512)

///  Register : ST_SLP
/// BEMF start slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define L647X_CONF_PARAM_ST_SLP (0.03815)

/// Register : FN_SLP_ACC
/// BEMF final acc slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step 
#define L647X_CONF_PARAM_FN_SLP_ACC (0.06256)

/// Register : FN_SLP_DEC
/// BEMF final dec slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define L647X_CONF_PARAM_FN_SLP_DEC (0.06256)

/// Register : CONFIG - field : F_PWM_INT
/// PWM Frequency Integer division, enum L647X_CONFIG_F_PWM_INT_TypeDef
#define L647X_CONF_PARAM_PWM_DIV (L647X_CONFIG_PWM_DIV_2)

/// Register : CONFIG - field : F_PWM_DEC
/// PWM Frequency Integer Multiplier, enum L647X_CONFIG_F_PWM_INT_TypeDef
#define L647X_CONF_PARAM_PWM_MUL (L647X_CONFIG_PWM_MUL_1)

#endif /* #ifdef L6470 */

#ifdef L6472 
  /// Register : TVAL_ACC 
	/// Acceleration torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_ACC  (328.12)
  
  /// Register : TVAL_DEC 
	/// Deceleration torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_DEC  (328.12)
  
  /// Register : TVAL_RUN 
	/// Running torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_RUN  (328.12)
  
  /// Register : TVAL_HOLD 
	/// Holding torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_HOLD (328.12)

  /// Register : CONFIG - field : EN_TQREG 
	/// External torque regulation enabling , enum L647X_ConfigEnTqReg_t
  #define L647X_CONF_PARAM_TQ_REG (L647X_CONFIG_TQ_REG_TVAL_USED)

  /// Register : CONFIG - field : PRED_EN 
	/// Predictive current enabling , enum L647X_ConfigPredEn_t
  #define L647X_CONF_PARAM_PRED_EN (L647X_CONFIG_PRED_DISABLE)

  ///  Register : TON_MIN 
	///  Minimum on-time in us, range 0.5us to 64us 
  #define L647X_CONF_PARAM_TON_MIN (3.0)

  /// Register : TOFF_MIN 
	///  Minimum off-time in us, range 0.5us to 64us 
  #define L647X_CONF_PARAM_TOFF_MIN (21.0)

  /// Register : T_FAST - field: TOFF_FAST 
	/// Maximum fast decay time , enum L647X_ToffFast_t
  #define L647X_CONF_PARAM_TOFF_FAST  (L647X_TOFF_FAST_8us)

  /// Register : T_FAST - field: FAST_STEP 
  ///  Maximum fall step time , enum L647X_FastStep_t
  #define L647X_CONF_PARAM_FAST_STEP  (L647X_FAST_STEP_12us)

  /// Register : CONFIG - field : TSW 
	/// Switching period, enum L647X_ConfigTsw_t
  #define L647X_CONF_PARAM_TSW (L647X_CONFIG_TSW_048us)
    
#endif /* #ifdef L6472 */
    
/******************************* Others *************************************/
/// Register : OCD_TH
/// Overcurrent threshold settings via enum L647X_OCD_TH_TypeDef
#define L647X_CONF_PARAM_OCD_TH (L647X_OCD_TH_3375mA)

/// Register : STALL_TH
/// Stall threshold settings in mA, range 31.25mA to 4000mA
#define L647X_CONF_PARAM_STALL_TH (2031.25)

/// Register : ALARM_EN
/// Alarm settings via bitmap enum L647X_ALARM_EN_TypeDef
#ifdef L6470    
#define L647X_CONF_PARAM_ALARM_EN (L647X_ALARM_EN_OVERCURRENT | L647X_ALARM_EN_THERMAL_SHUTDOWN | L647X_ALARM_EN_THERMAL_WARNING | L647X_ALARM_EN_UNDER_VOLTAGE | L647X_ALARM_EN_STALL_DET_A | L647X_ALARM_EN_STALL_DET_B | L647X_ALARM_EN_SW_TURN_ON | L647X_ALARM_EN_WRONG_NPERF_CMD)
#endif
#ifdef L6472    
#define L647X_CONF_PARAM_ALARM_EN (L647X_ALARM_EN_OVERCURRENT | L647X_ALARM_EN_THERMAL_SHUTDOWN | L647X_ALARM_EN_THERMAL_WARNING | L647X_ALARM_EN_UNDER_VOLTAGE | L647X_ALARM_EN_SW_TURN_ON | L647X_ALARM_EN_WRONG_NPERF_CMD)
#endif
    
/// Register : STEP_MODE - field : STEP_MODE
/// Step mode settings via enum L647X_STEP_SEL_TypeDef
#ifdef L6470   
#define L647X_CONF_PARAM_STEP_MODE (L647X_STEP_SEL_1_128)
#endif
#ifdef L6472   
#define L647X_CONF_PARAM_STEP_MODE (L647X_STEP_SEL_1_16)
#endif 
    
/// Register : STEP_MODE - Field : SYNC_MODE and SYNC_EN
/// Synch. Mode settings via enum L647X_SYNC_SEL_TypeDef
#define L647X_CONF_PARAM_SYNC_MODE (L647X_SYNC_SEL_DISABLED)

/// Register : CONFIG - field : POW_SR
/// Slew rate, enum L647X_CONFIG_POW_SR_TypeDef
#define L647X_CONF_PARAM_SR (L647X_CONFIG_SR_110V_us)

/// Register : CONFIG - field : OC_SD
/// Over current shutwdown enabling, enum L647X_CONFIG_OC_SD_TypeDef
#define L647X_CONF_PARAM_OC_SD (L647X_CONFIG_OC_SD_DISABLE)

/// Register : CONFIG - field : SW_MODE
/// External switch hard stop interrupt mode, enum L647X_CONFIG_SW_MODE_TypeDef
#define L647X_CONF_PARAM_SW_MODE 	(L647X_CONFIG_SW_HARD_STOP)

/// Register : CONFIG - field : OSC_CLK_SEL
/// Clock setting , enum L647X_CONFIG_OSC_MGMT_TypeDef
#define L647X_CONF_PARAM_CLOCK_SETTING (L647X_CONFIG_INT_16MHZ_OSCOUT_2MHZ)

/****************************************************************************/
/* Device 1                                                                 */
/****************************************************************************/
  
/**************************** Speed Profile *********************************/

/// Register : ACC
/// Acceleration rate in step/s2, range 14.55 to 59590 steps/s2
#define L647X_CONF_PARAM_ACC_DEVICE_1 (2008.164)

/// Register : DEC 
/// Deceleration rate in step/s2, range 14.55 to 59590 steps/s2
#define L647X_CONF_PARAM_DEC_DEVICE_1 (2008.164)

/// Register : MAX_SPEED 
/// Maximum speed in step/s, range 15.25 to 15610 steps/s 
#define L647X_CONF_PARAM_MAX_SPEED_DEVICE_1 (991.821)

/// Register : MIN_SPEED */
/// Minimum speed in step/s, range 0 to 976.3 steps/s
#define L647X_CONF_PARAM_MIN_SPEED_DEVICE_1 (0)

/// Register : FS_SPD
/// Full step speed in step/s, range 7.63 to 15625 steps/s
#define L647X_CONF_PARAM_FS_SPD_DEVICE_1 (595.093)

/************************ Phase Current Control *****************************/

#ifdef L6470    
/// Register : KVAL_HOLD
/// Hold duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_HOLD_DEVICE_1 (16.02)

/// Register : KVAL_RUN
///  Run duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_RUN_DEVICE_1 (16.02)

/// Register : KVAL_ACC
/// Acceleration duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_ACC_DEVICE_1 (16.02)

/// Register : KVAL_DEC
/// Deceleration duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_DEC_DEVICE_1 (16.02)

/// Register : CONFIG - field : EN_VSCOMP
/// Motor Supply Voltage Compensation enabling , enum L647X_CONFIG_EN_VSCOMP_TypeDef
#define L647X_CONF_PARAM_VS_COMP_DEVICE_1 (L647X_CONFIG_VS_COMP_DISABLE)

/// Register : MIN_SPEED - field : LSPD_OPT
/// Low speed optimization bit, enum L647X_LSPD_OPT_TypeDef
#define L647X_CONF_PARAM_LSPD_BIT_DEVICE_1 (L647X_LSPD_OPT_OFF)

/// Register : K_THERM
///  Thermal compensation param, range 1 to 1.46875
#define L647X_CONF_PARAM_K_THERM_DEVICE_1 (1)

/// Register : INT_SPEED
///  Intersect speed settings for BEMF compensation in steps/s, range 0 to 3906 steps/s
#define L647X_CONF_PARAM_INT_SPD_DEVICE_1 (61.512)

///  Register : ST_SLP
/// BEMF start slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define L647X_CONF_PARAM_ST_SLP_DEVICE_1 (0.03815)

/// Register : FN_SLP_ACC
/// BEMF final acc slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step 
#define L647X_CONF_PARAM_FN_SLP_ACC_DEVICE_1 (0.06256)

/// Register : FN_SLP_DEC
/// BEMF final dec slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define L647X_CONF_PARAM_FN_SLP_DEC_DEVICE_1 (0.06256)

/// Register : CONFIG - field : F_PWM_INT
/// PWM Frequency Integer division, enum L647X_CONFIG_F_PWM_INT_TypeDef
#define L647X_CONF_PARAM_PWM_DIV_DEVICE_1 (L647X_CONFIG_PWM_DIV_2)

/// Register : CONFIG - field : F_PWM_DEC
/// PWM Frequency Integer Multiplier, enum L647X_CONFIG_F_PWM_INT_TypeDef
#define L647X_CONF_PARAM_PWM_MUL_DEVICE_1 (L647X_CONFIG_PWM_MUL_1)

#endif /* #ifdef L6470 */

#ifdef L6472 
  /// Register : TVAL_ACC 
	/// Acceleration torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_ACC_DEVICE_1  (328.12)
  
  /// Register : TVAL_DEC 
	/// Deceleration torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_DEC_DEVICE_1  (328.12)
  
  /// Register : TVAL_RUN 
	/// Running torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_RUN_DEVICE_1  (328.12)
  
  /// Register : TVAL_HOLD 
	/// Holding torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_HOLD_DEVICE_1 (328.12)

  /// Register : CONFIG - field : EN_TQREG 
	/// External torque regulation enabling , enum L647X_ConfigEnTqReg_t
  #define L647X_CONF_PARAM_TQ_REG_DEVICE_1 (L647X_CONFIG_TQ_REG_TVAL_USED)

  /// Register : CONFIG - field : PRED_EN 
	/// Predictive current enabling , enum L647X_ConfigPredEn_t
  #define L647X_CONF_PARAM_PRED_EN_DEVICE_1 (L647X_CONFIG_PRED_DISABLE)

  ///  Register : TON_MIN 
	///  Minimum on-time in us, range 0.5us to 64us 
  #define L647X_CONF_PARAM_TON_MIN_DEVICE_1 (3.0)

  /// Register : TOFF_MIN 
	///  Minimum off-time in us, range 0.5us to 64us 
  #define L647X_CONF_PARAM_TOFF_MIN_DEVICE_1 (21.0)

  /// Register : T_FAST - field: TOFF_FAST 
	/// Maximum fast decay time , enum L647X_ToffFast_t
  #define L647X_CONF_PARAM_TOFF_FAST_DEVICE_1  (L647X_TOFF_FAST_8us)

  /// Register : T_FAST - field: FAST_STEP 
  ///  Maximum fall step time , enum L647X_FastStep_t
  #define L647X_CONF_PARAM_FAST_STEP_DEVICE_1  (L647X_FAST_STEP_12us)

  /// Register : CONFIG - field : TSW 
	/// Switching period, enum L647X_ConfigTsw_t
  #define L647X_CONF_PARAM_TSW_DEVICE_1 (L647X_CONFIG_TSW_048us)
    
#endif /* #ifdef L6472 */
/******************************* Others *************************************/
/// Register : OCD_TH
/// Overcurrent threshold settings via enum L647X_OCD_TH_TypeDef
#define L647X_CONF_PARAM_OCD_TH_DEVICE_1 (L647X_OCD_TH_3375mA)

/// Register : STALL_TH
/// Stall threshold settings in mA, range 31.25mA to 4000mA
#define L647X_CONF_PARAM_STALL_TH_DEVICE_1 (2031.25)

/// Register : ALARM_EN
/// Alarm settings via bitmap enum L647X_ALARM_EN_TypeDef
#ifdef L6470    
#define L647X_CONF_PARAM_ALARM_EN_DEVICE_1 (L647X_ALARM_EN_OVERCURRENT | L647X_ALARM_EN_THERMAL_SHUTDOWN | L647X_ALARM_EN_THERMAL_WARNING | L647X_ALARM_EN_UNDER_VOLTAGE | L647X_ALARM_EN_STALL_DET_A | L647X_ALARM_EN_STALL_DET_B | L647X_ALARM_EN_SW_TURN_ON | L647X_ALARM_EN_WRONG_NPERF_CMD)
#endif
#ifdef L6472    
#define L647X_CONF_PARAM_ALARM_EN_DEVICE_1 (L647X_ALARM_EN_OVERCURRENT | L647X_ALARM_EN_THERMAL_SHUTDOWN | L647X_ALARM_EN_THERMAL_WARNING | L647X_ALARM_EN_UNDER_VOLTAGE | L647X_ALARM_EN_SW_TURN_ON | L647X_ALARM_EN_WRONG_NPERF_CMD)
#endif

/// Register : STEP_MODE - field : STEP_MODE
/// Step mode settings via enum L647X_STEP_SEL_TypeDef
/// Register : STEP_MODE - field : STEP_MODE
/// Step mode settings via enum L647X_STEP_SEL_TypeDef
#ifdef L6470   
#define L647X_CONF_PARAM_STEP_MODE_DEVICE_1 (L647X_STEP_SEL_1_128)
#endif
#ifdef L6472   
#define L647X_CONF_PARAM_STEP_MODE_DEVICE_1 (L647X_STEP_SEL_1_16)
#endif 

/// Register : STEP_MODE - Field : SYNC_MODE and SYNC_EN
/// Synch. Mode settings via enum L647X_SYNC_SEL_TypeDef
#define L647X_CONF_PARAM_SYNC_MODE_DEVICE_1 (L647X_SYNC_SEL_DISABLED)

/// Register : CONFIG - field : POW_SR
/// Slew rate, enum L647X_CONFIG_POW_SR_TypeDef
#define L647X_CONF_PARAM_SR_DEVICE_1 (L647X_CONFIG_SR_110V_us)

/// Register : CONFIG - field : OC_SD
/// Over current shutwdown enabling, enum L647X_CONFIG_OC_SD_TypeDef
#define L647X_CONF_PARAM_OC_SD_DEVICE_1 (L647X_CONFIG_OC_SD_DISABLE)

/// Register : CONFIG - field : SW_MODE
/// External switch hard stop interrupt mode, enum L647X_CONFIG_SW_MODE_TypeDef
#define L647X_CONF_PARAM_SW_MODE_DEVICE_1 	(L647X_CONFIG_SW_HARD_STOP)

/// Register : CONFIG - field : OSC_CLK_SEL
/// Clock setting , enum L647X_CONFIG_OSC_MGMT_TypeDef
#define L647X_CONF_PARAM_CLOCK_SETTING_DEVICE_1 (L647X_CONFIG_INT_16MHZ_OSCOUT_2MHZ)

/****************************************************************************/
/* Device 2                                                                 */
/****************************************************************************/
  
/**************************** Speed Profile *********************************/

/// Register : ACC
/// Acceleration rate in step/s2, range 14.55 to 59590 steps/s2
#define L647X_CONF_PARAM_ACC_DEVICE_2 (2008.164)

/// Register : DEC 
/// Deceleration rate in step/s2, range 14.55 to 59590 steps/s2
#define L647X_CONF_PARAM_DEC_DEVICE_2 (2008.164)

/// Register : MAX_SPEED 
/// Maximum speed in step/s, range 15.25 to 15610 steps/s 
#define L647X_CONF_PARAM_MAX_SPEED_DEVICE_2 (991.821)

/// Register : MIN_SPEED */
/// Minimum speed in step/s, range 0 to 976.3 steps/s
#define L647X_CONF_PARAM_MIN_SPEED_DEVICE_2 (0)

/// Register : FS_SPD
/// Full step speed in step/s, range 7.63 to 15625 steps/s
#define L647X_CONF_PARAM_FS_SPD_DEVICE_2 (595.093)

/************************ Phase Current Control *****************************/

#ifdef L6470    
/// Register : KVAL_HOLD
/// Hold duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_HOLD_DEVICE_2 (16.02)

/// Register : KVAL_RUN
///  Run duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_RUN_DEVICE_2 (16.02)

/// Register : KVAL_ACC
/// Acceleration duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_ACC_DEVICE_2 (16.02)

/// Register : KVAL_DEC
/// Deceleration duty cycle (torque) in %, range 0 to 99.6%
#define L647X_CONF_PARAM_KVAL_DEC_DEVICE_2 (16.02)

/// Register : CONFIG - field : EN_VSCOMP
/// Motor Supply Voltage Compensation enabling , enum L647X_CONFIG_EN_VSCOMP_TypeDef
#define L647X_CONF_PARAM_VS_COMP_DEVICE_2 (L647X_CONFIG_VS_COMP_DISABLE)

/// Register : MIN_SPEED - field : LSPD_OPT
/// Low speed optimization bit, enum L647X_LSPD_OPT_TypeDef
#define L647X_CONF_PARAM_LSPD_BIT_DEVICE_2 (L647X_LSPD_OPT_OFF)

/// Register : K_THERM
///  Thermal compensation param, range 1 to 1.46875
#define L647X_CONF_PARAM_K_THERM_DEVICE_2 (1)

/// Register : INT_SPEED
///  Intersect speed settings for BEMF compensation in steps/s, range 0 to 3906 steps/s
#define L647X_CONF_PARAM_INT_SPD_DEVICE_2 (61.512)

///  Register : ST_SLP
/// BEMF start slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define L647X_CONF_PARAM_ST_SLP_DEVICE_2 (0.03815)

/// Register : FN_SLP_ACC
/// BEMF final acc slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step 
#define L647X_CONF_PARAM_FN_SLP_ACC_DEVICE_2 (0.06256)

/// Register : FN_SLP_DEC
/// BEMF final dec slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define L647X_CONF_PARAM_FN_SLP_DEC_DEVICE_2 (0.06256)

/// Register : CONFIG - field : F_PWM_INT
/// PWM Frequency Integer division, enum L647X_CONFIG_F_PWM_INT_TypeDef
#define L647X_CONF_PARAM_PWM_DIV_DEVICE_2 (L647X_CONFIG_PWM_DIV_2)

/// Register : CONFIG - field : F_PWM_DEC
/// PWM Frequency Integer Multiplier, enum L647X_CONFIG_F_PWM_INT_TypeDef
#define L647X_CONF_PARAM_PWM_MUL_DEVICE_2 (L647X_CONFIG_PWM_MUL_1)

#endif /* #ifdef L6470 */

#ifdef L6472 
  /// Register : TVAL_ACC 
	/// Acceleration torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_ACC_DEVICE_2  (328.12)
  
  /// Register : TVAL_DEC 
	/// Deceleration torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_DEC_DEVICE_2  (328.12)
  
  /// Register : TVAL_RUN 
	/// Running torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_RUN_DEVICE_2  (328.12)
  
  /// Register : TVAL_HOLD 
	/// Holding torque in mV, range from 7.8mV to 1000 mV 
  #define L647X_CONF_PARAM_TVAL_HOLD_DEVICE_2 (328.12)

  /// Register : CONFIG - field : EN_TQREG 
	/// External torque regulation enabling , enum L647X_ConfigEnTqReg_t
  #define L647X_CONF_PARAM_TQ_REG_DEVICE_2 (L647X_CONFIG_TQ_REG_TVAL_USED)

  /// Register : CONFIG - field : PRED_EN 
	/// Predictive current enabling , enum L647X_ConfigPredEn_t
  #define L647X_CONF_PARAM_PRED_EN_DEVICE_2 (L647X_CONFIG_PRED_DISABLE)

  ///  Register : TON_MIN 
	///  Minimum on-time in us, range 0.5us to 64us 
  #define L647X_CONF_PARAM_TON_MIN_DEVICE_2 (3.0)

  /// Register : TOFF_MIN 
	///  Minimum off-time in us, range 0.5us to 64us 
  #define L647X_CONF_PARAM_TOFF_MIN_DEVICE_2 (21.0)

  /// Register : T_FAST - field: TOFF_FAST 
	/// Maximum fast decay time , enum L647X_ToffFast_t
  #define L647X_CONF_PARAM_TOFF_FAST_DEVICE_2  (L647X_TOFF_FAST_8us)

  /// Register : T_FAST - field: FAST_STEP 
  ///  Maximum fall step time , enum L647X_FastStep_t
  #define L647X_CONF_PARAM_FAST_STEP_DEVICE_2  (L647X_FAST_STEP_12us)

  /// Register : CONFIG - field : TSW 
	/// Switching period, enum L647X_ConfigTsw_t
  #define L647X_CONF_PARAM_TSW_DEVICE_2 (L647X_CONFIG_TSW_048us)
    
#endif /* #ifdef L6472 */

/******************************* Others *************************************/
/// Register : OCD_TH
/// Overcurrent threshold settings via enum L647X_OCD_TH_TypeDef
#define L647X_CONF_PARAM_OCD_TH_DEVICE_2 (L647X_OCD_TH_3375mA)

/// Register : STALL_TH
/// Stall threshold settings in mA, range 31.25mA to 4000mA
#define L647X_CONF_PARAM_STALL_TH_DEVICE_2 (2031.25)

/// Register : ALARM_EN
/// Alarm settings via bitmap enum L647X_ALARM_EN_TypeDef
#ifdef L6470    
#define L647X_CONF_PARAM_ALARM_EN_DEVICE_2 (L647X_ALARM_EN_OVERCURRENT | L647X_ALARM_EN_THERMAL_SHUTDOWN | L647X_ALARM_EN_THERMAL_WARNING | L647X_ALARM_EN_UNDER_VOLTAGE | L647X_ALARM_EN_STALL_DET_A | L647X_ALARM_EN_STALL_DET_B | L647X_ALARM_EN_SW_TURN_ON | L647X_ALARM_EN_WRONG_NPERF_CMD)
#endif
#ifdef L6472    
#define L647X_CONF_PARAM_ALARM_EN_DEVICE_2 (L647X_ALARM_EN_OVERCURRENT | L647X_ALARM_EN_THERMAL_SHUTDOWN | L647X_ALARM_EN_THERMAL_WARNING | L647X_ALARM_EN_UNDER_VOLTAGE | L647X_ALARM_EN_SW_TURN_ON | L647X_ALARM_EN_WRONG_NPERF_CMD)
#endif
    
/// Register : STEP_MODE - field : STEP_MODE
/// Step mode settings via enum L647X_STEP_SEL_TypeDef
/// Register : STEP_MODE - field : STEP_MODE
/// Step mode settings via enum L647X_STEP_SEL_TypeDef
#ifdef L6470   
#define L647X_CONF_PARAM_STEP_MODE_DEVICE_2 (L647X_STEP_SEL_1_128)
#endif
#ifdef L6472   
#define L647X_CONF_PARAM_STEP_MODE_DEVICE_2 (L647X_STEP_SEL_1_16)
#endif 

/// Register : STEP_MODE - Field : SYNC_MODE and SYNC_EN
/// Synch. Mode settings via enum L647X_SYNC_SEL_TypeDef
#define L647X_CONF_PARAM_SYNC_MODE_DEVICE_2 (L647X_SYNC_SEL_DISABLED)

/// Register : CONFIG - field : POW_SR
/// Slew rate, enum L647X_CONFIG_POW_SR_TypeDef
#define L647X_CONF_PARAM_SR_DEVICE_2 (L647X_CONFIG_SR_110V_us)

/// Register : CONFIG - field : OC_SD
/// Over current shutwdown enabling, enum L647X_CONFIG_OC_SD_TypeDef
#define L647X_CONF_PARAM_OC_SD_DEVICE_2 (L647X_CONFIG_OC_SD_DISABLE)

/// Register : CONFIG - field : SW_MODE
/// External switch hard stop interrupt mode, enum L647X_CONFIG_SW_MODE_TypeDef
#define L647X_CONF_PARAM_SW_MODE_DEVICE_2 	(L647X_CONFIG_SW_HARD_STOP)

/// Register : CONFIG - field : OSC_CLK_SEL
/// Clock setting , enum L647X_CONFIG_OSC_MGMT_TypeDef
#define L647X_CONF_PARAM_CLOCK_SETTING_DEVICE_2 (L647X_CONFIG_INT_16MHZ_OSCOUT_2MHZ)


#endif /* __L647X_TARGET_CONFIG_H */
