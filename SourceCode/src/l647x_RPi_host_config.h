/**************************************************************************//**
  * @file    l647x_RPi_host_config.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 22, 2015
  * @brief   Predefined values for the Raspberry Pi host platform
  * 		 and for the devices parameters
  * @note    (C) COPYRIGHT 2015 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

#ifndef __L647X_RPI_HOST_CONFIG_H
#define __L647X_RPI_HOST_CONFIG_H

/******************************************************************************/
/* Raspberry Pi plateform definitions                                          */
/******************************************************************************/

/// SPI driver file path
#define L647X_SPI_FILE_PATH "/dev/spidev0.0"

#define L647X_LOW  0
#define L647X_HIGH 1

/// GPIO Pin used for the L647X busy pin
#define L647X_BUSY_PIN   (22)
/// GPIO port used for the L647X busy pin
///#define L647X_BUSY_PORT   (GPIOB)

/// GPIO Pin used for the L647X flag pin
#define L647X_FLAG_PIN   (23)
/// GPIO port used for the L647X flag pin
///#define L647X_FLAG_PORT   (GPIOA)

/// GPIO Pin used for the L647X step clock pin
#define L647X_STEP_CLOCK_A_PIN  (4)
#define L647X_STEP_CLOCK_B_PIN  (17)
#define L647X_STEP_CLOCK_C_PIN  (27)
#define L647X_STEP_CLOCK_D_PIN  (24)

/// GPIO Port used for the L647X step clock
///#define L647X_STEP_CLOCK_PORT  (GPIOC)

/// GPIO Pin used for the L647X reset pin
#define L647X_STBY_RESET_PIN  (25)
/// GPIO port used for the L647X reset pin
///#define L647X_STBY_RESET_PORT (GPIOA)

/// GPIO Pin used for the L647X SPI chip select pin
//#define L647X_CS_PIN  (GPIO_PIN_6)
/// GPIO port used for the L647X SPI chip select  pin
///#define L647X_CS_PORT (GPIOB)




#endif // __L647X_HOST_CONFIG_H
