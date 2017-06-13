/**************************************************************************//**
  * @file    gpio.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 7, 2015
  * @brief   Header for GPIO management
  * @note    (C) COPYRIGHT 2014 STMicroelectronics
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

#ifndef _GPIO_H_INCLUDED
#define _GPIO_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>   /* standard types definitions                      */
#include <sys/types.h>

#include <signal.h>
#include <sys/time.h>

#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>


// GPIO Pin number definition


// Constant definition
#define GPIO_SYS_FILE_PATH "/sys/class/gpio"

#define GPIO_MAX_BUFFER_SIZE 128

#define GPIO_MAX_GPIO 5

#define POLL_TIMEOUT (2 * 1000) /* 2 seconds */


// New type definition

/*--- direction ---*/
typedef enum {
	in = 0,
	out
} t_direction;

/*--- active level ---*/
typedef enum {
	low = 0,
	high = 1
} t_active_level;

/*--- edge ---*/
typedef enum {
	none = 0,
	falling,
	rising,
	both
} t_edge;



// Structure declaration

/*--- GPIO management struct ---*/
typedef struct {
	int file_id;
	uint8_t gpio_pin;

	/* GPIO configuration values */
	t_direction direction;
	t_active_level active_level;
	t_edge edge;

	/* Interrupt data  (only in case of 'in' GPIO */
	void (*InterruptCallback)(void);
} gpio_t;


// Functions declaration
int gpio_init( void);
int gpio_open( uint8_t gpio_pin, t_direction direction, t_active_level active_level, t_edge edge);
int gpio_set_value(uint8_t gpio_pin, uint8_t value);
int gpio_get_value(uint8_t gpio_pin);
int gpio_close( uint8_t gpio_pin);
int gpio_end( void);
int gpio_start_loop( void);

int gpio_start_clock(uint8_t gpio_pin, uint32_t frequency);
inline void gpio_change_clock( uint32_t frequency);
int gpio_stop_clock( void);

int gpio_set_callback( uint8_t gpio_pin, void (*callback)(void));



#endif  /* _GPIO_H_INCLUDED */
