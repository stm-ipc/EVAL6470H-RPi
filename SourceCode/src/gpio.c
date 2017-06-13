/**************************************************************************//**
  * @file    gpio.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 7, 2015
  * @brief   Library for Raspberry Pi GPIO management
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


#include "gpio.h"



/*
 * Internal variable
 */

// GPIO management variable declaration
gpio_t gpio_in[GPIO_MAX_GPIO];
gpio_t gpio_out[GPIO_MAX_GPIO];

char direction_table[2][5] = {"in", "out"};
char active_level_table[2][5] = {"high", "low"};
char edge_table[4][10] = {"none", "falling", "rising", "both"};

// GPIO interrupt variables
pthread_t loop_thread_id=0;
volatile uint8_t stay_in_loop_thread=1;
struct pollfd poll_fd[GPIO_MAX_GPIO];

// GPIO clock variables
pthread_t step_thread_id=0;
uint8_t stay_in_step_thread=1;
uint8_t gpio_clk_pin;

useconds_t half_clk;


struct sigaction sa;
struct itimerval timer;

static volatile int gpio_clk_file_id;


/***************************************************************
 * *************************************************************
 * PRIVATE FUNCTIONS
 * *************************************************************
 ***************************************************************/


/******************************************************//**
 * @brief  Activate a specific GPIO pin
 * @param[in] GPIO pin to activate
 * @retval 0 = OK ,  other value = ERROR
 **********************************************************/
int gpio_export(uint8_t gpio_pin)
{
	int file_id, len;
	char buffer[GPIO_MAX_BUFFER_SIZE];

	file_id = open(GPIO_SYS_FILE_PATH "/export", O_WRONLY);
	if (file_id < 0) {
		perror("opening gpio/export");
		return file_id;
	}

	len = snprintf(buffer, GPIO_MAX_BUFFER_SIZE, "%d", gpio_pin);
	write(file_id, buffer, len);
	close(file_id);

	return 0;
}

/******************************************************//**
 * @brief  De-activate a specific GPIO pin
 * @param[in] GPIO pin to de-activate
 * @retval 0 = OK ,  other value = ERROR
 **********************************************************/
int gpio_unexport(uint8_t gpio_pin)
{
	int file_id, len;
	char buffer[GPIO_MAX_BUFFER_SIZE];

	file_id = open(GPIO_SYS_FILE_PATH "/unexport", O_WRONLY);
	if (file_id < 0) {
		perror("opening gpio/unexport");
		return file_id;
	}

	len = snprintf(buffer, GPIO_MAX_BUFFER_SIZE, "%d", gpio_pin);
	write(file_id, buffer, len);

	close(file_id);
	return 0;
}

/******************************************************//**
 * @brief  Set edge value for a specific GPIO pin.
 * @param[in] GPIO pin to activate
 * @param[in] Edge value, can be none, falling, rising, both
 * @retval 0 = OK ,  other value = ERROR
 **********************************************************/
int gpio_set_edge(uint8_t gpio_pin, uint8_t edge)
{
	int file_id;
	char buffer[GPIO_MAX_BUFFER_SIZE];

	snprintf(buffer, GPIO_MAX_BUFFER_SIZE, GPIO_SYS_FILE_PATH "/gpio%d/edge", gpio_pin);

	file_id = open(buffer, O_WRONLY);
	if (file_id < 0) {
		perror("Opening gpioXX/edge");
		return file_id;
	}

	write(file_id, edge_table[edge], strlen(edge_table[edge]) + 1);

	close(file_id);
	return 0;
}

/******************************************************//**
 * @brief  Set level for a specific GPIO pin.
 * @param[in] GPIO pin to activate
 * @param[in] Level value, can be high or low
 * @retval 0 = OK ,  other value = ERROR
 **********************************************************/
int gpio_set_level(uint8_t gpio_pin, uint8_t level)
{
	int file_id;
	char buffer[GPIO_MAX_BUFFER_SIZE];

	snprintf(buffer, GPIO_MAX_BUFFER_SIZE, GPIO_SYS_FILE_PATH "/gpio%d/active_low", gpio_pin);

	file_id = open(buffer, O_WRONLY);
	if (file_id < 0) {
		perror("Opening gpioXX/active_low");
		return file_id;
	}

	write(file_id, active_level_table[level], strlen(active_level_table[level]) + 1);

	close(file_id);
	return 0;
}

/******************************************************//**
 * @brief  Set level for a specific GPIO pin.
 * @param[in] GPIO pin to activate
 * @param[in] Level value, can be high or low
 * @retval 0 = OK ,  other value = ERROR
 **********************************************************/
int gpio_set_direction(uint8_t gpio_pin, uint8_t direction)
{
	int file_id, nb_write;
	char buffer[GPIO_MAX_BUFFER_SIZE];

	snprintf(buffer, GPIO_MAX_BUFFER_SIZE, GPIO_SYS_FILE_PATH "/gpio%d/direction", gpio_pin);

	file_id = open(buffer, O_WRONLY);
	if (file_id < 0) {
		perror("Opening gpio/direction");
		return file_id;
	}

	nb_write = write(file_id, direction_table[direction], sizeof(direction_table[direction]));
	if( nb_write != sizeof(direction_table[direction]))
		return -1;

	close(file_id);
	return 0;
}

/******************************************************//**
 * @brief  Find the index in the GPIO IN table of the
 * specified GPIO pin.
 * @param[in] GPIO pin
 * @retval index value or -1 if not found.
 **********************************************************/
int gpio_find_entry_gpio_in( uint8_t gpio_pin)
{
	uint8_t cpt=0;

	while( (cpt<GPIO_MAX_GPIO) && (gpio_in[cpt].gpio_pin != gpio_pin))	cpt++;
	if( cpt == GPIO_MAX_GPIO )
	{
		perror("GPIO entry not found in table.");
		return -1;
	}

	return cpt;
}

/******************************************************//**
 * @brief  Find the index in the GPIO OUT table of the
 * specified GPIO pin.
 * @param[in] GPIO pin
 * @retval index value or -1 if not found.
 **********************************************************/
int gpio_find_entry_gpio_out( uint8_t gpio_pin)
{
	uint8_t cpt=0;

	while( (cpt<GPIO_MAX_GPIO) && (gpio_out[cpt].gpio_pin != gpio_pin))	cpt++;
	if( cpt == GPIO_MAX_GPIO )
	{
		perror("GPIO entry not found in table.");
		return -1;
	}

	return cpt;
}


/******************************************************//**
 * @brief  Fill the poll table. This table is used by the
 * polling thread to check status of input GPIO
 * @param[in] none
 * @retval none
 **********************************************************/
uint8_t gpio_fill_poll_table(void)
{
	uint8_t cpt;

	// Clean fd table
	memset((void*)poll_fd, 0, sizeof(poll_fd));

	cpt=0;
	while( gpio_in[cpt].file_id)
	{
		poll_fd[cpt].fd = gpio_in[cpt].file_id;
		poll_fd[cpt].events = POLLPRI;
		cpt++;
	}

	return cpt;
}

/******************************************************//**
 * @brief  Polling loop function call in a specific thread.
 * Process stay in this function while volatile variable
 * 'stay_in_loop_thread' is true. In this function, all
 * configured GPIO IN pin are scanned to detect event.
 * @param[in] pointer on data sent to process. None in our case.
 * @retval none.
 **********************************************************/
void *gpio_thread_loop(void *ptr)
{

	int timeout = POLL_TIMEOUT;
	int rc;
	char buf[64];
	uint8_t cpt;
	uint8_t nb_fd;

	while( stay_in_loop_thread)
	{
		nb_fd = gpio_fill_poll_table();

		if( !nb_fd)
		{
			usleep(200000);
			continue;
		}

		rc = poll(poll_fd, nb_fd, timeout);

		if (rc < 0) {
			perror("Poll() failed");
			return 0;
		}

		for( cpt=0; cpt<nb_fd; cpt++)
		{
			if ( poll_fd[cpt].revents & POLLPRI)
			{
				int val;

				lseek( poll_fd[cpt].fd, 0, SEEK_SET);

				read(poll_fd[cpt].fd, buf, 64);
				val = gpio_get_value(gpio_in[cpt].gpio_pin);
				if( (0 == val) &&
					(NULL!=gpio_in[cpt].InterruptCallback) )
				{
					printf("poll():   gpio %d (id=%d) interrupt occurred !!\n",
							gpio_in[cpt].gpio_pin, poll_fd[cpt].fd);
					(*(gpio_in[cpt].InterruptCallback))();
				}
			}
		}
	}

	return NULL;
}


/******************************************************//**
 * @brief  While 'stay_in_step_thread' is true, this function
 * will generate a step clock for motor on pin L647X_STEP_CLOCK_PIN.
 * @param[in] Pointer on data sent to process. None in our case.
 * @retval None
 **********************************************************/
void *gpio_thread_step(void *ptr)
{
	while( stay_in_step_thread)
	{
		write( gpio_clk_file_id , "1", 2);
		usleep(half_clk);
		write( gpio_clk_file_id , "0", 2);
		usleep(half_clk);
	}

	printf("Exit from gpio_thread_step\n");
	return NULL;
}


void step_clock_handler (int signum)
{
	static char out_level = 0;
	static char out_msg[]={0,0};

	out_msg[0] = '0' + out_level;
	write( gpio_clk_file_id , out_msg, 2);

	out_level++;
	if(out_level>1) out_level=0;
}



/******************************************************//**
 * @brief  Start the thread loop used to detect event that
 * may occurred on GPIO IN.
 * @param[in] None
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_start_loop( void)
{
	// Create loop thread
	stay_in_loop_thread = 1;
	if( 0 != pthread_create(&loop_thread_id, NULL, gpio_thread_loop, NULL) )
	{
		perror("Can't create loop thread");
		return -1;
	}

	return 0;
}


/***************************************************************
 * *************************************************************
 * PUBLIC FUNCTIONS
 * *************************************************************
 ***************************************************************/

/******************************************************//**
 * @brief   library initialization function. It will clear
 * table and start the poll thread.
 * @param[in] None
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_init( void)
{
	uint8_t i;

	step_thread_id = 0;

	// init structure
	for( i=0; i<GPIO_MAX_GPIO; i++)
	{
		memset( &gpio_in[i], 0x0, sizeof(gpio_t));
		memset( &gpio_out[i], 0x0, sizeof(gpio_t));
	}

	return gpio_start_loop();

}


/******************************************************//**
 * @brief  Open GPIO port (in or out) and configure a new entry
 * for this GPIO in the correct table gpio_in or gpio_out.
 * @param[in] gpio_in - port number
 * @param[in] direction - in or out
 * @param[in] active_level - low (0) or high (1)
 * @param[in] edge - none or falling or rising or both
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_open( uint8_t gpio_pin, t_direction direction, t_active_level active_level, t_edge edge)
{
	char buffer[GPIO_MAX_BUFFER_SIZE];
	gpio_t *gpio;
	int file_flags;
	uint8_t cpt;

	if( 	(gpio_export( gpio_pin) < 0) ||
			(gpio_set_direction( gpio_pin, direction) < 0) ||
//			(gpio_set_level( gpio_pin, active_level) < 0) ||
			(gpio_set_edge( gpio_pin, edge) < 0) )
		return -1;

	if( direction == in)
	{
		file_flags = O_RDONLY | O_NONBLOCK;
		gpio = gpio_in;
	}
	else
	{
		file_flags = O_WRONLY;
		gpio = gpio_out;
	}


	// Fill gpio structure and open file descriptor
	cpt = 0;
	while( (cpt<GPIO_MAX_GPIO) && (gpio[cpt].file_id != 0))	cpt++;


	if( cpt<GPIO_MAX_GPIO )
	{
		gpio[cpt].gpio_pin = gpio_pin;
		gpio[cpt].direction = direction;
		gpio[cpt].active_level = active_level;
		gpio[cpt].edge = edge;


		snprintf(buffer, GPIO_MAX_BUFFER_SIZE, GPIO_SYS_FILE_PATH "/gpio%d/value", gpio_pin);
		gpio[cpt].file_id = open( buffer, file_flags);
		if( gpio[cpt].file_id < 0)
		{
			perror("Opening gpioXX/value");
			return gpio[cpt].file_id;
		}


		return 0;
	}
	else
	{
		printf("GPIO %d not opened - exceed nbr max of GPIO!!\n", gpio_pin);
		return -1;
	}

}


/******************************************************//**
 * @brief  Set value on specified pin.
 * @param[in] gpio_pin number
 * @param[in] value - 0 or 1
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_set_value(uint8_t gpio_pin, uint8_t value)
{
	int nb_write;
	int cpt=gpio_find_entry_gpio_out( gpio_pin);

	if( cpt<0) return cpt;

	if (value)
		nb_write = write( gpio_out[cpt].file_id , "1", 2);
	else
		nb_write = write( gpio_out[cpt].file_id , "0", 2);

	if( 2 != nb_write)
		return -1;

	return 0;
}


/******************************************************//**
 * @brief  Get value available on the specified pin.
 * @param[in] gpio_pin number
 * @retval Value available on the pin: 0 or 1
 **********************************************************/
int gpio_get_value(uint8_t gpio_pin)
{
	int len;
	char val[5];
	int cpt=gpio_find_entry_gpio_in( gpio_pin);

	if( cpt<0)
	{
		printf("gpio%d not found !!!\n", gpio_pin);
		return cpt;
	}

	// Move position to beginning of file
	lseek( gpio_in[cpt].file_id, 0, SEEK_SET);

	len = read(gpio_in[cpt].file_id, val, 5);

	if( len < 1)
		return -1;

	return atoi(val);
}


/******************************************************//**
 * @brief  Configure the callback function that will be called
 * when event occurred on the specified gpio pin.
 * @param[in] gpio_pin number
 * @param[in] Callback function pointer
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_set_callback( uint8_t gpio_pin, void (*callback)(void))
{
	int cpt=gpio_find_entry_gpio_in( gpio_pin);

	if( cpt<0)
	{
		printf("ERROR - GPIO not found!!\n");
		return cpt;
	}
	gpio_in[cpt].InterruptCallback = callback;

	printf("Callback for GPIO %d is set : %ld\n", gpio_in[cpt].gpio_pin, (long)gpio_in[cpt].InterruptCallback);

	return 0;
}


/******************************************************//**
 * @brief  Close the input GPIO specified in parameter and
 * clear the associated information in the gpio_in table.
 * @param[in] gpio_pin number
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_in_close( uint8_t gpio_pin)
{
	int cpt=gpio_find_entry_gpio_in( gpio_pin);

	// Indicate to the system that GPIO <gpio_nbr> shall be de-activated
	gpio_unexport(gpio_pin);

	if( cpt >= 0 )
	{
		if( gpio_in[cpt].file_id != 0)
			close( gpio_in[cpt].file_id);
		memset( &gpio_in[cpt], 0x0, sizeof(gpio_t));
	}

	return 0;
}


/******************************************************//**
 * @brief  Close the output GPIO specified in parameter and
 * clear the associated information in the gpio_out table.
 * @param[in] gpio_pin number
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_out_close( uint8_t gpio_pin)
{
	int cpt=gpio_find_entry_gpio_out( gpio_pin);

	// Indicate to the system that GPIO <gpio_nbr> shall be de-activated
	gpio_unexport(gpio_pin);

	if( cpt >= 0 )
	{
		if( gpio_out[cpt].file_id != 0)
			close( gpio_out[cpt].file_id);
		memset( &gpio_out[cpt], 0x0, sizeof(gpio_t));
	}

	return 0;
}


/******************************************************//**
 * @brief  Start the step clock at the indicated frequency
 * on the gpio_pin specified. Only one step clock can be
 * configured.
 * @param[in] gpio_pin number
 * @param[in] Frequency in hertz
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_start_clock(uint8_t gpio_pin, uint32_t frequency)
{
	int gpio_idx;

	if( step_thread_id)
	{
		// !!!
		// Clock already started, so change just the frequency
		// !!!
		gpio_change_clock(frequency);

		return 0;
	}

	if( 0 > gpio_open( gpio_pin, out, high, falling) )
	{
		perror("Opening gpio");
		return -1;
	}

	gpio_idx = gpio_find_entry_gpio_out(gpio_pin);
	gpio_clk_file_id = gpio_out[gpio_idx].file_id;
	gpio_clk_pin = gpio_pin;

	/* Install timer_handler as the signal handler for SIGVTALRM. */
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = &step_clock_handler;
	sigaction (SIGALRM, &sa, NULL);

	/* Configure the timer to expire after 100\B5s the first time. */
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = 100;

	// Create step thread
	gpio_change_clock(frequency);  // Initial value in \B5s

/*
	stay_in_step_thread=1;

	if( 0 != pthread_create(&step_thread_id, NULL, gpio_thread_step, NULL) )
	{
		perror("Can't create step thread");
		return -1;
	}
*/
	return 0;
}


/******************************************************//**
 * @brief  Change the gpio step clock frequency.
 * @param[in] New frequency in hertz
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
inline void gpio_change_clock( uint32_t frequency)
{
	uint32_t half_value = (uint32_t)1000000 / (frequency * (uint32_t)2);

	printf("Half value in \B5s is : %d\n", half_value);
	half_clk = (useconds_t)half_value;

	/* ... and every 250 msec after that. */
	timer.it_interval.tv_sec = 0;  ///!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	timer.it_interval.tv_usec = half_value;
	/* Start a virtual timer. It counts down whenever this process is
	  executing. */
	setitimer (ITIMER_REAL, &timer, NULL);
}


/******************************************************//**
 * @brief  Stop the gpio step clock and close the
 * associated gpio.
 * @param[in] None
 * @param[in] Frequency in hertz
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_stop_clock( void)
{
	stay_in_step_thread=0;

    //pthread_join( step_thread_id, NULL);
    step_thread_id = 0;

	return gpio_out_close(gpio_clk_pin);
}


/******************************************************//**
 * @brief  Close  library, stop loop thread and, if needed,
 * stop step thread. Clear all tables and variables.
 * @param[in] None
 * @retval Status: 0 = ok; other value = KO
 **********************************************************/
int gpio_end( void)
{
	uint8_t i;


	stay_in_loop_thread = 0;
	stay_in_step_thread = 0;

    pthread_join( loop_thread_id, NULL);

    if( step_thread_id)
    	pthread_join( step_thread_id, NULL);

	// init structure
	for( i=0; i<GPIO_MAX_GPIO; i++)
	{
		if( gpio_in[i].file_id != 0)
		{
			close( gpio_in[i].file_id);
			gpio_unexport( gpio_in[i].gpio_pin);
			memset( &gpio_in[i], 0x0, sizeof(gpio_t));
		}
		if( gpio_out[i].file_id != 0)
		{
			close( gpio_out[i].file_id);
			gpio_unexport( gpio_out[i].gpio_pin);
			memset( &gpio_out[i], 0x0, sizeof(gpio_t));
		}
	}

	return 0;
}
