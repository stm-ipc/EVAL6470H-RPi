#  **********************************************************************************
#  * @file    l647x_linux.c
#  * @author  IPC Rennes
#  * @version V1.0.0
#  * @date    January 7, 2015
#  * @brief   dSpin library for Linux
#  * @note    (C) COPYRIGHT 2015 STMicroelectronics
#  **********************************************************************************
#  * @attention
#  *
#  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
#  *
#  * Redistribution and use in source and binary forms, with or without modification,
#  * are permitted provided that the following conditions are met:
#  *   1. Redistributions of source code must retain the above copyright notice,
#  *      this list of conditions and the following disclaimer.
#  *   2. Redistributions in binary form must reproduce the above copyright notice,
#  *      this list of conditions and the following disclaimer in the documentation
#  *      and/or other materials provided with the distribution.
#  *   3. Neither the name of STMicroelectronics nor the names of its contributors
#  *      may be used to endorse or promote products derived from this software
#  *      without specific prior written permission.
#  *
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  *
#  ***********************************************************************************



# Compilation tools
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

# Other tools
RM = rm -f

# Output file
OBJS = l647x_linux.o gpio.o
LIB = libl647x.a
APPLICATION = motor
STEP_CLOCK_TEST = step_clock_test

# Compilation flag
CFLAGS = -Wall -DL6470 -DUSE_RASPBERRY_PI



ifeq ($(DEBUG),1) 
CFLAGS += -O0 -g
else
CFLAGS += -O2
endif


# Included libs
LDINCL = -Llibs/arm -Lusr/lib
LDFLAGS = -lpthread


SRC = l647x_linux.c  l647x_linux.h  l647x_target_config.h l647x_RPi_host_config.h gpio.c  gpio.h



all: $(LIB) $(APPLICATION) $(STEP_CLOCK_TEST)

lib: $(LIB)

application: $(APPLICATION)

step_clock_test: $(STEP_CLOCK_TEST)

clean:
	$(RM) $(OBJS) $(LIB) $(APPLICATION) $(STEP_CLOCK_TEST) *~

cleanobj:
	$(RM) $(OBJS)

$(LIB): $(OBJS)
	$(AR) rcs $(LIB) $(OBJS)

$(APPLICATION): $(OBJS)
	    $(CC) $(LDINCL) $(LDFLAGS) $(CFLAGS) -o $(APPLICATION) $(OBJS) main.c
	    
$(STEP_CLOCK_TEST): $(OBJS)
	$(CC) $(LDINCL) $(LDFLAGS) $(CFLAGS) -o $(STEP_CLOCK_TEST) $(OBJS) main_stepclock.c
#dspin.o: $(SRC)

.c.o:
	$(CC) $(CFLAGS) -c -o $@ $<


