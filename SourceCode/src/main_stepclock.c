/*
 * main.c
 *
 *  Created on: Jan 6, 2015
 *      Author: B. Diraison
 *
 * Updated on: April 25, 2016
 *	Author: B. diraison
 *	Add of board support.
 */

#include <stdlib.h>
#include "l647x_linux.h"
#include <unistd.h>


void MyBusyInterruptHandler(void);
void MyFlagInterruptHandler(void);

void usage(void)
{
	printf("Usage: step_clock_test <step clock pin (A, B, C or D)> <frequency (Hz)>\n");
	exit(-1);
}

int menu(void)
{
	unsigned int choice;
	printf("\nMenu :\n\t1-Change frequency\n\t2-Change output (not available)\n\n\t0-Quit\n");
	printf("Your choice :");

	scanf("%d", &choice);

	return (choice);
}

int select_pin( char choice)
{
	switch(choice)
	{
	case 'A': case 'a':
		return(L647X_STEP_CLOCK_A_PIN);
		break;

	case 'B': case 'b':
		return(L647X_STEP_CLOCK_B_PIN);
		break;

	case 'C': case 'c':
		return(L647X_STEP_CLOCK_C_PIN);
		break;

	case 'D': case 'd':
		return(L647X_STEP_CLOCK_D_PIN);
		break;

	default:
		return(-1);
	}
}


int main(int argc, char *argv[])
{
	uint8_t step_clock_pin;
	int choice;
	uint16_t frequency;
	FILE *pMotorConfigFile = NULL;
	char clock_pin_choice;




	if( argc < 2)
		usage();

	if( -1 == (choice=select_pin( argv[1][0])) )
		usage();
	step_clock_pin = (uint8_t)choice;

	frequency = atoi(argv[2]);



	// Open motor configuration file is present
	pMotorConfigFile = fopen("/home/pi/motor_config.txt", "r");
	if( NULL != pMotorConfigFile)
		L647X_LoadMotorConfigValuesFromFile( pMotorConfigFile);
	else
		L647X_SetMotorConfigToPredefinedValues();


	/* Start the L647X library to use nbr_of_motor device(s) */
	L647X_Begin( 2);

	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	L647X_AttachFlagInterrupt( &MyFlagInterruptHandler);

	/* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
	L647X_AttachBusyInterrupt( &MyBusyInterruptHandler);


	/* Enable Step Clock Mode */
	L647X_CmdStepClock(0, FORWARD);

	/* Enable the step clock */
	L647X_StartStepClock( step_clock_pin, frequency);

#if 0
	while( (choice=menu()) != 99)
	{
		switch( choice)
		{
		case 1:
			do {
				printf("Select clock pin : ");
				scanf("%c", &clock_pin_choice);
			} while( -1 == (choice=select_pin( clock_pin_choice)) );
			step_clock_pin = (uint8_t)choice;

			printf("Enter frequency in Hz : ");
			scanf("%d", &choice);
			frequency = (uint16_t)choice;

			L647X_StartStepClock( step_clock_pin, frequency);

			break;

		default:
			printf("!!!Bad choice!!!\n\n");
		}
	}
#endif

	while(1);

	printf(" Your choice is %d\n\n", choice);


	/* Stop the step clock */
	L647X_StopStepClock();

	/* Close */
	L647X_Close();

	return 0;
}



/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  /* Get the value of the status register via the L647X command GET_STATUS */
  uint16_t statusRegister;

  printf("Enter in MyFlagInterruptHandler function\n");

  statusRegister = L647X_CmdGetStatus(0);

  printf("statusRegister = %d\n", statusRegister);


  /* Check HIZ flag: if set, power brigdes are disabled */
  if((statusRegister & L647X_STATUS_HIZ) == L647X_STATUS_HIZ)
  {
	  // HIZ state
	  printf("HIZ flag is set\n");
  }

  /* Check BUSY flag: if not set, a command is under execution */
  if ((statusRegister & L647X_STATUS_BUSY) == 0)
  {
	  // BUSY
	  printf("BUSY flag is set\n");
  }

  /* Check SW_F flag: if not set, the SW input is opened */
  if ((statusRegister & L647X_STATUS_SW_F ) == 0)
  {
	  // SW OPEN
	  printf("SW_F is set\n");
  }
  else
  {
	  // SW CLOSED
	  printf("SW CLOSED is set\n");
  }
  /* Check SW_EVN bit */
  if ((statusRegister & L647X_STATUS_SW_EVN) == L647X_STATUS_SW_EVN)
  {
	  // switch turn_on event
	  printf("SW_EVN is set\n");
  }
  /* Check direction bit */
  if ((statusRegister & L647X_STATUS_DIR) == 0)
  {
	  // BACKWARD
	  printf("BACKWARD is set\n");
  }
  else
  {
	  // FORWARD
	  printf("FORWARD is set\n");
  }
  if ((statusRegister & L647X_STATUS_MOT_STATUS) ==   L647X_STATUS_MOT_STATUS_STOPPED )
  {
	  // MOTOR STOPPED
	  printf("MOTOR STOPPED bit is set\n");
  }
  else  if ((statusRegister & L647X_STATUS_MOT_STATUS) ==   L647X_STATUS_MOT_STATUS_ACCELERATION )
  {
	  // MOTOR ACCELERATION
	  printf("MOTOR ACCELERATION bit is set\n");
  }
  else  if ((statusRegister & L647X_STATUS_MOT_STATUS) ==   L647X_STATUS_MOT_STATUS_DECELERATION )
  {
	  // MOTOR DECELERATION
	  printf("MOTOR DECELERATION bit is set\n");
  }
  else  if ((statusRegister & L647X_STATUS_MOT_STATUS) ==   L647X_STATUS_MOT_STATUS_CONST_SPD )
  {
	  // MOTOR RUNNING AT CONSTANT SPEED
	  printf("MOTOR RUNNING AT CONSTANT SPEED bit is set\n");
  }

  /* Check Not Performed Command flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the L647X */
  /* while it is in HIZ state */
  if ((statusRegister & L647X_STATUS_NOTPERF_CMD) == L647X_STATUS_NOTPERF_CMD)
  {
	  // Command can't be performed
	  printf("!! Command can't be performed !!\n");
  }

  /* Check Wrong Command Error flag: if set, the command does not exist */
  if ((statusRegister & L647X_STATUS_WRONG_CMD) == L647X_STATUS_WRONG_CMD)
  {
      // Command does not exist
	  printf("!! Command does not exist !!\n");
  }

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L647X_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out
	  printf("!! Under voltage lock-out !!\n");
  }


  /* Check thermal warning flags: if  not set, there is a thermal warning */
  if ((statusRegister & L647X_STATUS_TH_WRN) != 0)
  {
    //thermal warning
  }

  /* Check thermal shutdown flags: if  not set, there is a thermal shutdown */
  if ((statusRegister & L647X_STATUS_TH_SD) != 0)
  {
    //thermal shutdown
  }

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L647X_STATUS_OCD) == 0)
  {
    //overcurrent detection
  }
#ifdef L6470
  /* Check Step Loss A flag: if not set, there is a Stall condition on bridge A */
  if ((statusRegister & L647X_STATUS_STEP_LOSS_A) == 0)
  {
    //stall detected on bridge A
  }

  /* Check Step Loss B flag: if not set, there is a Stall condition on bridge B */
  if ((statusRegister & L647X_STATUS_STEP_LOSS_B) == 0)
  {
    //stall detected on bridge B
  }
#endif
  /* Check Step Clock Mode flag: if  set, the device is working in step clock mode */
  if ((statusRegister & L647X_STATUS_SCK_MOD) == L647X_STATUS_SCK_MOD)
  {
    //step clock mode is enabled
  }
}


/**
  * @brief  This function is the User handler for the busy interrupt
  * @param  None
  * @retval None
  */
void MyBusyInterruptHandler(void)
{
	printf("Function 'MyBusyInterruptHandler\n");

//   if (L647X_CheckBusyHw())
	if(1)
	{
      /* Busy pin is low, so at list one L647X chip is busy */
     /* To be customized (for example Switch on a LED) */
   }
   else
   {
     /* To be customized (for example Switch off a LED) */
   }
}


