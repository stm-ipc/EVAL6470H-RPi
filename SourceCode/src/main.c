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



int main(int argc, char *argv[])
{
	int param1, param2;
	
	uint8_t nbr_motor = 0;

	uint8_t idx_motor = 0;
	uint8_t action = 0;
	int32_t nb_steep = 0;
	uint8_t loop;
	FILE *pfile = NULL;
	FILE *pMotorConfigFile = NULL;

	// Read the number of board in temp file
	while( !pfile) 
		pfile = fopen("/tmp/nbOfBoard.txt", "r");

	fscanf( pfile, "%d", &param1);
	fclose( pfile);
	pfile = NULL;
	system("rm /tmp/nbOfBoard.txt");

	// Open motor configuration file is present
	pMotorConfigFile = fopen("/home/pi/motor_config.txt", "r");
	if( NULL != pMotorConfigFile)
		L647X_LoadMotorConfigValuesFromFile( pMotorConfigFile);
	else
		L647X_SetMotorConfigToPredefinedValues();

	/* 2 motor per board */
	nbr_motor = 2 * (uint8_t)param1;
	
	/* Start the L647X library to use nbr_of_motor device(s) */
	L647X_Begin( nbr_motor);

	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	L647X_AttachFlagInterrupt( &MyFlagInterruptHandler);

	/* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
	L647X_AttachBusyInterrupt( &MyBusyInterruptHandler);

	for(;;) {
		while( !pfile) 
			pfile = fopen("/tmp/cmd.txt", "r");

		fscanf( pfile, "%d:%d:%d", &param1, &param2, &nb_steep);
		fclose( pfile);
		pfile = NULL;
		system("rm /tmp/cmd.txt");

		idx_motor = (uint8_t)param1;
		action = (uint8_t)param2;

		if( action == 99)
		{
			// Shutdown the system.
			printf("!!!!=> Shutdown the system.<=!!!!\n");
			L647X_Close();
			system("/sbin/shutdown -h now");
		}

		printf("#################################\n");
		printf("index motor = %d\n", idx_motor);
		printf("action = %d\n", action);
		printf("nbr steep = %d\n", nb_steep);
		printf("#################################\n\n\n");

		switch( action)
		{
			case 1:  // Go To
				L647X_CmdGoTo( idx_motor,nb_steep);
				break;

			case 2:  // Move
				if( nb_steep >= 0)
					L647X_CmdMove( idx_motor, FORWARD, nb_steep);
				else
					L647X_CmdMove( idx_motor, BACKWARD, -nb_steep);
				break;

			case 3:  // Run
				if( nb_steep >= 0)
					L647X_CmdRun( idx_motor, FORWARD, Speed_Steps_to_Par(nb_steep));
				else
					L647X_CmdRun( idx_motor, BACKWARD, Speed_Steps_to_Par(-nb_steep));
				break;

			case 4:  // Hard Stop
				L647X_CmdHardStop( idx_motor);
				break;

			case 5:  // Reload devices register values
				for (loop = 0; loop < nbr_motor; loop++)
				{
					L647X_CmdHardStop(loop);
					// Set all registers to their values
					L647X_SetRegisterValues(loop);
					// Put the L647X in HiZ state
					L647X_CmdHardHiZ(loop);
				}
				 L647X_FetchAndClearAllStatus();

			default:
				break;
		}
	}

	/* Wait for the motor of device 0 ends moving */
	// L647X_WaitWhileActive(cpt_motor);
	sleep(3);

	/* Close */
	//L647X_Close();
#if 0

	sleep(2);
	/* Request soft stop of device 0 */
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===>  Send Soft STOP command to device number %d\n", cpt_motor);
		L647X_CmdSoftStop(cpt_motor);
	}


	sleep(2);

	/* Move of 16000 steps in the FW direction */
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n==>  Move motor %d of 16000 steps in the FW direction\n", cpt_motor);

		L647X_CmdMove(cpt_motor, FORWARD, 160000);
	}

	sleep(2);

	/* Request device to go to position -6400 */
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===>  Device %d go to position -6400\n", cpt_motor);
		L647X_CmdGoTo( cpt_motor,-6400);

		/* Wait for the motor of device 0 ends moving */
		L647X_WaitWhileActive(cpt_motor);
	}

	/* Wait for 2 seconds */
	sleep(2);

	/* Step clock mode example */
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===>  Step clock mode example for motor %d\n", cpt_motor);

		/* Enable Step Clock Mode */
		L647X_CmdStepClock(cpt_motor, FORWARD);

		/* Enable the step clock at 333 Hz */
		L647X_StartStepClock(1000);

		/* Let the motor runs for 5 second at 333 step/s */
		sleep(5);

		/* Stop the step clock */
		L647X_StopStepClock();
	}

	/* Wait for 2 seconds */
	sleep(2);


	//----- Move of 16000 steps in the BW direction
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Move device %d of 16000 steps in the BW direction\n", cpt_motor);

		/* Move device of 16000 steps in the BACKWARD direction*/
		L647X_CmdMove( cpt_motor, BACKWARD, 16000);

		/* Wait for the motor of device ends moving */
		L647X_WaitWhileActive(cpt_motor);

		/* Set the current position of device  to be the Home position */
		L647X_CmdResetPos(cpt_motor);
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- Go to position -6400
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d Go to position -6400\n", cpt_motor);

		/* Request device to go to position -6400 */
		L647X_CmdGoTo(cpt_motor,-6400);

		/* Wait for the motor ends moving */
		L647X_WaitWhileActive(cpt_motor);

		/* Get current position of device 0*/
		pos = L647X_GetPosition(cpt_motor);

		/* Set the current position of device 0 to be the Mark position */
		if (pos == -6400)
		{
			L647X_SetMark(cpt_motor);
		}
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- Go Home
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d Go home\n", cpt_motor);

		/* Request device to go to Home */
		L647X_CmdGoHome(cpt_motor);
		L647X_WaitWhileActive(cpt_motor);

		/* Get current position of device */
		pos = L647X_GetPosition(cpt_motor);
	}


	/* Wait for 2 seconds */
	sleep(2);

	//----- Go to position 6400
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d Go to position 6400\n", cpt_motor);

		/* Request device 0 to go to position 6400 */
		L647X_CmdGoTo(cpt_motor,6400);

		/* Wait for the motor of device 0 ends moving */
		L647X_WaitWhileActive(cpt_motor);

		/* Get current position of device 0*/
		pos = L647X_GetPosition(cpt_motor);
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- Go Mark which was set previously after go to -6400
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d Go to Mark which was set previously after go to -6400\n", cpt_motor);

		/* Request device to go to Mark position */
		L647X_CmdGoMark(cpt_motor);

		/* Wait for the motor of device ends moving */
		L647X_WaitWhileActive(cpt_motor);

		/* Get current position of device */
		pos = L647X_GetPosition(cpt_motor);
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- Run the motor BACKWARD
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d : Run the motor BACKWARD\n", cpt_motor);

		/* Request device to run BACKWARD at 400 step/s*/
		L647X_CmdRun( cpt_motor, BACKWARD, Speed_Steps_to_Par(300));

		sleep(1);

		/* Wait for device reaches the targeted speed */
		do
		{
			readData = L647X_CmdGetParam( cpt_motor, L647X_SPEED);
		} while (readData != Speed_Steps_to_Par(300));
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- Soft stopped required while running
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d : Soft stopped required while running\n", cpt_motor);

		/* Request soft stop of device */
		L647X_CmdSoftStop(cpt_motor);

		/* Wait for the motor of device ends moving */
		L647X_WaitWhileActive(cpt_motor);
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- Run stopped by hardstop
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d : Run stopped by hardstop\n", cpt_motor);

		/* Request device to run in FORWARD direction at 300 step/s*/
		printf("\t\t1- run forward at 300 step/s\n");
		L647X_CmdRun(cpt_motor,FORWARD,Speed_Steps_to_Par(300));

		sleep(2);
		/* Request device immediatly stop */
		printf("\t\t2- request device immediatly stop\n");
		L647X_CmdHardStop(cpt_motor);
		L647X_WaitWhileActive(cpt_motor);
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- GOTO stopped by softstop
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d : GOTO stopped by softstop\n", cpt_motor);

		/* Request device to go to position 20000  */
		L647X_CmdGoTo(cpt_motor,20000);

		sleep(2);

		/* Request device to perform a soft stop */
		L647X_CmdSoftStop(cpt_motor);
		L647X_WaitWhileActive(cpt_motor);
	}

	/* Wait for 2 seconds */
	sleep(2);

	//----- Read inexistent register to test MyFlagInterruptHandler
	printf("\n===>  Read inexistent register to test MyFlagInterruptHandler for device 0\n");

	printf("     Try to read an inexistent register, the flag interrupt should be raised ");
	printf("and the MyFlagInterruptHandler function called\n");
	L647X_CmdGetParam(0,(L647X_Registers_t)0x1F);


	sleep(2);

	//----- Put the bridges in high impedance
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d : Put the bridges in high impedance\n", cpt_motor);

		/* Request disabling of device 0 power bridges */
		L647X_CmdHardHiZ(cpt_motor);
	}

	sleep(2);

	//----- Step clock mode example
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d : Step clock mode example\n", cpt_motor);

		/* Enable Step Clock Mode */
		L647X_CmdStepClock(cpt_motor, FORWARD);

		/* Wait for 1 second */
		sleep(1);

		/* Enable the step clock at 333 Hz */
		L647X_StartStepClock(333);

		/* Let the motor runs for 5 second at 333 step/s */
		sleep(5);

		/* Stop the step clock */
		L647X_StopStepClock();
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- Change step mode to full step mode
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d : Change step mode to full step mode\n", cpt_motor);

		/* Select full step mode for device */
		L647X_SelectStepMode(cpt_motor,L647X_STEP_SEL_1);

		/* Set speed  to be consistent with full step mode */
		L647X_CmdSetParam(cpt_motor,L647X_MAX_SPEED, MaxSpd_Steps_to_Par(50));

		/* Request device to go position 200 */
		L647X_CmdGoTo(cpt_motor,200);

		/* Wait for the motor of device 0 ends moving */
		L647X_WaitWhileActive(cpt_motor);

		/* Get current position */
		pos = L647X_GetPosition(cpt_motor);
	}


	/* Wait for 2 seconds */
	sleep(2);


	//----- Restore initial microstepping mode
	for(cpt_motor=0; cpt_motor<nbr_of_motor; cpt_motor++)
	{
		printf("\n===> Device %d : Restore initial microstepping mode\n", cpt_motor);

		/* Reset device to its initial  microstepping mode */
		L647X_SelectStepMode(cpt_motor,L647X_CONF_PARAM_STEP_MODE_DEVICE_0);

		/* Update speed, acceleration, deceleration for initial microstepping mode*/
		L647X_CmdSetParam(cpt_motor,L647X_MAX_SPEED, MaxSpd_Steps_to_Par(L647X_CONF_PARAM_MAX_SPEED_DEVICE_0));

		/* Infinite loop */
		for(loop=0; loop<20; loop++)
		{
			/* Request device to go position -6400 */
			L647X_CmdGoTo(cpt_motor, -6400);

			/* Wait for the motor of device ends moving */
			L647X_WaitWhileActive(cpt_motor);

			/* Request device to go position 6400 */
			L647X_CmdGoTo(cpt_motor,6400);

			/* Wait for the motor of device ends moving */
			L647X_WaitWhileActive(cpt_motor);
		}
	}


	/* Wait for 2 seconds */
	sleep(2);


	/* Close */
	L647X_Close();

	printf("The END!!\n");
#endif



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


