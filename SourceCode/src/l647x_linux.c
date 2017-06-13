/**************************************************************************//**
  * @file    l647x_linux.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 7, 2017
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


#include "l647x_linux.h"


/* Private variables ---------------------------------------------------------*/

static uint8_t numberOfDevices;
static uint8_t spiTxBursts[L647X_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
static uint8_t spiRxBursts[L647X_CMD_ARG_MAX_NB_BYTES][ MAX_NUMBER_OF_DEVICES];

static uint8_t debug_flip;
spi_interface_struct_t spi_interface_struct;

static char errorMsg[200];


#define NB_PARAM_ENTRIES 37

#define ACC_IDX 0
#define DEC_IDX 1
#define MAX_SPEED_IDX 2
#define MIN_SPEED_IDX 3
#define FS_SPD_IDX 4
#define OCD_TH_IDX 5
#define ALARM_EN_IDX 6
#define SYNC_MODE_IDX 7
#define STEP_MODE_IDX 8
#define CLOCK_SETTING_IDX 9
#define SW_MODE_IDX 10
#define OC_SD_IDX 11
// L6470
#define LSPD_BIT_IDX 12
#define KVAL_HOLD_IDX 13
#define KVAL_RUN_IDX 14
#define KVAL_ACC_IDX 15
#define KVAL_DEC_IDX 16
#define INT_SPD_IDX 17
#define ST_SLP_IDX 18
#define FN_SLP_ACC_IDX 19
#define FN_SLP_DEC_IDX 20
#define K_THERM_IDX 21
#define STALL_TH_IDX 22
#define VS_COMP_IDX 23
#define PWM_DIV_IDX 24
#define PWM_MUL_IDX 25
// L6472
#define TVAL_ACC_IDX 26
#define TVAL_DEC_IDX 27
#define TVAL_RUN_IDX 28
#define TVAL_HOLD_IDX 29
#define TOFF_FAST_IDX 30
#define FAST_STEP_IDX 31
#define TON_MIN_IDX 32
#define TOFF_MIN_IDX 33
#define TQ_REG_IDX 34
#define TSW_IDX 35
#define PRED_EN_IDX 36

config_entry_t  config_entry[] =
{
		{ "Acc", ACC_IDX },
		{ "Dec", DEC_IDX },
		{ "MaxSpeed", MAX_SPEED_IDX },
		{ "MinSpeed", MIN_SPEED_IDX },
		{ "FS_Spd", FS_SPD_IDX },
		{ "OcdTh", OCD_TH_IDX },
		{ "AlarmEN", ALARM_EN_IDX },
		{ "SyncMode", SYNC_MODE_IDX },
		{ "StepMode", STEP_MODE_IDX },
		{ "ClockSetting", CLOCK_SETTING_IDX },
		{ "SwMode", SW_MODE_IDX },
		{ "OcSd", OC_SD_IDX },

		// L6470
		{ "LSPDbit", LSPD_BIT_IDX },
		{ "KvalHold", KVAL_HOLD_IDX },
		{ "KvalRun", KVAL_RUN_IDX },
		{ "KvalAcc", KVAL_ACC_IDX },
		{ "KvalDec", KVAL_DEC_IDX },
		{ "IntSpd", INT_SPD_IDX },
		{ "StSlp", ST_SLP_IDX },
		{ "FnSlpAcc", FN_SLP_ACC_IDX },
		{ "FnSlpDec", FN_SLP_DEC_IDX },
		{ "K_Therm", K_THERM_IDX },
		{ "StallTh", STALL_TH_IDX },
		{ "VsComp", VS_COMP_IDX },
		{ "PwmDiv", PWM_DIV_IDX },
		{ "PwmMul", PWM_MUL_IDX },

		// L6472
		{ "TvalAcc", TVAL_ACC_IDX },
		{ "TvalDec", TVAL_DEC_IDX },
		{ "TvalRun", TVAL_RUN_IDX },
		{ "TvalHold", TVAL_HOLD_IDX },
		{ "ToffFast", TOFF_FAST_IDX },
		{ "FastStep", FAST_STEP_IDX },
		{ "TonMin", TON_MIN_IDX },
		{ "ToffMin", TOFF_MIN_IDX },
		{ "TqReg", TQ_REG_IDX },
		{ "Tsw", TSW_IDX },
		{ "PredEn", PRED_EN_IDX }
};

#define NB_MAX_MOTOR 8
uint16_t motor_config[NB_MAX_MOTOR][NB_PARAM_ENTRIES];


/* Private function ----------------------------------------------------------*/

/******************************************************//**
 * @brief  Attaches a user callback to the busy Interrupt
 * The call back will be then called each time the busy
 * pin is set or reset
 * @param[in] callback Name of the callback to attach
 * to the Busy Interrupt
 * @retval None
 **********************************************************/
void L647X_AttachBusyInterrupt(void (*callback)(void))
{
  //busyInterruptCallback = (volatile void (*)())callback;
  gpio_set_callback( L647X_BUSY_PIN, callback);

}


/******************************************************//**
 * @brief  Attaches a user callback to the flag Interrupt
 * The call back will be then called each time the status
 * flag pin will be pulled down due to the occurrence of
 * a programmed alarms ( OCD, thermal pre-warning or
 * shutdown, UVLO, wrong command, non-performable command)
 * @param[in] callback Name of the callback to attach
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void L647X_AttachFlagInterrupt(void (*callback)(void))
{
  //flagInterruptCallback = (volatile void (*)())callback;
  gpio_set_callback( L647X_FLAG_PIN, callback);
}


/******************************************************//**
 * @brief Transmits/Receives several bytes to L647X over SPI
 * @param[in] pTxByte pTxBytePointer to TX bytes
 * @param[in] pRxByte Pointer to RX bytes
 * @param[in] nb_bytes Number of bytes to transmit.
 * @retval None
 *********************************************************/
int L647X_SpiTransmitReceive(	uint8_t *pByteToTransmit,
                         	 	 	uint8_t *pReceivedByte,
                         	 	 	uint16_t nb_bytes)
{
	struct spi_ioc_transfer spi[nb_bytes];
	int i = 0;
	int retVal = -1;

	memset(spi, 0x0, nb_bytes * sizeof(struct spi_ioc_transfer));

	for (i = 0 ; i < nb_bytes ; i++)
	{
		spi[i].tx_buf        = (unsigned long)(pByteToTransmit + i); // transmit from "data"
		spi[i].rx_buf        = (unsigned long)(pReceivedByte + i) ; // receive into "data"
		spi[i].len           = sizeof(*(pByteToTransmit + i)) ;
		spi[i].delay_usecs   = 0 ;
		spi[i].speed_hz      = spi_interface_struct.spi_speed;
		spi[i].bits_per_word = spi_interface_struct.spi_bitsPerWord;
		spi[i].cs_change = 0;
	}

	retVal = ioctl( spi_interface_struct.file_desc, SPI_IOC_MESSAGE(nb_bytes), &spi) ;

	if(retVal < 0)
	{
		perror("Error - Problem transmitting spi data..ioctl");
		return -1;
	}

#if DEBUG_LEVEL == 1
	// Debug part
	for(i=0; i<nb_bytes; i++)
	{
		if( !debug_flip)
			printf("//DEBUG// L647X_SpiTransmitReceive: (0) Tx = 0x%02x  ;  Rx = 0x%02x",
					(int)*(pByteToTransmit + i),
					(int)*(pReceivedByte + i));
		else
			printf("  ;  (%d) Tx = 0x%02x  ;  Rx = 0x%02x", debug_flip,
					(int)*(pByteToTransmit + i),
					(int)*(pReceivedByte + i));

		if( debug_flip == (numberOfDevices -1))
			printf("\n");

		debug_flip = (debug_flip+1)%numberOfDevices;
	}
	// End Debug part
#endif

	return 0;
}


/******************************************************//**
 * @brief Transmits/Receives several bytes to L647X
 * @param[in] pTxByte pTxBytePointer to TX bytes
 * @param[in] pRxByte Pointer to RX bytes
 * @retval None
 *********************************************************/
inline void L647X_WriteReadBytes(uint8_t *pByteToTransmit,
                         	 uint8_t *pReceivedByte)
{

	if (L647X_SpiTransmitReceive( pByteToTransmit, pReceivedByte,
								  (uint16_t)numberOfDevices) != 0)
	{
		exit(-1);
	}
}


/******************************************************//**
 * @brief  Sends a command to a given device Id via the SPI
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Command to send (all L647X commmands
 * except L647X_SET_PARAM, L647X_GET_PARAM,
 * L647X_GET_STATUS)
 * @param[in] value arguments to send on 32 bits
 * @retval None
 **********************************************************/
void L647X_SendCommand(uint8_t deviceId, uint8_t param, uint32_t value)
{
  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t maxArgumentNbBytes = 0;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;

    for (loop = 0; loop < numberOfDevices; loop++)
    {
        spiTxBursts[0][loop] = L647X_NOP;
        spiTxBursts[1][loop] = L647X_NOP;
        spiTxBursts[2][loop] = L647X_NOP;
        spiTxBursts[3][loop] = L647X_NOP;
    }
    switch (param & DAISY_CHAIN_COMMAND_MASK)
    {
      case L647X_RUN:
      case L647X_MOVE:
      case L647X_GO_TO:
      case L647X_GO_TO_DIR:
      case L647X_GO_UNTIL:
      case L647X_GO_UNTIL_ACT_CPY:
               spiTxBursts[0][spiIndex] = param;
               spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
               spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
               spiTxBursts[3][spiIndex] = (uint8_t)(value);
               maxArgumentNbBytes = 3;
               break;
      default:
               spiTxBursts[0][spiIndex] = L647X_NOP;
               spiTxBursts[1][spiIndex] = L647X_NOP;
               spiTxBursts[2][spiIndex] = L647X_NOP;
               spiTxBursts[3][spiIndex] = param;
    }


    for (loop = L647X_CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes;
         loop < L647X_CMD_ARG_MAX_NB_BYTES;
         loop++)
    {

        L647X_WriteReadBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);

    }

  }
}


/******************************************************//**
 * @brief  Sets current position to be the Mark position
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void L647X_SetMark(uint8_t deviceId)
{
  uint32_t mark = L647X_CmdGetParam(deviceId, L647X_ABS_POS);
  L647X_CmdSetParam(deviceId, L647X_MARK, mark);
}


uint8_t find_param_entry( char *paramName)
{
	uint8_t cpt=0;
	while( 	(cpt<NB_PARAM_ENTRIES) &&
			strcmp( config_entry[cpt].name, paramName) )
		cpt++;

	if( cpt == NB_PARAM_ENTRIES)
		return 0xFF;

	return cpt;
}

uint8_t parse_line( char *line_in, int *deviceId, char *paramName, float *value)
{
	uint8_t cptChar = 0;
	uint8_t cptTmp = 0;
	uint8_t board,motor;
	const char delim[2] = ":";
	char *token;

	// Suppress space or comments
	while( line_in[cptChar])
	{
		if( line_in[cptChar] == ' ')
		{
			cptTmp = cptChar;
			while( line_in[cptTmp] == ' ')  cptTmp++;
			strcpy( &line_in[cptChar], &line_in[cptTmp]);
		}
		if( line_in[cptChar] == '/')
		{
			line_in[cptChar] = 0x00;
			break;
		}
		cptChar++;
	}

	if( strlen(line_in)<5)
		return 1;

	/* get the first token */
	token = strtok(line_in, delim);
	/* walk through other tokens */
	if( token != NULL )
	{
		// Board number
		board = atoi(token);

		// Motor number
		token = strtok(NULL, delim);
		if( token != NULL )
			motor = atoi(token);
		else
			return 1;

		// Parameter name
		token = strtok(NULL, delim);
		if( token != NULL )
			strcpy(paramName, token);
		else
			return 1;

		// Float value
		token = strtok(NULL, delim);
		if( token != NULL )
			*value = atof(token);
		else
			return 1;

		*deviceId = (board * 2) + motor;
	}
	else
		return 1;

	return 0;  // OK
}


/******************************************************//**
 * @brief  Sets the registers of the L647X to their predefined values
 * from L647X_target_config.h
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L647X_LoadMotorConfigValuesFromFile(FILE *ptr)
{
	uint8_t nb_field=0;
	int device_id;
	char line_in[255];
	char param_name[20];
	uint8_t param_entry;
	float value;

	if( !ptr)
		return;

	while (fgets( line_in, 250, ptr))
	{
		if( parse_line( line_in, &device_id, param_name, &value))
			continue;

		param_entry = find_param_entry( param_name);
		if( param_entry == 0xFF)
			continue;

		nb_field++;

		switch( param_entry)
		{
		case ACC_IDX:
		case DEC_IDX:
			motor_config[device_id][param_entry] = AccDec_Steps_to_Par(value);
			break;

		case MAX_SPEED_IDX:
		case MIN_SPEED_IDX:
			motor_config[device_id][param_entry] = MaxSpd_Steps_to_Par(value);
			break;

		case FS_SPD_IDX:
			motor_config[device_id][param_entry] = FSSpd_Steps_to_Par(value);
			break;

		case OCD_TH_IDX:
		case ALARM_EN_IDX:
		case LSPD_BIT_IDX:
		case STEP_MODE_IDX:
		case CLOCK_SETTING_IDX:
		case SW_MODE_IDX:
		case OC_SD_IDX:
		case SYNC_MODE_IDX:
		case VS_COMP_IDX:
		case PWM_DIV_IDX:
		case PWM_MUL_IDX:
		case TOFF_FAST_IDX:
		case FAST_STEP_IDX:
		case TQ_REG_IDX:
		case TSW_IDX:
		case PRED_EN_IDX:
			motor_config[device_id][param_entry] = (uint16_t)value;
			break;

// L6470
		case KVAL_HOLD_IDX:
		case KVAL_RUN_IDX:
		case KVAL_ACC_IDX:
		case KVAL_DEC_IDX:
			motor_config[device_id][param_entry] = Kval_Perc_to_Par(value);
			break;

		case INT_SPD_IDX:
			motor_config[device_id][param_entry] = IntSpd_Steps_to_Par(value);
			break;

		case ST_SLP_IDX:
		case FN_SLP_ACC_IDX:
		case FN_SLP_DEC_IDX:
			motor_config[device_id][param_entry] = BEMF_Slope_Perc_to_Par(value);
			break;

		case K_THERM_IDX:
			motor_config[device_id][param_entry] = KTherm_to_Par(value);
			break;

		case STALL_TH_IDX:
			motor_config[device_id][param_entry] = StallTh_to_Par(value);
			break;

// L6472
		case TVAL_ACC_IDX:
		case TVAL_DEC_IDX:
		case TVAL_RUN_IDX:
		case TVAL_HOLD_IDX:
			motor_config[device_id][param_entry] = Tval_Current_to_Par(value);
			break;

		case TON_MIN_IDX:
		case TOFF_MIN_IDX:
			break;
			motor_config[device_id][param_entry] = Tmin_Time_to_Par(value);

		default:
			break;
		}
	}

#if 0
	{
		// Debug
		uint8_t cpt_device, cpt_param;
		for( cpt_device=0; cpt_device<8; cpt_device++)
		{
			printf("\n\n****** Motor %d ******\n", cpt_device);
			for( cpt_param=0; cpt_param<NB_PARAM_ENTRIES ; cpt_param++)
				printf("device[%d].%s \t=\t%d\n", cpt_device, config_entry[cpt_param].name,
						motor_config[cpt_device][cpt_param]);
		}
	}
#endif
}


/******************************************************//**
 * @brief  Sets the registers of the L647X to their predefined values
 * from L647X_target_config.h
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L647X_SetMotorConfigToPredefinedValues( void)
{
	uint8_t motorCpt;

	for( motorCpt=0; motorCpt<8; motorCpt++)
	{
		motor_config[motorCpt][ACC_IDX] = 			AccDec_Steps_to_Par(L647X_CONF_PARAM_ACC);
		motor_config[motorCpt][DEC_IDX] = 			AccDec_Steps_to_Par(L647X_CONF_PARAM_DEC);
		motor_config[motorCpt][MAX_SPEED_IDX] = 	MaxSpd_Steps_to_Par(L647X_CONF_PARAM_MAX_SPEED);
		motor_config[motorCpt][FS_SPD_IDX] =  		FSSpd_Steps_to_Par(L647X_CONF_PARAM_FS_SPD);
		motor_config[motorCpt][OCD_TH_IDX] =  		(uint8_t)L647X_CONF_PARAM_OCD_TH;
		motor_config[motorCpt][ALARM_EN_IDX] = 		L647X_CONF_PARAM_ALARM_EN;

#ifdef L6470
		motor_config[motorCpt][LSPD_BIT_IDX] =  	L647X_CONF_PARAM_LSPD_BIT;
		motor_config[motorCpt][MIN_SPEED_IDX] = 	MinSpd_Steps_to_Par(L647X_CONF_PARAM_MIN_SPEED);
		motor_config[motorCpt][KVAL_HOLD_IDX] = 	Kval_Perc_to_Par(L647X_CONF_PARAM_KVAL_HOLD);
		motor_config[motorCpt][KVAL_RUN_IDX] =  	Kval_Perc_to_Par(L647X_CONF_PARAM_KVAL_RUN);
		motor_config[motorCpt][KVAL_ACC_IDX] =  	Kval_Perc_to_Par(L647X_CONF_PARAM_KVAL_ACC);
		motor_config[motorCpt][KVAL_DEC_IDX] =  	Kval_Perc_to_Par(L647X_CONF_PARAM_KVAL_DEC);
		motor_config[motorCpt][INT_SPD_IDX] =   	IntSpd_Steps_to_Par(L647X_CONF_PARAM_INT_SPD);
		motor_config[motorCpt][ST_SLP_IDX] =    	BEMF_Slope_Perc_to_Par(L647X_CONF_PARAM_ST_SLP);
		motor_config[motorCpt][FN_SLP_ACC_IDX] =	BEMF_Slope_Perc_to_Par(L647X_CONF_PARAM_FN_SLP_ACC);
		motor_config[motorCpt][FN_SLP_DEC_IDX] =	BEMF_Slope_Perc_to_Par(L647X_CONF_PARAM_FN_SLP_DEC);
		motor_config[motorCpt][K_THERM_IDX] = 		KTherm_to_Par(L647X_CONF_PARAM_K_THERM);
		motor_config[motorCpt][STALL_TH_IDX] = 		StallTh_to_Par(L647X_CONF_PARAM_STALL_TH);
		motor_config[motorCpt][SYNC_MODE_IDX] =		(uint8_t)L647X_CONF_PARAM_SYNC_MODE;
		motor_config[motorCpt][STEP_MODE_IDX] =		(uint8_t)L647X_CONF_PARAM_STEP_MODE;
		motor_config[motorCpt][CLOCK_SETTING_IDX] = (uint16_t)L647X_CONF_PARAM_CLOCK_SETTING;
		motor_config[motorCpt][SW_MODE_IDX] = 		(uint16_t)L647X_CONF_PARAM_SW_MODE;
		motor_config[motorCpt][VS_COMP_IDX] = 		(uint16_t)L647X_CONF_PARAM_VS_COMP;
		motor_config[motorCpt][OC_SD_IDX] = 		(uint16_t)L647X_CONF_PARAM_OC_SD;
		motor_config[motorCpt][PWM_DIV_IDX] = 		(uint16_t)L647X_CONF_PARAM_PWM_DIV;
		motor_config[motorCpt][PWM_MUL_IDX] = 		(uint16_t)L647X_CONF_PARAM_PWM_MUL;
#endif /* #ifdef L6470 */
#ifdef L6472
		motor_config[motorCpt][MIN_SPEED_IDX] = 	MinSpd_Steps_to_Par(L647X_CONF_PARAM_MIN_SPEED);
		motor_config[motorCpt][TVAL_HOLD_IDX] =  	Tval_Current_to_Par(L647X_CONF_PARAM_TVAL_HOLD);
		motor_config[motorCpt][TVAL_RUN_IDX] =   	Tval_Current_to_Par(L647X_CONF_PARAM_TVAL_RUN);
		motor_config[motorCpt][TVAL_ACC_IDX] =   	Tval_Current_to_Par(L647X_CONF_PARAM_TVAL_ACC);
		motor_config[motorCpt][TVAL_DEC_IDX] =   	Tval_Current_to_Par(L647X_CONF_PARAM_TVAL_DEC);
		motor_config[motorCpt][TOFF_FAST_IDX] =     (uint8_t)L647X_CONF_PARAM_TOFF_FAST;
		motor_config[motorCpt][FAST_STEP_IDX] =     (uint8_t)L647X_CONF_PARAM_FAST_STEP;
		motor_config[motorCpt][TON_MIN_IDX] =    	Tmin_Time_to_Par(L647X_CONF_PARAM_TON_MIN);
		motor_config[motorCpt][TOFF_MIN_IDX] =   	Tmin_Time_to_Par(L647X_CONF_PARAM_TOFF_MIN);
		motor_config[motorCpt][SYNC_MODE_IDX] =  	(uint8_t)L647X_CONF_PARAM_SYNC_MODE;
		motor_config[motorCpt][STEP_MODE_IDX] =  	(uint8_t)L647X_CONF_PARAM_STEP_MODE;
		motor_config[motorCpt][CLOCK_SETTING_IDX] = (uint16_t)L647X_CONF_PARAM_CLOCK_SETTING;
		motor_config[motorCpt][SW_MODE_IDX] =     	(uint16_t)L647X_CONF_PARAM_SW_MODE;
		motor_config[motorCpt][TQ_REG_IDX] =		(uint16_t)L647X_CONF_PARAM_TQ_REG;
		motor_config[motorCpt][OC_SD_IDX] =     	(uint16_t)L647X_CONF_PARAM_OC_SD;
		motor_config[motorCpt][TSW_IDX] =     		(uint16_t)L647X_CONF_PARAM_TSW;
		motor_config[motorCpt][PRED_EN_IDX] =     	(uint16_t)L647X_CONF_PARAM_PRED_EN;
#endif /* #ifdef L6472 */
	}
}


/******************************************************//**
 * @brief  Sets the registers of the L647X to their predefined values
 * from L647X_target_config.h
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L647X_SetRegisterValues( uint8_t deviceId)
{
  L647X_CmdSetParam(deviceId, L647X_ABS_POS, 0);
  L647X_CmdSetParam(deviceId, L647X_EL_POS, 0);
  L647X_CmdSetParam(deviceId, L647X_MARK, 0);


  L647X_CmdSetParam(deviceId, L647X_ACC,  motor_config[deviceId][ACC_IDX]);
  L647X_CmdSetParam(deviceId, L647X_DEC,  motor_config[deviceId][DEC_IDX]);
  L647X_CmdSetParam(deviceId, L647X_MAX_SPEED, motor_config[deviceId][MAX_SPEED_IDX]);
  L647X_CmdSetParam(deviceId, L647X_FS_SPD, motor_config[deviceId][FS_SPD_IDX]);
  L647X_CmdSetParam(deviceId, L647X_OCD_TH,    (uint8_t)motor_config[deviceId][OCD_TH_IDX]);
  L647X_CmdSetParam(deviceId, L647X_ALARM_EN,  motor_config[deviceId][ALARM_EN_IDX]);

#ifdef L6470
  L647X_CmdSetParam(deviceId, L647X_MIN_SPEED, 	motor_config[deviceId][LSPD_BIT_IDX] |
												motor_config[deviceId][MIN_SPEED_IDX]);
  L647X_CmdSetParam(deviceId, L647X_KVAL_HOLD,  motor_config[deviceId][KVAL_HOLD_IDX]);
  L647X_CmdSetParam(deviceId, L647X_KVAL_RUN,   motor_config[deviceId][KVAL_RUN_IDX]);
  L647X_CmdSetParam(deviceId, L647X_KVAL_ACC,   motor_config[deviceId][KVAL_ACC_IDX]);
  L647X_CmdSetParam(deviceId, L647X_KVAL_DEC,   motor_config[deviceId][KVAL_DEC_IDX]);
  L647X_CmdSetParam(deviceId, L647X_INT_SPD,   motor_config[deviceId][INT_SPD_IDX]);
  L647X_CmdSetParam(deviceId, L647X_ST_SLP,     motor_config[deviceId][ST_SLP_IDX]);
  L647X_CmdSetParam(deviceId, L647X_FN_SLP_ACC, motor_config[deviceId][FN_SLP_ACC_IDX]);
  L647X_CmdSetParam(deviceId, L647X_FN_SLP_DEC, motor_config[deviceId][FN_SLP_DEC_IDX]);
  L647X_CmdSetParam(deviceId, L647X_K_THERM,    motor_config[deviceId][K_THERM_IDX]);
  L647X_CmdSetParam(deviceId, L647X_STALL_TH,  motor_config[deviceId][STALL_TH_IDX]);
  L647X_CmdSetParam(deviceId, L647X_STEP_MODE, (uint8_t)motor_config[deviceId][SYNC_MODE_IDX] |
		  	  	  	  	  	  	  	  	  	    (uint8_t)motor_config[deviceId][STEP_MODE_IDX]);
  L647X_CmdSetParam(deviceId, L647X_CONFIG,    (uint16_t)motor_config[deviceId][CLOCK_SETTING_IDX] |
		  	  	  	  	  	  	  	  	  	   (uint16_t)motor_config[deviceId][SW_MODE_IDX]       |
                                                   (uint16_t)motor_config[deviceId][VS_COMP_IDX]       |
                                                   (uint16_t)motor_config[deviceId][OC_SD_IDX]         |
                                                   (uint16_t)motor_config[deviceId][PWM_DIV_IDX]       |
                                                   (uint16_t)motor_config[deviceId][PWM_MUL_IDX]);
#endif /* #ifdef L6470 */
#ifdef L6472
  L647X_CmdSetParam(deviceId, L647X_MIN_SPEED, motor_config[deviceId][MIN_SPEED_IDX]);
  L647X_CmdSetParam(deviceId, L647X_TVAL_HOLD, motor_config[deviceId][TVAL_HOLD_IDX]);
  L647X_CmdSetParam(deviceId, L647X_TVAL_RUN,  motor_config[deviceId][TVAL_RUN_IDX]);
  L647X_CmdSetParam(deviceId, L647X_TVAL_ACC,  motor_config[deviceId][TVAL_ACC_IDX]);
  L647X_CmdSetParam(deviceId, L647X_TVAL_DEC,  motor_config[deviceId][TVAL_DEC_IDX]);
  L647X_CmdSetParam(deviceId, L647X_T_FAST,    (uint8_t)motor_config[deviceId][TOFF_FAST_IDX] |
		  (uint8_t)motor_config[deviceId][FAST_STEP_IDX]);
  L647X_CmdSetParam(deviceId, L647X_TON_MIN,   motor_config[deviceId][TON_MIN_IDX]);
  L647X_CmdSetParam(deviceId, L647X_TOFF_MIN,  motor_config[deviceId][TOFF_MIN_IDX]);
  L647X_CmdSetParam(deviceId, L647X_STEP_MODE, (uint8_t)motor_config[deviceId][SYNC_MODE_IDX] 	|
                                                            	(uint8_t)(1 << 3)						|
																(uint8_t)motor_config[deviceId][STEP_MODE_IDX]);
  L647X_CmdSetParam(deviceId, L647X_CONFIG,    (uint16_t)motor_config[deviceId][CLOCK_SETTING_IDX] 		|
		  (uint16_t)motor_config[deviceId][SW_MODE_IDX]   |
		  (uint16_t)motor_config[deviceId][TQ_REG_IDX]    |
		  (uint16_t)motor_config[deviceId][OC_SD_IDX]     |
		  (uint16_t)motor_config[deviceId][TSW_IDX]       |
		  (uint16_t)motor_config[deviceId][PRED_EN_IDX]);
#endif /* #ifdef L6472 */

}


#if 1
/******************************************************//**
 * @brief  Sets the registers of the L647X to their predefined values
 * from L647X_target_config.h
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L647X_SetRegisterToPredefinedValues(uint8_t deviceId)
{
  L647X_CmdSetParam(deviceId, L647X_ABS_POS, 0);
  L647X_CmdSetParam(deviceId, L647X_EL_POS, 0);
  L647X_CmdSetParam(deviceId, L647X_MARK, 0);



      L647X_CmdSetParam(deviceId, L647X_ACC,       AccDec_Steps_to_Par(L647X_CONF_PARAM_ACC));
      L647X_CmdSetParam(deviceId, L647X_DEC,       AccDec_Steps_to_Par(L647X_CONF_PARAM_DEC));
      L647X_CmdSetParam(deviceId, L647X_MAX_SPEED, MaxSpd_Steps_to_Par(L647X_CONF_PARAM_MAX_SPEED));
      L647X_CmdSetParam(deviceId, L647X_FS_SPD,    FSSpd_Steps_to_Par(L647X_CONF_PARAM_FS_SPD));
      L647X_CmdSetParam(deviceId, L647X_OCD_TH,    (uint8_t)L647X_CONF_PARAM_OCD_TH);
      L647X_CmdSetParam(deviceId, L647X_ALARM_EN,  L647X_CONF_PARAM_ALARM_EN);

#ifdef L6470
      L647X_CmdSetParam(deviceId, L647X_MIN_SPEED, L647X_CONF_PARAM_LSPD_BIT|
                                                   MinSpd_Steps_to_Par(L647X_CONF_PARAM_MIN_SPEED));
      L647X_CmdSetParam(deviceId, L647X_KVAL_HOLD,  Kval_Perc_to_Par(L647X_CONF_PARAM_KVAL_HOLD));
      L647X_CmdSetParam(deviceId, L647X_KVAL_RUN,   Kval_Perc_to_Par(L647X_CONF_PARAM_KVAL_RUN));
      L647X_CmdSetParam(deviceId, L647X_KVAL_ACC,   Kval_Perc_to_Par(L647X_CONF_PARAM_KVAL_ACC));
      L647X_CmdSetParam(deviceId, L647X_KVAL_DEC,   Kval_Perc_to_Par(L647X_CONF_PARAM_KVAL_DEC));
      L647X_CmdSetParam(deviceId, L647X_INT_SPD,   IntSpd_Steps_to_Par(L647X_CONF_PARAM_INT_SPD));
      L647X_CmdSetParam(deviceId, L647X_ST_SLP,     BEMF_Slope_Perc_to_Par(L647X_CONF_PARAM_ST_SLP));
      L647X_CmdSetParam(deviceId, L647X_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(L647X_CONF_PARAM_FN_SLP_ACC));
      L647X_CmdSetParam(deviceId, L647X_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(L647X_CONF_PARAM_FN_SLP_DEC));
      L647X_CmdSetParam(deviceId, L647X_K_THERM,    KTherm_to_Par(L647X_CONF_PARAM_K_THERM));
      L647X_CmdSetParam(deviceId, L647X_STALL_TH,  StallTh_to_Par(L647X_CONF_PARAM_STALL_TH));
      L647X_CmdSetParam(deviceId, L647X_STEP_MODE, (uint8_t)L647X_CONF_PARAM_SYNC_MODE |
                                                   (uint8_t)L647X_CONF_PARAM_STEP_MODE);
      L647X_CmdSetParam(deviceId, L647X_CONFIG,    (uint16_t)L647X_CONF_PARAM_CLOCK_SETTING |
                                                   (uint16_t)L647X_CONF_PARAM_SW_MODE       |
                                                   (uint16_t)L647X_CONF_PARAM_VS_COMP       |
                                                   (uint16_t)L647X_CONF_PARAM_OC_SD         |
                                                   (uint16_t)L647X_CONF_PARAM_PWM_DIV       |
                                                   (uint16_t)L647X_CONF_PARAM_PWM_MUL);
#endif /* #ifdef L6470 */
#ifdef L6472
      L647X_CmdSetParam(deviceId, L647X_MIN_SPEED, MinSpd_Steps_to_Par(L647X_CONF_PARAM_MIN_SPEED));
      L647X_CmdSetParam(deviceId, L647X_TVAL_HOLD, Tval_Current_to_Par(L647X_CONF_PARAM_TVAL_HOLD));
      L647X_CmdSetParam(deviceId, L647X_TVAL_RUN,  Tval_Current_to_Par(L647X_CONF_PARAM_TVAL_RUN));
      L647X_CmdSetParam(deviceId, L647X_TVAL_ACC,  Tval_Current_to_Par(L647X_CONF_PARAM_TVAL_ACC));
      L647X_CmdSetParam(deviceId, L647X_TVAL_DEC,  Tval_Current_to_Par(L647X_CONF_PARAM_TVAL_DEC));
      L647X_CmdSetParam(deviceId, L647X_T_FAST,    (uint8_t)L647X_CONF_PARAM_TOFF_FAST |
                                                            (uint8_t)L647X_CONF_PARAM_FAST_STEP);
      L647X_CmdSetParam(deviceId, L647X_TON_MIN,   Tmin_Time_to_Par(L647X_CONF_PARAM_TON_MIN));
      L647X_CmdSetParam(deviceId, L647X_TOFF_MIN,  Tmin_Time_to_Par(L647X_CONF_PARAM_TOFF_MIN));
      L647X_CmdSetParam(deviceId, L647X_STEP_MODE, (uint8_t)L647X_CONF_PARAM_SYNC_MODE |
                                                            (uint8_t)(1 << 3)|
                                                            (uint8_t)L647X_CONF_PARAM_STEP_MODE);
      L647X_CmdSetParam(deviceId, L647X_CONFIG,    (uint16_t)L647X_CONF_PARAM_CLOCK_SETTING |
                                                            (uint16_t)L647X_CONF_PARAM_SW_MODE       |
                                                            (uint16_t)L647X_CONF_PARAM_TQ_REG        |
                                                            (uint16_t)L647X_CONF_PARAM_OC_SD         |
                                                            (uint16_t)L647X_CONF_PARAM_TSW           |
                                                            (uint16_t)L647X_CONF_PARAM_PRED_EN);
#endif /* #ifdef L6472 */

}
#endif


/**********************************************************
 * @brief  Initialisation of SPI peripherals
 * @param None
 * @retval None
 **********************************************************/
int L647X_SPIOpenPort( char spi_interface_name[] )
{
    int status_value;

    //----- Available SPI MODE -----
    //SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
    //SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
    //SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
    //SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
    spi_interface_struct.spi_mode = SPI_MODE_3;

    spi_interface_struct.spi_speed = 100000;		//1000000 = 1MHz (1uS per bit)

    spi_interface_struct.spi_bitsPerWord = 8;


    spi_interface_struct.file_desc = open( spi_interface_name, O_RDWR);
    if (spi_interface_struct.file_desc < 0) {
    	sprintf( errorMsg, "Error while opening '%s'", spi_interface_name);
        perror(errorMsg);
        exit(EXIT_FAILURE);
    }

    status_value = ioctl(spi_interface_struct.file_desc, SPI_IOC_WR_MODE, &spi_interface_struct.spi_mode);
    if(status_value < 0)
    {
        perror("Could not set SPIMode (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spi_interface_struct.file_desc, SPI_IOC_RD_MODE, &spi_interface_struct.spi_mode);
    if(status_value < 0)
    {
      perror("Could not set SPIMode (RD)...ioctl fail");
      exit(1);
    }

    status_value = ioctl(spi_interface_struct.file_desc, SPI_IOC_WR_BITS_PER_WORD, &spi_interface_struct.spi_bitsPerWord);
    if(status_value < 0)
    {
      perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
      exit(1);
    }

    status_value = ioctl(spi_interface_struct.file_desc, SPI_IOC_RD_BITS_PER_WORD, &spi_interface_struct.spi_bitsPerWord);
    if(status_value < 0)
    {
      perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
      exit(1);
    }

    status_value = ioctl(spi_interface_struct.file_desc, SPI_IOC_WR_MAX_SPEED_HZ, &spi_interface_struct.spi_speed);
    if(status_value < 0)
    {
      perror("Could not set SPI speed (WR)...ioctl fail");
      exit(1);
    }

    status_value = ioctl(spi_interface_struct.file_desc, SPI_IOC_RD_MAX_SPEED_HZ, &spi_interface_struct.spi_speed);
    if(status_value < 0)
    {
      perror("Could not set SPI speed (RD)...ioctl fail");
      exit(1);
    }

 //   close(fd_spi);
    return EXIT_SUCCESS;
}


/**********************************************************
 * @brief  Initialisation of GPIO peripherals
 * @param None
 * @retval None
 **********************************************************/
int L647X_GPIOinit()
{
	gpio_init();
	gpio_open( L647X_BUSY_PIN, in, low, falling);
	gpio_open( L647X_FLAG_PIN, in, low, falling);
	gpio_open( L647X_STBY_RESET_PIN, out, high, none);

	sleep(2);

	return 0;
}


/******************************************************//**
 * @brief  Releases the L647X reset (pin set to High) of all devices
 * @param  None
 * @retval None
 **********************************************************/
void L647X_ReleaseReset(void)
{
	//HAL_GPIO_WritePin(L647X_STBY_RESET_PORT, L647X_STBY_RESET_PIN, GPIO_PIN_SET);
	gpio_set_value( L647X_STBY_RESET_PIN, L647X_HIGH);
}


/******************************************************//**
 * @brief  Resets the L647X (reset pin set to low) of all devices
 * @param  None
 * @retval None
 **********************************************************/
void L647X_Reset(void)
{
	//HAL_GPIO_WritePin(L647X_STBY_RESET_PORT, L647X_STBY_RESET_PIN, GPIO_PIN_RESET);
	gpio_set_value( L647X_STBY_RESET_PIN, L647X_LOW);
}



/******************************************************//**
 * @brief Starts the L647X library
 * @param[in] nbDevices Number of L647X devices to use (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval None
 **********************************************************/
void L647X_Begin(uint8_t nbDevices)
{
  uint8_t loop;

  if( nbDevices > MAX_NUMBER_OF_DEVICES)
  {
      perror("Too much devices!!!!!");
	  return;
  }

  debug_flip = 0;

  numberOfDevices = nbDevices;

  // Interfaces initialization
  L647X_SPIOpenPort( L647X_SPI_FILE_PATH);

  // L647X GPIO initialization
  L647X_GPIOinit();

  /* Reset L647X */
  L647X_Reset();

  /* Let a delay after reset */
  usleep(500000);

  /* Standby-reset deactivation */
  L647X_ReleaseReset();

  /* Let a delay after reset deactivation*/
  usleep(500000);

  // For all devices
  for (loop = 0; loop < numberOfDevices; loop++)
  {
	  // Set all registers to their predefined values from L647X_target_config.h
	  //L647X_SetRegisterToPredefinedValues(loop);
	  L647X_SetRegisterValues(loop);
	  // Put the L647X in HiZ state
	  L647X_CmdHardHiZ(loop);
  }

  L647X_FetchAndClearAllStatus();

}


/******************************************************//**
 * @brief Starts the L647X library
 * @param[in] nbDevices Number of L647X devices to use (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval None
 **********************************************************/
void L647X_Close(void)
{
  // GPIO close
	gpio_end();
}


/******************************************************//**
 * @brief Issues L647X Get Param command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param L647X register address
 * @retval Register value - 1 to 3 bytes (depends on register)
 *********************************************************/
uint32_t L647X_CmdGetParam(uint8_t deviceId,
                           L647X_Registers_t param)
{
  uint32_t spiRxData = 0;

  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t maxArgumentNbBytes = 0;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;

    for (loop = 0; loop < numberOfDevices; loop++)
    {
      spiTxBursts[0][loop] = L647X_NOP;
      spiTxBursts[1][loop] = L647X_NOP;
      spiTxBursts[2][loop] = L647X_NOP;
      spiTxBursts[3][loop] = L647X_NOP;
      spiRxBursts[0][loop] = 0;
      spiRxBursts[1][loop] = 0;
      spiRxBursts[2][loop] = 0;
      spiRxBursts[3][loop] = 0;
    }
    switch (param)
    {
      case L647X_ABS_POS:
      case L647X_MARK:
      case L647X_SPEED:
        spiTxBursts[0][spiIndex] = ((uint8_t)L647X_GET_PARAM )| (param);
        maxArgumentNbBytes = 3;
        break;
      case L647X_EL_POS:
      case L647X_ACC:
      case L647X_DEC:
      case L647X_MAX_SPEED:
      case L647X_MIN_SPEED:
      case L647X_FS_SPD:
#ifdef L6470
      case L647X_INT_SPD:
#endif
      case L647X_CONFIG:
      case L647X_STATUS:
        spiTxBursts[1][spiIndex] = ((uint8_t)L647X_GET_PARAM )| (param);
        maxArgumentNbBytes = 2;
        break;
      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)L647X_GET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }

    for (loop = L647X_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
         loop < L647X_CMD_ARG_MAX_NB_BYTES;
         loop++)
    {
        L647X_WriteReadBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
    }

    spiRxData = ((uint32_t)spiRxBursts[1][spiIndex] << 16)|
    			 (uint32_t)(spiRxBursts[2][spiIndex] << 8) |
				 (uint32_t)(spiRxBursts[3][spiIndex]);
  }

  return (spiRxData);
}

/******************************************************//**
 * @brief Issues L647X Get Status command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval Status Register content
 *********************************************************/
uint16_t L647X_CmdGetStatus(uint8_t deviceId)
{
  uint16_t status = 0;

  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;

      for (loop = 0; loop < numberOfDevices; loop++)
      {
         spiTxBursts[0][loop] = L647X_NOP;
         spiTxBursts[1][loop] = L647X_NOP;
         spiTxBursts[2][loop] = L647X_NOP;
         spiTxBursts[3][loop] = L647X_NOP;
         spiRxBursts[0][loop] = 0;
         spiRxBursts[1][loop] = 0;
         spiRxBursts[2][loop] = 0;
         spiRxBursts[3][loop] = 0;
      }
      spiTxBursts[0][spiIndex] = L647X_GET_STATUS;

    for (loop = 0; loop < L647X_CMD_ARG_NB_BYTES_GET_STATUS + L647X_RSP_NB_BYTES_GET_STATUS; loop++)
    {
        L647X_WriteReadBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
    }

    status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  }

  return (status);
}

/******************************************************//**
 * @brief Issues L647X Go Home command (Shorted path to zero position)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdGoHome(uint8_t deviceId)
{
   L647X_SendCommand(deviceId, L647X_GO_HOME, 0);
}

/******************************************************//**
 * @brief Issues L647X Go Mark command
 * @param[in] deviceId(from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdGoMark(uint8_t deviceId)
{
  L647X_SendCommand(deviceId, L647X_GO_MARK, 0);
}

/******************************************************//**
 * @brief Issues L647X Go To command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] abs_pos absolute position where requested to move
 * @retval None
 *********************************************************/
void L647X_CmdGoTo(uint8_t deviceId, int32_t abs_pos)
{
  L647X_SendCommand(deviceId, (uint8_t)L647X_GO_TO, abs_pos);
}

/******************************************************//**
 * @brief Issues L647X Go To Dir command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction movement direction
 * @param[in] abs_pos absolute position where requested to move
 * @retval None
 *********************************************************/
void L647X_CmdGoToDir(uint8_t deviceId,
                      L647X_Direction_t direction,
                      int32_t abs_pos)
{
  L647X_SendCommand(deviceId,
                          (uint8_t)L647X_GO_TO_DIR |
                          (uint8_t)direction, abs_pos);
}

/******************************************************//**
 * @brief Issues L647X Go Until command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action ACTION_RESET or ACTION_COPY
 * @param[in] direction movement direction
 * @param[in] speed
 * @retval None
 *********************************************************/
void L647X_CmdGoUntil(uint8_t deviceId,
                           L647X_Action_t action,
                           L647X_Direction_t direction,
                           uint32_t speed)
{
  L647X_SendCommand(deviceId, (uint8_t)L647X_GO_UNTIL | (uint8_t)action | (uint8_t)direction, speed);
}

/******************************************************//**
 * @brief Issues L647X Hard HiZ command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdHardHiZ(uint8_t deviceId)
{
  L647X_SendCommand(deviceId, L647X_HARD_HIZ, 0);
}

/******************************************************//**
 * @brief Issues L647X Hard Stop command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdHardStop(uint8_t deviceId)
{
  L647X_SendCommand(deviceId, L647X_HARD_STOP, 0);
}

/******************************************************//**
 * @brief Issues L647X Move command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction Movement direction
 * @param[in] n_step number of steps
 * @retval None
 *********************************************************/
void L647X_CmdMove(uint8_t deviceId,
                         L647X_Direction_t direction,
                         uint32_t n_step)
{
  L647X_SendCommand(deviceId, (uint8_t)L647X_MOVE | (uint8_t)direction, n_step);
}

/******************************************************//**
 * @brief Issues L647X NOP command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdNop(uint8_t deviceId)
{
  /* Send NOP operation code to L647X */
  L647X_SendCommand(deviceId, L647X_NOP, 0);
}

/******************************************************//**
 * @brief Issues L647X Release SW command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action
 * @param[in] direction movement direction
 * @retval None
 *********************************************************/
void L647X_CmdReleaseSw(uint8_t deviceId,
                             L647X_Action_t action,
                             L647X_Direction_t direction)
{
   L647X_SendCommand(deviceId,
                     (uint8_t)L647X_RELEASE_SW |
                     (uint8_t)action |
                     (uint8_t)direction,
                     0);
}

/******************************************************//**
 * @brief Issues L647X Reset Device command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdResetDevice(uint8_t deviceId)
{
  L647X_SendCommand(deviceId, L647X_RESET_DEVICE, 0);
}

/******************************************************//**
 * @brief Issues L647X Reset Pos command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdResetPos(uint8_t deviceId)
{
  L647X_SendCommand(deviceId, L647X_RESET_POS, 0);
}

/******************************************************//**
 * @brief Issues L647X Run command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction Movement direction (FORWARD, BACKWARD)
 * @param[in] speed in steps/s
 * @retval None
 *********************************************************/
void L647X_CmdRun(uint8_t deviceId,
                  L647X_Direction_t direction,
                  uint32_t speed)
{
  L647X_SendCommand(deviceId, (uint8_t)L647X_RUN | (uint8_t)direction, speed);
}

/******************************************************//**
 * @brief Issues the SetParam command to the L647X of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Register adress (L647X_ABS_POS, L647X_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 *********************************************************/
void L647X_CmdSetParam(uint8_t deviceId, L647X_Registers_t param, uint32_t value)
{

  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t maxArgumentNbBytes = 0;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;

    for (loop = 0;loop < numberOfDevices; loop++)
    {
      spiTxBursts[0][loop] = L647X_NOP;
      spiTxBursts[1][loop] = L647X_NOP;
      spiTxBursts[2][loop] = L647X_NOP;
      spiTxBursts[3][loop] = L647X_NOP;
    }

    switch (param)
    {
      case L647X_ABS_POS:
      case L647X_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)L647X_SET_PARAM )| (param);
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;
      case L647X_EL_POS:
      case L647X_ACC:
      case L647X_DEC:
      case L647X_MAX_SPEED:
      case L647X_MIN_SPEED:
      case L647X_FS_SPD:
#ifdef L6470
      case L647X_INT_SPD:
#endif
      case L647X_CONFIG:
        spiTxBursts[1][spiIndex] = ((uint8_t)L647X_SET_PARAM )| (param);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 2;
        break;
      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)L647X_SET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    spiTxBursts[3][spiIndex] = (uint8_t)(value);

    /* SPI transfer */
    for (loop = L647X_CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes;
         loop < L647X_CMD_ARG_MAX_NB_BYTES;
         loop++)
    {
        L647X_WriteReadBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
    }
  }
}

/******************************************************//**
 * @brief Issues L647X Soft HiZ command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdSoftHiZ(uint8_t deviceId)
{
  L647X_SendCommand(deviceId, L647X_SOFT_HIZ, 0);
}

/******************************************************//**
 * @brief Issues L647X Soft Stop command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void L647X_CmdSoftStop(uint8_t deviceId)
{
  L647X_SendCommand(deviceId, L647X_SOFT_STOP, 0);
}


/******************************************************//**
 * @brief  Start the step clock A by using the given frequency
 * @param[in] newFreq in Hz of the step clock
 * @retval None
 **********************************************************/
void L647X_StartStepClock(uint8_t step_clock_pin, uint32_t newFreq)
{
	gpio_start_clock(step_clock_pin, newFreq);
}

/******************************************************//**
 * @brief  Stops the PWM uses for the step clock
 * @param  None
 * @retval None
 **********************************************************/
void L647X_StopStepClock(void)
{
	gpio_stop_clock();
}


/******************************************************//**
 * @brief Issues L647X Step Clock command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction Movement direction (FORWARD, BACKWARD)
 * @retval None
 *********************************************************/
void L647X_CmdStepClock(uint8_t deviceId,
                             L647X_Direction_t direction)
{
  L647X_SendCommand(deviceId,
                    (uint8_t)L647X_STEP_CLOCK | (uint8_t)direction,
                    0);
}

/******************************************************//**
 * @brief  Converts the ABS_POSITION register value to a 32b signed integer
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position
 **********************************************************/
int32_t L647X_ConvertPosition(uint32_t abs_position_reg)
{
	int32_t operation_result;

  if (abs_position_reg & L647X_ABS_POS_SIGN_BIT_MASK)
  {
		/* Negative register value */
		abs_position_reg = ~abs_position_reg;
		abs_position_reg += 1;

		operation_result = (int32_t) (abs_position_reg & L647X_ABS_POS_VALUE_MASK);
		operation_result = -operation_result;
  }
  else
  {
		operation_result = (int32_t) abs_position_reg;
	}
	return operation_result;
}

/******************************************************//**
 * @brief Fetch and clear status flags of all devices
 * by issuing a GET_STATUS command simultaneously
 * to all devices.
 * Then, the fetched status of each device can be retrieved
 * by using the L647X_GetFetchedStatus function
 * provided there is no other calls to functions which
 * use the SPI in between.
 * @param None
 * @retval None
 *********************************************************/
void L647X_FetchAndClearAllStatus(void)
{
  uint8_t loop;

  for (loop = 0; loop < numberOfDevices; loop++)
  {
     spiTxBursts[0][loop] = L647X_GET_STATUS;
     spiTxBursts[1][loop] = L647X_NOP;
     spiTxBursts[2][loop] = L647X_NOP;
     spiTxBursts[3][loop] = L647X_NOP;
     spiRxBursts[0][loop] = 0;
     spiRxBursts[1][loop] = 0;
     spiRxBursts[2][loop] = 0;
     spiRxBursts[3][loop] = 0;
  }

  for (loop = 0;
       loop < L647X_CMD_ARG_NB_BYTES_GET_STATUS +
              L647X_RSP_NB_BYTES_GET_STATUS;
       loop++)
  {
        L647X_WriteReadBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
  }
}


/******************************************************//**
 * @brief Get the value of the STATUS register which was
 * fetched by using L647X_FetchAndClearAllStatus.
 * The fetched values are available  as long as there
 * no other calls to functions which use the SPI.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval Last fetched value of the STATUS register
 *********************************************************/
uint16_t L647X_GetFetchedStatus(uint8_t deviceId)
{
  uint16_t status = 0;
  if (numberOfDevices > deviceId)
  {
    uint8_t spiIndex = numberOfDevices - deviceId - 1;
    status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  }
  return (status);
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @param None
 * @retval L647X_FW_VERSION
 **********************************************************/
uint8_t L647X_GetFwVersion(void)
{
  return (L647X_FW_VERSION);
}

/******************************************************//**
 * @brief  Returns the mark position  of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval Mark register value converted in a 32b signed integer
 **********************************************************/
int32_t L647X_GetMark(uint8_t deviceId)
{
  return L647X_ConvertPosition(L647X_CmdGetParam(deviceId, L647X_MARK));
}

/******************************************************//**
 * @brief Return the number of devices in the daisy chain
 * @param None
 * @retval number of devices from 1 to MAX_NUMBER_OF_DEVICES
 *********************************************************/
uint8_t L647X_GetNbDevices(void)
{
    return (numberOfDevices);
}

/******************************************************//**
 * @brief  Returns the ABS_POSITION of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t L647X_GetPosition(uint8_t deviceId)
{
  return L647X_ConvertPosition(L647X_CmdGetParam(deviceId, L647X_ABS_POS));
}

/******************************************************//**
 * @brief Checks if the specified device is busy
 * by reading the Busy flag bit ot its status Register
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval true if device is busy, false zero
 *********************************************************/
bool L647X_IsDeviceBusy(uint8_t deviceId)
{
  if(!(L647X_CmdGetStatus(deviceId) & L647X_STATUS_BUSY))
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}


/******************************************************//**
 * @brief  Set the stepping mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] stepMod from full step to 1/128 microstep as specified in enum L647X_StepSel_t
 * @retval None
 **********************************************************/
void L647X_SelectStepMode(uint8_t deviceId, L647X_StepSel_t stepMod)
{
  uint8_t stepModeRegister;

  /* Set the L647X in HiZ state */
  L647X_CmdHardHiZ(deviceId);

  /* Read Step mode register and clear STEP_SEL field */
  stepModeRegister = (uint8_t)(0xF8 & L647X_CmdGetParam(deviceId,L647X_STEP_MODE)) ;

  /* Apply new step mode */
  L647X_CmdSetParam(deviceId, L647X_STEP_MODE, stepModeRegister | (uint8_t)stepMod);

  /* Reset abs pos register */
  L647X_CmdResetPos(deviceId);
}


/******************************************************//**
 * @brief Put commands in queue before synchronous sending
 * done by calling L647X_SendQueuedCommands.
 * Any call to functions that use the SPI between the calls of
 * L647X_QueueCommands and L647X_SendQueuedCommands
 * will corrupt the queue.
 * A command for each device of the daisy chain must be
 * specified before calling L647X_SendQueuedCommands.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Command to queue (all L647X commmands
 * except L647X_SET_PARAM, L647X_GET_PARAM,
 * L647X_GET_STATUS)
 * @param[in] value argument of the command to queue
 * @retval None
 *********************************************************/
void L647X_QueueCommands(uint8_t deviceId, uint8_t param, uint32_t value)
{
  if (numberOfDevices > deviceId)
  {
    uint8_t spiIndex = numberOfDevices - deviceId - 1;

    switch (param & DAISY_CHAIN_COMMAND_MASK)
    {
      case L647X_RUN:
      case L647X_MOVE:
      case L647X_GO_TO:
      case L647X_GO_TO_DIR:
      case L647X_GO_UNTIL:
      case L647X_GO_UNTIL_ACT_CPY:
       spiTxBursts[0][spiIndex] = param;
       spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
       spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
       spiTxBursts[3][spiIndex] = (uint8_t)(value);
       break;
      default:
       spiTxBursts[0][spiIndex] = L647X_NOP;
       spiTxBursts[1][spiIndex] = L647X_NOP;
       spiTxBursts[2][spiIndex] = L647X_NOP;
       spiTxBursts[3][spiIndex] = param;
    }
  }
}

/******************************************************//**
 * @brief Put commands in queue before synchronous sending
 * done by calling L647X_SendQueuedCommands.
 * Any call to functions that use the SPI between the calls of
 * L647X_QueueCommands and L647X_SendQueuedCommands
 * will corrupt the queue.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Command to queue (all L647X commmands
 * except L647X_SET_PARAM, L647X_GET_PARAM,
 * L647X_GET_STATUS)
 * @param None
 * @retval None
 *********************************************************/
void L647X_SendQueuedCommands(void)
{
  uint8_t loop;

  for (loop = 0;
       loop < L647X_CMD_ARG_MAX_NB_BYTES;
       loop++)
  {
        L647X_WriteReadBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
  }
}

/******************************************************//**
 * @brief  Locks until the device becomes not busy
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void L647X_WaitWhileActive(uint8_t deviceId)
 {
	/* Wait while motor is running */
	while (L647X_IsDeviceBusy(deviceId) != 0);
}

/******************************************************//**
 * @brief  Locks until all devices become not busy
 * @param None
 * @retval None
 **********************************************************/
void L647X_WaitForAllDevicesNotBusy(void)
{
	bool busy = TRUE;
	uint8_t loop;

	/* Wait while at least one is active */
	while (busy)
  {
    busy = FALSE;
    for (loop = 0; loop < numberOfDevices; loop++)
    {
      busy  |= L647X_IsDeviceBusy(loop);
    }
  }
}
