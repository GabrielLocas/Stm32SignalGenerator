/*
 * comm.c
 *
 *  Created on: Jun 19, 2023
 *      Author: gabri
 */
#include "../../Inc/comm.h"
#include "../../Inc/waves.h"
#include "stdint.h"
#include "main.h"

extern DMA_HandleTypeDef hdma_dac1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c2;

//UART reception data
uint8_t Rx_data[PACKET_SIZE] = {0};

//Default parameters
uint8_t stim_freq = 1;  		// 0 - 255 (INT_MAX) Hz
uint8_t duty_cycle = 0; 		// 0 - 255 (INT_MAX) is max duty cycle
uint8_t wave_type = 0;			// 0 (sine) 1 (triangle) 2 (square) 3 (saw), anything else produces no sound
unsigned int pitch = 440; 		// 0 - 65535 Hz
uint8_t randomOn = 0; 			// 0 (off), anything else is random
uint8_t sound_intensity = 0; 	// 0 - 255 (INT_MAX)
uint8_t light_intensity = 0; 	// 0 - 255 (INT_MAX) where 255 is max intensity
uint8_t i2cFlag = 0;			// flag for knowing when to send i2c packet

void receiveUARTpacket(){
	// Stop DAC
	  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
	  HAL_TIM_Base_Stop(&htim2);

	  //Get values from Rx buffer
	  wave_type = Rx_data[0];
	  pitch = Rx_data[1] << 8 | Rx_data[2];
	  stim_freq = Rx_data[3];
	  duty_cycle = Rx_data[4];
	  randomOn = Rx_data[5];
	  sound_intensity = Rx_data[6];
	  light_intensity = Rx_data[7];

	  //Set pitch
	  if (pitch){
		htim2.Instance->PSC = (42000000/N_SAMPLES) / pitch;
	  }

	  //Set stimulation frequency
	  if (stim_freq){
		  htim3.Instance->PSC = 21000/stim_freq;
	  }

	  //Set duty cycle for light frequency
	  if (!randomOn){
		  htim3.Instance->CCR1 = (TIMER_PRESCALER*duty_cycle)/INT_MAX;
	  }

	  //Set intensity for MAX9744 with I2C
	  //uint8_t tmp[] = {sound_intensity};
	  //HAL_I2C_Master_Transmit(&hi2c2, 0b10010010, tmp, 1, 1);
	  i2cFlag = 1;

	  //Set frequency for PWM controlling light intensity
	  htim4.Instance->PSC = (htim3.Instance->PSC)/LIGHT_INTENSITY_PWM_MULTIPLIER;

	  // Start DAC timer
	  HAL_TIM_Base_Start(&htim2);
}
