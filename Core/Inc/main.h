/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Enum typedef containing servo's state. Values based on competition driverless state data logging
typedef enum {SERVICE_BRAKE_DISENGAGED = 1,
							SERVICE_BRAKE_ENGAGED,
							SERVICE_BRAKE_AVAILABLE} serviceBrakeStatus;

// Enum typedef containing EBS' state. Values based on competition driverless state data logging
typedef enum {EBS_UNAVAILABLE = 1,
							EBS_ARMED,
							EBS_TRIGGERED} ebsStatus;

// Struct typedef that contains continuous monitoring variables
typedef struct {
	uint16_t period_ms;													// Continuous monitoring timer period in ms (determined on build time)
	
	bool allOK;																	// Continuous monitoring variable. True -> Every check is successful, False -> One or more checks failed
	
	bool watchdogInitialCheckBypass;						// Bypass variable that when true watchdog pulse is deactivated (even if all the checks are successful). Used in AS initial check routine
	
	bool tankPressureOK;												// If true, tank pressure sensor input messages are on time and in-bounds
	uint32_t tankPressureErrorCounter;					// Counter that increments by 1 every time an invalid value arrives via CAN
	uint32_t tankPressureTimeoutCounter;				// Counter that increments by 1 with every tick of the continuous monitoring timer
	
	bool brakePressureOK;												// If true, brake sensor input messages are on time and in-bounds
	uint32_t brakePressureErrorCounter;					// Counter that increments by 1 every time an invalid value arrives via CAN
	uint32_t brakePressureTimeoutCounter;				// Counter that increments by 1 with every tick of the continuous monitoring timer
	
	bool servoTransferFunctionCheckOK;					// If true, continuous monitoring of servo's transfer function check is ok
	bool servoInterlockCheckOK;									// If true, continuous monitoring of servo's connection is ok
	bool servoSupplyCheckOK;										// If true, continuous monitoring of servo's supply is ok (positive supply -> overvoltage/undervoltage, negative supply -> grounded)
	uint32_t servoTransferFunctionErrorCounter; // Counter that increments by 1 every time a servo's transfer function check fails
	uint32_t servoInterlockErrorCounter;				// Counter that increments by 1 every time servo's connector is detected to be unconnected through its interlock
	uint32_t servoSupplyCheckErrorCounter;			// Counter that increments by 1 every time servo's supply check is read logical LOW from servo supply check PCB
	
	bool apuTimeoutOK;													// If true, APU is sending watchdog message on time. If false, APU timed-out
	uint32_t apuTimeoutCounter;									// Counter that increments by 1 with every tick of the continuous monitoring timer
	
} continuousMonitoringStruct;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Redundancy_Supply_OK_Pin GPIO_PIN_13
#define Redundancy_Supply_OK_GPIO_Port GPIOC
#define Set_finish_state_Pin GPIO_PIN_0
#define Set_finish_state_GPIO_Port GPIOC
#define ASB_LED_Pin GPIO_PIN_1
#define ASB_LED_GPIO_Port GPIOC
#define Error_Pin GPIO_PIN_2
#define Error_GPIO_Port GPIOC
#define AS_driving_mode_Pin GPIO_PIN_3
#define AS_driving_mode_GPIO_Port GPIOC
#define Tank_pressure_Pin GPIO_PIN_1
#define Tank_pressure_GPIO_Port GPIOA
#define EBS_Redundancy_PWM_Pin GPIO_PIN_6
#define EBS_Redundancy_PWM_GPIO_Port GPIOA
#define EBS_set_armed_Pin GPIO_PIN_7
#define EBS_set_armed_GPIO_Port GPIOA
#define K2_RES_Pin GPIO_PIN_4
#define K2_RES_GPIO_Port GPIOC
#define ASMS_out_Pin GPIO_PIN_5
#define ASMS_out_GPIO_Port GPIOC
#define AS_finished_Pin GPIO_PIN_0
#define AS_finished_GPIO_Port GPIOB
#define EBS_Valve_In_Pin GPIO_PIN_2
#define EBS_Valve_In_GPIO_Port GPIOB
#define EBS_is_armed_Pin GPIO_PIN_10
#define EBS_is_armed_GPIO_Port GPIOB
#define AS_relay_out_Pin GPIO_PIN_14
#define AS_relay_out_GPIO_Port GPIOB
#define K3_RES_Pin GPIO_PIN_6
#define K3_RES_GPIO_Port GPIOC
#define EBS_activated_Pin GPIO_PIN_7
#define EBS_activated_GPIO_Port GPIOC
#define TSMS_out_NOT_Pin GPIO_PIN_8
#define TSMS_out_NOT_GPIO_Port GPIOC
#define DCDC_Enable_Pin GPIO_PIN_9
#define DCDC_Enable_GPIO_Port GPIOC
#define Watchdog_pulse_Pin GPIO_PIN_8
#define Watchdog_pulse_GPIO_Port GPIOA
#define User_LED_Pin GPIO_PIN_10
#define User_LED_GPIO_Port GPIOA
#define Brake_engage_NOT_Pin GPIO_PIN_10
#define Brake_engage_NOT_GPIO_Port GPIOC
#define Power_Good_Pin GPIO_PIN_11
#define Power_Good_GPIO_Port GPIOC
#define AS_close_SDC_Pin GPIO_PIN_12
#define AS_close_SDC_GPIO_Port GPIOC
#define Servo_Interlock_Pin GPIO_PIN_4
#define Servo_Interlock_GPIO_Port GPIOB
#define AS_relay_signal_buffered_Pin GPIO_PIN_6
#define AS_relay_signal_buffered_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
