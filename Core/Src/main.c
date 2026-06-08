/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
	
	/* TIMER FREQUENCIES AND FUNCTIONALITY
	 * 	 TIM1  (APB2 -> 180MHz) : 18000  : 200 		-> 50Hz (20ms)  -> Watchdog PWM
	 * 	 TIM2  (APB1 -> 90MHz)  : 900 	 : 2000 	-> 50Hz (20ms)  -> Continuous Monitoring timer
	 * 	 TIM3  (APB1 -> 90MHz)  : 100 	 : 18000 	-> 50Hz (20ms)  -> Servo PWM
	 * 	 TIM4  (APB1 -> 90MHz)  : 900 	 : 100 		-> 1kHz (1ms)   -> Custom delay function
	 * 	 TIM5  (APB1 -> 90MHz)  : 900 	 : 100 		-> 1kHz (1ms)   -> GPIO, servo handling and EBS state check
	 * 	 TIM7  (APB1 -> 90MHz)  : 900 	 : 5000 	-> 20Hz (50ms) 	-> CAN transmitting
	 *	 TIM8  (APB2 -> 180MHz) : 18000  : 200 		-> 50Hz (20ms)  -> ADC conversion start
	 * 	 TIM12 (APB1 -> 90MHz)  : 900 	 : 100 		-> 1kHz (1ms)   -> EBS Visible check function
	 */
	 
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "can_mcu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define OUTSIDE_OF_VEHICLE_MODE 						false			// If true, MCU won't wait for CAN messages and will assume certain variables have values that ensure the program's normal flow (for debugging purposes)
//#define NO_SERVO_MODE												false 		// IF true, servo is considered to be non functional and its continuous monitoring routine will always return ok

#define SERVO_MIN_POS_PWM_VALUE 			 			430				// Absolute min value for __HAL_TIM_SetCompare PWM output to make servo go to 0deg (with TIM3 prescaler value of 99 and counter period of 17999)
#define SERVO_MAX_POS_PWM_VALUE 			 			2260   		// Absolute max value for __HAL_TIM_SetCompare PWM output to make servo go to 180deg (with TIM3 prescaler value of 99 and counter period of 17999)
#define SERVO_0_PERCENT_PWM_VALUE  		 			800 			// PWM width value that corresponds to servo's position just before braking force is applied to the brake calipers
#define SERVO_100_PERCENT_PWM_VALUE 	 			2250			// PWM width value that corresponds to servo's final position (According to tests, servo can handle at least ~15bar to the point where it was mounted)
#define SERVO_ENGAGED_POS_PWM_VALUE					2000 			// PWM width value that corresponds to servo's engaged position (prevents vehicle from rolling on a 15% slope)
#define SERVO_ACTIVATED_POS_PWM_VALUE				2000			// PWM width value that corresponds to servo's activated position (for use with simple control logic with 2 servo states deactivated-activated)
#define SERVO_ACTIVATED_POS_PWM_VALUE_ACCEL 2000			// PWM width value that corresponds to servo's activated position on acceleration event (for use with simple control logic with 2 servo states deactivated-activated)
#define SERVO_TRANSFER_FUNCTION_ERROR_MS		1500			// How many milliseconds have to pass to declare an error in servo's transfer function check as an continuous monitoring error
#define SERVO_INTERLOCK_ERROR_MS 						1000			// How many milliseconds have to pass to declare an error in servo's interlock readout from corresponding GPIO
#define SERVO_SUPPLY_CHECK_ERROR_MS					1000			// How many milliseconds have to pass to declare an error in servo's supply check readout from corresponding GPIO (digital signal from Servo Supply Check PCB)

#define TANK_PRESSURE_THRESHOLD 			 			0.522			// 0.5219780 bar corresponds to about 1000 raw pressure in EBS tank regulated output
#define TANK_PRESSURE_LOWER_BOUND 		 			-1.4 			// -1.4120879 bar corresponds to 360 raw pressure (sensor output is 1V-5V)
#define TANK_PRESSURE_UPPER_BOUND 		 			8.88			// 8.883791208 bar corresponds to 3767 raw pressure ~4.6V -> ~9bar (sensor output is 1V-5V but we fill the tank with less than 9bar so we can check for upper bound values)
#define TANK_PRESSURE_TIMES_ERROR 		 			3 				// How many consecutive out of bounds tank pressure values must be received before stopping watchdog to EBS NPL
#define TANK_PRESSURE_TIMEOUT_MS			 			2000			// How many milliseconds have to pass since last receiving tank pressure message to declaring tankPressureOK -> false

#define BRAKE_PRESSURE_BRAKING 				 			3.972			// 3.972 Lower bound for hydraulic brake pressure in bar that is considered adequate to light up the brakelight (485->~4bar)
#define BRAKE_PRESSURE_RELEASED 			 			1.499			// Upper bound for hydraulic brake pressure in bar that is assumed that the brake calipers do not exert force on the brake disc (438->~1.5bar) (Currently used only for servo function check)
#define BRAKE_PRESSURE_LOWER_BOUND  	 			-2.604		// Pressure in bar that corresponds to ~0.4V to use for sensor error checking (sensor outputs 0.5V to 4.5V)
#define BRAKE_PRESSURE_UPPER_BOUND  	 			172.34		// Pressure in bar that corresponds to 4.5V to use for sensor error checking (sensor outputs 0.5V to 4.5V)
#define BRAKE_PRESSURE_TIMES_ERROR  	 			10				// How many consecutive out of bounds hydraulic brake pressure values must be received before stopping watchdog to EBS NPL
#define BRAKE_PRESSURE_TIMEOUT_MS			 			200				// How many milliseconds have to pass since last receiving brake pressure message to declaring brakePressureOK -> false

#define APU_TIMEOUT_MS								 			2000				// Time in ms that if the APU does not send the MISSION_STATE CAN message (APU watchdog), stop toggling watchdog to EBS NPL

#define WAIT_BEFORE_INFINITE_LOOP_MS				50				// Time in ms to wait before starting infinite while-loop					
#define ASB_NPL_WAIT_FOR_FLIP_FLOPS_MS 			50				// Time in ms to wait for flip-flop latching in NPL PCB and digital read input
#define ASB_WAIT_FOR_WATCHDOG_TOGGLE_MS 		400	 			// Time in ms to wait for UCC2946 (EBS-NPL) to toggle its watchdog pin in case of watchdog pulse abscence/presence. Depends on capacitor connected to WP pin (check EBS_2020.pdf booklet)
#define ASB_WAIT_FOR_ACTUATION_MS 		 			1200			// Time in ms to wait for the brake pressure to rise

#define ASB_LED_INITIAL_CHECK_MS 			 			2325			// Time in ms showing how long the EBS LED will stay on for visible check after power cycle
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */
// System Variables
uint32_t delay_counter;									// Variable used in delay_ms
int8_t   state;													// Debugging variable
int8_t   initialCheckStep;							// Debugging variable
int8_t   message_arrived;								// Debugging variable
static bool watchdog_running = true;		// Static variable that contain last state written on each digital-input/pwm pin. Used instead of continuously setting pin state to the same value as the previous one
float Vadc;
float Vout;
float tank_pressure;

// GPIO Variables
uint8_t ASMS_out, TSMS_out_NOT, AS_relay_signal_buffered, K2_RES, K3_RES;										// Digital INPUTS from SDC and EBS NPL
uint8_t EBS_is_armed, AS_finished, Error, EBS_activated;																		// Digital INPUTS from EBS NPL
uint8_t Brake_engage_NOT, EBS_set_armed, AS_driving_mode, Set_finish_state, AS_close_SDC;		// Digital OUTPUTS to EBS NPL
uint8_t DCDC_Power_Good, DCDC_Enable;																												// Digital signals from/to servo DCDC
uint8_t Redundancy_Supply_OK, Servo_Interlock;																							// Digital signals related to servo monitoring

// ADC Variables
bool     ADC_Ready = false;
uint16_t adcValue[1] = {0};

// Initial state variables
bool 		 initialChecked;											// Variable that is true when initial check completes successfully and the vehicle transitions to AS Ready or Manual Driving state. Resets every time the vehicle transitions to AS Off
bool		 ASB_LED_visibleCheckFlag;						// Visible check LED variable (True for n seconds counting from startup then false for the rest time)
uint16_t ASB_LED_visibleCheckFlag_counter;		// Visible check LED counter variable (Counts up to a value that represents the aforementioned n seconds that the LED is lit up)

// Continuous monitoring variables
continuousMonitoringStruct monitor;						// Struct that contains continuous monitoring variables, see main.h
bool monitoring;															// When is true, continuous monitoring function runs

// Status variables
serviceBrakeStatus servoStatus;								// Enum variable that contains servo status based on competition data logging, see main.h
ebsStatus 				 ebsState;									// Enum variable that contains EBS status based on competition data logging, see main.h

// CAN Variables
CAN_TxHeaderTypeDef asbStatusMsgTx;																			// Header of the transmited CAN message
CAN_RxHeaderTypeDef msgHeaderRx;																				// Header of the received CAN message
uint32_t txMailbox;																											// Keeps the last mailbox used to send CAN msg
uint8_t rxData[8];																											// Buffer for received data
uint8_t txData[8];																											// Buffer for transmitted data
struct can_mcu_dash_brake_t canBrakePressureStruct;											// Structs that contain the last values received from the CAN for each msg (from files generated using cantools and dbc files)
struct can_mcu_apu_state_mission_t canStateMissionStruct;
struct can_mcu_ecu_bools_t canECUBoolsStruct;
struct can_mcu_asb_t canTxStruct;
bool 	 servoCommandReceived = false;																		// Flag that becomes true when a new CAN msg regarding servo commands is received. Becomes false when the command is executed by the MCU
//int i = 1800;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM12_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void supervisorInit(void);								// Initializes functions and variables
void variablesInit(void);									// Initializes personal variables
void CAN_initialize(void);								// Function that initializes CAN Tx msg and CAN filter 
void delay_ms(uint32_t millis); 					// Custom made delay function (since HAL_Delay has strange behaviour)
void asInitialCheck(void);								// Autonomous initial checkup sequence
bool continuousMonitoring(void);					// Continuous monitoring function
void manualInitialCheck(void);						// Manual initial checkup sequence
bool isMissionSelectedAutonomous(void); 	// Function that if mission selected is autonomous returns true
bool isMissionSelectedManual(void); 			// Function that if mission selected is manual returns true
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
	{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_TIM12_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	supervisorInit();
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcValue, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		initialCheckStep = -1;
		if (isMissionSelectedAutonomous())  						// Autonomous mission received from APU AND AS_mode = false received from ECU
		{
			AS_driving_mode = 1;			// In autonomous mission, AS_driving_mode must be HIGH in order to arm the EBS and close the AS relay
			HAL_GPIO_WritePin(AS_driving_mode_GPIO_Port, AS_driving_mode_Pin, GPIO_PIN_SET);
			if (canStateMissionStruct.as_state == CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_OFF_CHOICE)
			{
				DCDC_Enable = 1;
				HAL_GPIO_WritePin(DCDC_Enable_GPIO_Port, DCDC_Enable_Pin, GPIO_PIN_SET);
				state = 1;																	// Debugging variable
				initialChecked = false;											// Every time the vehicle enters AS_off state, initialChecked returns to false. To make it true again you have to enter any of the initial checks (as or manual) and complete them successfully
				Set_finish_state = 0;												// Not in finished state (AS finished == 0)
				HAL_GPIO_WritePin(Set_finish_state_GPIO_Port, Set_finish_state_Pin, GPIO_PIN_RESET);
				servoStatus = SERVICE_BRAKE_DISENGAGED;			// Servo is unavailable (ASMS is off so service brake supply is cut)
				monitoring = true;
				if(ASMS_out == 1 && EBS_is_armed == 0 && AS_finished == 0)		// If ASMS was closed, then proceed to initial check. Also wait for user to reset car before proceeding to initial check. If the vehicle was initial checked before, the EBS_is_armed and AS_finished flip flops will be latched and Reset button must be pressed to clear them. 
					asInitialCheck();
			}
			else if(canStateMissionStruct.as_state == CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_READY_CHOICE)
			{
				DCDC_Enable = 1;
				HAL_GPIO_WritePin(DCDC_Enable_GPIO_Port, DCDC_Enable_Pin, GPIO_PIN_SET);
				state = 2;																	// Debugging variable
				Set_finish_state = 0;												// Not in finished state
				HAL_GPIO_WritePin(Set_finish_state_GPIO_Port, Set_finish_state_Pin, GPIO_PIN_RESET);
				monitoring = true;
				if (ASMS_out == 1)													// If ASMS is on, then servo is supplied and must be engaged (Check AS flowchart)
					servoStatus = SERVICE_BRAKE_AVAILABLE; // OG: servoStatus = SERVICE_BRAKE_ENGAGED;
				else																				// However, one may want to transition back to AS OFF state and for that to happen ASMS must be in the OFF position (servo not supplied and therefore is unavailable)
					servoStatus = SERVICE_BRAKE_DISENGAGED;
			}
			else if(canStateMissionStruct.as_state == CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_DRIVING_CHOICE)
			{
				DCDC_Enable = 1;
				HAL_GPIO_WritePin(DCDC_Enable_GPIO_Port, DCDC_Enable_Pin, GPIO_PIN_SET);
				state = 4;																	// Debugging variable
				Set_finish_state = 0;
				HAL_GPIO_WritePin(Set_finish_state_GPIO_Port, Set_finish_state_Pin, GPIO_PIN_RESET);
				servoStatus = SERVICE_BRAKE_AVAILABLE;			// In AS Driving state the service brake responds to commands from the AS
				monitoring = true;
				delay_ms(200);
				if(canStateMissionStruct.as_set_finished == CAN_MCU_APU_STATE_MISSION_AS_SET_FINISHED_SET__FINISHED__TRUE_CHOICE)
				{
					DCDC_Enable = 1;
					HAL_GPIO_WritePin(DCDC_Enable_GPIO_Port, DCDC_Enable_Pin, GPIO_PIN_SET);
					state = 16;																	// Debugging variable
					Set_finish_state = 1;												// AS Finished state so the EBS must be activated through AS Finished latch
					HAL_GPIO_WritePin(Set_finish_state_GPIO_Port, Set_finish_state_Pin, GPIO_PIN_SET);
					delay_ms(200);
					monitoring = false;
					if(ASMS_out == 1)														// If ASMS is on, service brake should be engaged
						servoStatus = SERVICE_BRAKE_ENGAGED;
					else																				// If ASMS is off, then brakes should be released (Turning off ASMS will cut the supply to servo and the brakes will be released)
						servoStatus = SERVICE_BRAKE_DISENGAGED;
				}
			}
			else if(canStateMissionStruct.as_state == CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_EMERGENCY_CHOICE)
			{
				DCDC_Enable = 1;
				HAL_GPIO_WritePin(DCDC_Enable_GPIO_Port, DCDC_Enable_Pin, GPIO_PIN_SET);
				state = 8;																	// Debugging variable
				Set_finish_state = 0;												// Not in finished state
				HAL_GPIO_WritePin(Set_finish_state_GPIO_Port, Set_finish_state_Pin, GPIO_PIN_RESET);
				monitoring = true;													// Continuous monitoring function
				if(ASMS_out == 1)														// If ASMS is on, service brake should be engaged
					servoStatus = SERVICE_BRAKE_ENGAGED;
				else																				// If ASMS is off, then brakes should be released (Turning off ASMS will cut the supply to servo and the brakes will be released)
					servoStatus = SERVICE_BRAKE_DISENGAGED;
			}
		}
		else if (isMissionSelectedManual())			// Manual_Driving mission received from APU OR AS_mode = true received from ECU
		{
			AS_driving_mode = 0;		// In manual mission, AS_driving_mode must be LOW
			HAL_GPIO_WritePin(AS_driving_mode_GPIO_Port, AS_driving_mode_Pin, GPIO_PIN_RESET);
			DCDC_Enable = 0;
			HAL_GPIO_WritePin(DCDC_Enable_GPIO_Port, DCDC_Enable_Pin, GPIO_PIN_RESET);
			Set_finish_state = 0;
			HAL_GPIO_WritePin(Set_finish_state_GPIO_Port, Set_finish_state_Pin, GPIO_PIN_RESET);
//			if(canStateMissionStruct.as_state == CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_OFF_CHOICE)
//			{
			state = -1;
			initialChecked = false;
			if((ASMS_out == 0 && EBS_is_armed == 0 && AS_finished == 0 /*&& ebsState == EBS_UNAVAILABLE*/))
				manualInitialCheck();
//			}
			else if(canStateMissionStruct.as_state == CAN_MCU_APU_STATE_MISSION_AS_STATE_MANUAL_DRIVING_CHOICE)
			{
				state = -2;
				if (ebsState == EBS_TRIGGERED)
				{
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since the initial check failed and the SDC was closed in previous steps
					delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
				}
			}
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 17999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 899;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 17999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 899;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 899;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 899;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 17999;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 899;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 99;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Set_finish_state_Pin|ASB_LED_Pin|AS_driving_mode_Pin|DCDC_Enable_Pin
                          |Brake_engage_NOT_Pin|AS_close_SDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EBS_set_armed_Pin|User_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Redundancy_Supply_OK_Pin Error_Pin K2_RES_Pin ASMS_out_Pin
                           K3_RES_Pin EBS_activated_Pin TSMS_out_NOT_Pin Power_Good_Pin */
  GPIO_InitStruct.Pin = Redundancy_Supply_OK_Pin|Error_Pin|K2_RES_Pin|ASMS_out_Pin
                          |K3_RES_Pin|EBS_activated_Pin|TSMS_out_NOT_Pin|Power_Good_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Set_finish_state_Pin ASB_LED_Pin AS_driving_mode_Pin DCDC_Enable_Pin
                           Brake_engage_NOT_Pin */
  GPIO_InitStruct.Pin = Set_finish_state_Pin|ASB_LED_Pin|AS_driving_mode_Pin|DCDC_Enable_Pin
                          |Brake_engage_NOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EBS_set_armed_Pin User_LED_Pin */
  GPIO_InitStruct.Pin = EBS_set_armed_Pin|User_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AS_finished_Pin EBS_Valve_In_Pin EBS_is_armed_Pin AS_relay_out_Pin
                           Servo_Interlock_Pin AS_relay_signal_buffered_Pin */
  GPIO_InitStruct.Pin = AS_finished_Pin|EBS_Valve_In_Pin|EBS_is_armed_Pin|AS_relay_out_Pin
                          |Servo_Interlock_Pin|AS_relay_signal_buffered_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AS_close_SDC_Pin */
  GPIO_InitStruct.Pin = AS_close_SDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AS_close_SDC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void asInitialCheck(void)
{/*	Notes:
		1) The supply check is done with the Redundancy_Supply_OK pin and it checks for Overvoltage, Undervoltage and open-wire.
		2) Currently unused variables:
				K2 RES ------------------------(RES button - used for GO signal)
				K3 RES ------------------------(RES switch - can be used for GO signal as well but it doesn't in Thetis)
				AS relay signal buffered ------(Output of AS relay. Could be used as an extra check if the relay is closed)
				AS relay out ------------------(ENTER EXPLANATION...)
	*/
	// Wait for RES relays to close and for errors regarding watchdog to disappear
	servoStatus = SERVICE_BRAKE_ENGAGED;
	initialCheckStep = 0;
	do {
		if(ASMS_out == 0 || (!isMissionSelectedAutonomous()))	// If ASMS state shifts to 0 or mission changes to NOT(autonomous), then stop initial check
			return;
		delay_ms(50);
	} while(Error == 0); // Error == 1 means that we have both closed SDC before AS relay and watchdog is received successfully

	// Check watchdog. Since it's disabled, NPL must report an Error
	initialCheckStep = 1;
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);					// Stop watchdog PWM
	delay_ms(ASB_WAIT_FOR_WATCHDOG_TOGGLE_MS);				// Wait a bit for watchdog signal (on EBS-NPL) to go to LOW before checking
	if (Error == 1)																		// if Error is still HIGH then something went wrong.
		return;
		
	// Check watchdog. If it starts, NPL must not report an Error
	initialCheckStep = 2;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);					// Start watchdog PWM
	delay_ms(ASB_WAIT_FOR_WATCHDOG_TOGGLE_MS);				// Wait a bit for watchdog signal (on EBS-NPL) to go to LOW before checking
	if (Error == 0)																		// if Error is not HIGH then something went wrong.
		return;
		
	// Check tank pressure's raw value
	initialCheckStep = 3;
	do {		// Wait until both pressures are above their thresholds
		if (ASMS_out == 0 || (!isMissionSelectedAutonomous())) // If ASMS has opened (ASMS_out == 0) or mission changes to NOT(autonomous), then stop initial check
			return;
		delay_ms(50);		// If it's all ok, the program won't enter the above if-statement so it will be the same as an empty loop.
										// However, it is noticed that this empty loop doesn't work (it doesn't even perform the check of the condition).
										// So a slight delay was put just to not be empty. It has been used in all loops of the program for the same reason.
	} while ((tank_pressure < TANK_PRESSURE_THRESHOLD) || (canBrakePressureStruct.brake_pressure < BRAKE_PRESSURE_BRAKING));
		
	/* -------------------------------- CLOSE AUTONOMOUS SDC -------------------------------- */
	initialCheckStep = 4;
	AS_close_SDC = 1;
	HAL_GPIO_WritePin(AS_close_SDC_GPIO_Port, AS_close_SDC_Pin, GPIO_PIN_SET);			// Since watchdog is ok, AS_close_SDC shall go to HIGH
	delay_ms(ASB_NPL_WAIT_FOR_FLIP_FLOPS_MS);		// Wait for time-delays (RC circuits) to reach voltages necessary to change logic level on EBS NPL PCB
	AS_close_SDC = 0;
	HAL_GPIO_WritePin(AS_close_SDC_GPIO_Port, AS_close_SDC_Pin, GPIO_PIN_RESET);		// No need to hold AS_close_SDC high since the flip-flop is latched
	HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_SET);							// Turn on LED while waiting for rest of SDC to close (TSMS and AS relay-AdAct)
	/* Time to close autonomous SDC. Requirements:
	 * ASMS_out = HIGH
	 * AS_driving_mode = HIGH
	 * AS_close_SDC = HIGH
	 * EBS_is_armed = HIGH
	 * AS_finished = LOW
	 */	
	do {		// Wait for TSMS to close
		if (ASMS_out == 0 || EBS_is_armed == 1 || (!isMissionSelectedAutonomous())) // If ASMS has opened OR EBS is armed OR mission changes to NOT(autonomous), then stop autonomous initial check
		{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since the SDC was closed in previous steps
			delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
			return;
		}
		delay_ms(50);
	} while (TSMS_out_NOT == 1);
	HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);		// Turn off LED
	servoStatus = SERVICE_BRAKE_DISENGAGED;
//	delay_ms(1000);
//	while(canBrakePressureStruct.brake_pressure >= BRAKE_PRESSURE_BRAKING)	// Wait for hydraulic brake pressure to drop before actuating EBS (ASF correction request)
//	{
//		if(TSMS_out_NOT == 1 || EBS_is_armed == 1 || ASMS_out == 0 || (!isMissionSelectedAutonomous())) {
//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since SDC was closed in previous steps
//			delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
//			return;
//		}
//	}
//	delay_ms(1000);
	
	// Check EBS arming
	initialCheckStep = 5;
	EBS_set_armed = 1;
	HAL_GPIO_WritePin(EBS_set_armed_GPIO_Port, EBS_set_armed_Pin, GPIO_PIN_SET);				// set the EBS to armed state (EBS_is_armed = HIGH)
	delay_ms(ASB_NPL_WAIT_FOR_FLIP_FLOPS_MS);																						// Wait a bit before checking
	HAL_GPIO_WritePin(EBS_set_armed_GPIO_Port, EBS_set_armed_Pin, GPIO_PIN_RESET);			// No need to hold EBS_set_armed high since the flip-flop is latched
	EBS_set_armed = 0;
	delay_ms(1000);
	
	// Turn off EBS electric valve
	initialCheckStep = 6;
	Brake_engage_NOT = 0;															// De-energize EBS electric valve (EBS_activated = HIGH)
	HAL_GPIO_WritePin(Brake_engage_NOT_GPIO_Port, Brake_engage_NOT_Pin, GPIO_PIN_RESET);
	delay_ms(ASB_WAIT_FOR_ACTUATION_MS);							// Wait a bit for the cylinder to actuate the brake pedal
	do {		// Check if sufficient brake pressure is built up
		if (TSMS_out_NOT == 1 || EBS_is_armed == 0 || ASMS_out == 0 || (!isMissionSelectedAutonomous()))
		{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since the SDC was closed in previous steps
			delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
			return;
		}
		delay_ms(50);
	} while (canBrakePressureStruct.brake_pressure < BRAKE_PRESSURE_BRAKING);
	delay_ms(1000);
	
	Brake_engage_NOT = 1;
	HAL_GPIO_WritePin(Brake_engage_NOT_GPIO_Port, Brake_engage_NOT_Pin, GPIO_PIN_SET);				// Disengage brake
	do {		// Wait for hydraulic brake pressure to drop before actuating servo (ASF correction request)
		if (TSMS_out_NOT == 1 || EBS_is_armed == 0 || ASMS_out == 0 || (!isMissionSelectedAutonomous()))
		{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since the SDC was closed in previous steps
			delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
			return;
		}
		delay_ms(50);
	} while (canBrakePressureStruct.brake_pressure >= BRAKE_PRESSURE_BRAKING);
	delay_ms(1000);
	
	// Check service brake (servo motor)
	initialCheckStep = 7;
	servoStatus = SERVICE_BRAKE_ENGAGED;								// Engage brake pedal with service brake
	delay_ms(ASB_WAIT_FOR_ACTUATION_MS);								// Wait a bit for the servo to actuate the brake pedal
	
	do {		// Check if sufficient brake pressure is built up
		delay_ms(200);
		if (TSMS_out_NOT == 1 || ASMS_out == 0 || (!isMissionSelectedAutonomous())) // || EBS_is_armed == 0
		{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since the SDC was closed in previous steps
			delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
			return;
		}
		delay_ms(50);
	} while (canBrakePressureStruct.brake_pressure < BRAKE_PRESSURE_BRAKING);
	servoStatus = SERVICE_BRAKE_AVAILABLE;							// Disengage servo to let ECU engage it
	delay_ms(2000);
	
	
	// Signal APU that initial check is OK and wait for transition to AS Ready
	initialCheckStep = 8;
	initialChecked = true;
	for(int i = 0; i < 100; i++)		// Check every t(ms) and n times if AS_READY state was received and if it is received, return
	{
		delay_ms(90);
		if (canStateMissionStruct.as_state == CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_READY_CHOICE)
			return;
	}
	initialChecked = false;													// If APU did not respond, initial check failed
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since the initial check failed and the SDC was closed in previous steps
	delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
}

void manualInitialCheck(void)
{
	/* -------------------------------- CLOSE MANUAL SDC -------------------------------- */
	initialCheckStep = 20;
	/* Time to close manual SDC. Requirements:
	 * ASMS_out = LOW
	 * AS_driving_mode = LOW
	 * AS_close_SDC = HIGH
	 */
	AS_close_SDC = 1;
	HAL_GPIO_WritePin(AS_close_SDC_GPIO_Port, AS_close_SDC_Pin, GPIO_PIN_SET);		// Time to close SDC
	HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_SET);						// Turn on LED while waiting for rest of SDC to close (TSMS and AS relay-AdAct)
	do {		// wait for TSMS to close (TSMS_out_NOT == 1)
		if ((ASMS_out == 1) || (!isMissionSelectedManual()) || ebsState != EBS_UNAVAILABLE)		// If ASMS closes (ASMS_out = 1) or mission changes to anything but manual or EBS is triggered, then stop manual initial check
		{
				AS_close_SDC = 0;
				HAL_GPIO_WritePin(AS_close_SDC_GPIO_Port, AS_close_SDC_Pin, GPIO_PIN_RESET);
//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since the SDC was closed in previous steps
//			delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
			return;
		}
		delay_ms(50);
	} while (TSMS_out_NOT == 1);
	delay_ms(ASB_NPL_WAIT_FOR_FLIP_FLOPS_MS);																		// Wait for time-delays (RC circuits) to reach voltages necessary to change logic level on EBS NPL PCB
	AS_close_SDC = 0;			// No need to hold AS_close_SDC high since the flip-flop is latched
	HAL_GPIO_WritePin(AS_close_SDC_GPIO_Port, AS_close_SDC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);					// Turn off LED
	
	// Check communication with APU
	initialCheckStep = 21;
	initialChecked = true;
	// No need to check for communication with APU while running manually
//	for(int i = 0; i < 100; i++)		// Check every t(ms) and n times if AS_READY state was received and if it is received, return
//	{
//		delay_ms(50);
//		if(canStateMissionStruct.as_state == CAN_MCU_APU_STATE_MISSION_AS_STATE_MANUAL_DRIVING_CHOICE)
//			return;
//	}
//	initialChecked = false;					// If APU did not respond, initial check failed
//	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);				// We need to disable watchdog to provoke an error on EBS NPL to open the SDC since the initial check failed and the SDC was closed in previous steps
//	delay_ms(500);																	// Wait a bit for the absence of watchdog to provoke an error
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// Retract watchdog bypass
}

bool continuousMonitoring(void)
{
	/* ---------------- TANK PRESSURE CHECK ---------------- */
	// Check if tank pressure is within the limits
	if(tank_pressure < TANK_PRESSURE_LOWER_BOUND || tank_pressure > TANK_PRESSURE_UPPER_BOUND || tank_pressure < TANK_PRESSURE_THRESHOLD) // Logic says that tank's pressure will always first pass from TANK_PRESSURE_THRESHOLD and then TANK_PRESSURE_LOWER_BOUND
		monitor.tankPressureErrorCounter++;
	else
		monitor.tankPressureErrorCounter = 0;
	
	// Check for exceeding number of errors
	if(monitor.tankPressureErrorCounter >= TANK_PRESSURE_TIMES_ERROR)
		monitor.tankPressureOK = false;
	else
		monitor.tankPressureOK = true;
	
	/* ---------------- BRAKE PRESSURE CHECK ---------------- */
	// Check if brake pressure is within the limits
	if(canBrakePressureStruct.brake_pressure < BRAKE_PRESSURE_LOWER_BOUND || canBrakePressureStruct.brake_pressure > BRAKE_PRESSURE_UPPER_BOUND) // Logic says that tank will always first pass from TANK_PRESSURE_THRESHOLD and then TANK_PRESSURE_LOWER_BOUND
		monitor.brakePressureErrorCounter++;
	else
		monitor.brakePressureErrorCounter = 0;
	
	// Check for timeout and exceeding number of errors
	monitor.brakePressureTimeoutCounter++;
	if(monitor.brakePressureErrorCounter >= BRAKE_PRESSURE_TIMES_ERROR || (monitor.brakePressureTimeoutCounter * monitor.period_ms) > BRAKE_PRESSURE_TIMEOUT_MS)
		monitor.brakePressureOK = false;
	else
		monitor.brakePressureOK = true;
	
	/* ---------------- APU COMMUNICATION CHECK ---------------- */
	// Check for delays of communication with APU (APU timeout)
	monitor.apuTimeoutCounter++;
	if((monitor.apuTimeoutCounter * monitor.period_ms) > APU_TIMEOUT_MS)
		monitor.apuTimeoutOK = false;
	else
		monitor.apuTimeoutOK = true;
	
	/* ---------------- SERVO CHECKS (Transfer function, Interlock, Servo Supply) ---------------- */
	// Servo transfer function  check
//	if(servoStatus == SERVICE_BRAKE_AVAILABLE)
//	{
//		if(__HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_1) > SERVO_0_PERCENT_PWM_VALUE)
//		{
//			if(canBrakePressureStruct.brake_pressure <= BRAKE_PRESSURE_RELEASED)
//				monitor.servoTransferFunctionErrorCounter++;
//			else
//				monitor.servoTransferFunctionErrorCounter = 0;
//		}
//	}
//	else if(servoStatus == SERVICE_BRAKE_ENGAGED)
//	{
//		if(canBrakePressureStruct.brake_pressure <= BRAKE_PRESSURE_BRAKING)
//			monitor.servoTransferFunctionErrorCounter++;
//		else
//			monitor.servoTransferFunctionErrorCounter = 0;
//	}
//	else
//		monitor.servoTransferFunctionErrorCounter = 0;
//		
//	if(monitor.servoTransferFunctionErrorCounter * monitor.period_ms > SERVO_TRANSFER_FUNCTION_ERROR_MS)
//		monitor.servoTransferFunctionCheckOK = false;
//	else
//		monitor.servoTransferFunctionCheckOK = true;
	
	// Servo interlock connection check
	if(Servo_Interlock == 0)
		monitor.servoInterlockErrorCounter++;
	else
		monitor.servoInterlockErrorCounter = 0;
		
	if(monitor.servoInterlockErrorCounter * monitor.period_ms > SERVO_INTERLOCK_ERROR_MS)
		monitor.servoInterlockCheckOK = false;
	else
		monitor.servoInterlockCheckOK = true;
	
	// THIS CODE SECTION IS UNTESTED
	// Servo supply check
//	if(Redundancy_Supply_OK == 0)
//		monitor.servoSupplyCheckErrorCounter++;
//	else
//		monitor.servoSupplyCheckErrorCounter = 0;
//	
//	if(monitor.servoSupplyCheckErrorCounter * monitor.period_ms > SERVO_SUPPLY_CHECK_ERROR_MS)
//		monitor.servoSupplyCheckOK = false;
//	else
//		monitor.servoSupplyCheckOK = true;
	
	monitor.allOK = monitor.tankPressureOK && monitor.brakePressureOK && monitor.servoInterlockCheckOK && monitor.apuTimeoutOK;
	return monitor.allOK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		tank_pressure = (float)adcValue[0];
		Vadc = adcValue[0] * 3.3 / 4095.0;
		Vout = (3 * Vadc) / 2;
		tank_pressure = (Vout - 1) / 0.4;		// [bar] - SMC pressure sensor's transfer function: V = 0.4 * P + 1
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)  	// CAN1 Interrupt
	{
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msgHeaderRx, rxData))		// Read received message
			Error_Handler();
		if(msgHeaderRx.StdId == CAN_MCU_DASH_BRAKE_FRAME_ID && msgHeaderRx.DLC == CAN_MCU_DASH_BRAKE_LENGTH)
		{
			message_arrived++;		// Debugging variable to see if there are incoming messages from CAN
			can_mcu_dash_brake_unpack((struct can_mcu_dash_brake_t*)&canBrakePressureStruct, rxData, msgHeaderRx.DLC);
			monitor.brakePressureTimeoutCounter = 0;
			canBrakePressureStruct.brake_pressure /= 100.0;
		}
		else if(msgHeaderRx.StdId == CAN_MCU_APU_STATE_MISSION_FRAME_ID && msgHeaderRx.DLC == CAN_MCU_APU_STATE_MISSION_LENGTH)
		{
			message_arrived++;
			can_mcu_apu_state_mission_unpack((struct can_mcu_apu_state_mission_t*)&canStateMissionStruct, rxData, msgHeaderRx.DLC);
			monitor.apuTimeoutCounter = 0;
		}
		else if(msgHeaderRx.StdId == CAN_MCU_ECU_BOOLS_FRAME_ID && msgHeaderRx.DLC == CAN_MCU_ECU_BOOLS_LENGTH)
		{
			can_mcu_ecu_bools_unpack((struct can_mcu_ecu_bools_t*)&canECUBoolsStruct, rxData, msgHeaderRx.DLC);
			if(servoStatus == SERVICE_BRAKE_AVAILABLE)
				servoCommandReceived = true;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		
	if (htim->Instance == TIM2)							// Timer callback used for continuous monitoring
	{
		if (monitoring == true)
		{
			continuousMonitoring();
			if (continuousMonitoring() ==  false)
			{
				if (watchdog_running != false)
				{
					watchdog_running = false;
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);			// Stop watchdog PWM
					//servoStatus = SERVICE_BRAKE_ENGAGED;					// Servo engaged
				}
			}
			else
			{
				if (watchdog_running != true)
				{
					watchdog_running = true;
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);			// Start watchdog PWM
//					servoStatus == SERVICE_BRAKE_DISENGAGED					// Servo disengaged
				}
			}
		}
	}
	else if(htim->Instance == TIM4)											// HAL_Delay timer callback
		delay_counter++;
	else if(htim->Instance == TIM5)  										// Read/write digital inputs/outputs, control servo based on status and check EBS state
	{
		// Digital inputs
		ASMS_out = (uint8_t)HAL_GPIO_ReadPin(ASMS_out_GPIO_Port, ASMS_out_Pin);
		TSMS_out_NOT = (uint8_t)HAL_GPIO_ReadPin(TSMS_out_NOT_GPIO_Port, TSMS_out_NOT_Pin);
		AS_relay_signal_buffered = (uint8_t)HAL_GPIO_ReadPin(AS_relay_signal_buffered_GPIO_Port, AS_relay_signal_buffered_Pin);
		K2_RES = (uint8_t)HAL_GPIO_ReadPin(K2_RES_GPIO_Port, K2_RES_Pin);
		K3_RES = (uint8_t)HAL_GPIO_ReadPin(K3_RES_GPIO_Port, K3_RES_Pin);
		EBS_is_armed = (uint8_t)HAL_GPIO_ReadPin(EBS_is_armed_GPIO_Port, EBS_is_armed_Pin);
		EBS_activated = (uint8_t)HAL_GPIO_ReadPin(EBS_activated_GPIO_Port, EBS_activated_Pin);
		AS_finished = (uint8_t)HAL_GPIO_ReadPin(AS_finished_GPIO_Port, AS_finished_Pin);
		Error = (uint8_t)HAL_GPIO_ReadPin(Error_GPIO_Port, Error_Pin);
		Redundancy_Supply_OK = (uint8_t)HAL_GPIO_ReadPin(Redundancy_Supply_OK_GPIO_Port, Redundancy_Supply_OK_Pin);
		Servo_Interlock = (uint8_t)HAL_GPIO_ReadPin(Servo_Interlock_GPIO_Port, Servo_Interlock_Pin);
		
		// Servo control
		if(servoStatus == SERVICE_BRAKE_DISENGAGED)			// If the status is disengaged then set the servo to disengaged position
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, SERVO_0_PERCENT_PWM_VALUE);
		else if(servoStatus == SERVICE_BRAKE_ENGAGED)		// If the status is engaged then set the servo to engaged position
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, SERVO_ENGAGED_POS_PWM_VALUE);
		else if(servoCommandReceived)										// If the status is not disengaged or engaged and a command has been received then activate service brake
		{
			if(canECUBoolsStruct.servo_commanded > 0)
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, SERVO_ACTIVATED_POS_PWM_VALUE);
			else if(canECUBoolsStruct.servo_commanded > 0 && canStateMissionStruct.as_mission == CAN_MCU_APU_STATE_MISSION_AS_MISSION_ACCELERATION_CHOICE)
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, SERVO_ACTIVATED_POS_PWM_VALUE_ACCEL);
			else
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, SERVO_0_PERCENT_PWM_VALUE);
			servoCommandReceived = false;
		}
		
		//If servo engaged or EBS Triggered, ASB Active = true, else false
		//Servo brake by ECU in AS READY
		
		// EBS state read
		if(tank_pressure < TANK_PRESSURE_THRESHOLD){			// If pressure is not high enough then the EBS is unavailable
			ebsState = EBS_UNAVAILABLE;
			if(ASMS_out == 1)														// If ASMS is on, service brake should be engaged 
				servoStatus = SERVICE_BRAKE_ENGAGED;
			else																				// If ASMS is off, then brakes should be released (Turning off ASMS will cut the supply to servo and the brakes will be released)
				servoStatus = SERVICE_BRAKE_DISENGAGED;

		}
		else
		{
			if((ASMS_out == 0) || (TSMS_out_NOT == 1) || (Error == 0) || (Brake_engage_NOT == 0))
				ebsState = EBS_TRIGGERED;										// If ASMS (ASMS_out) or RES2 (TSMS_out_NOT) or EBS Relay (TSMS_out_NOT) or Mosfet power stage (Brake_Engage) is open then the EBS is activated
			else
				ebsState = EBS_ARMED;												// If none of the above is not true, then the EBS is armed
		}
	}
	else if(htim->Instance == TIM7)				// Timer callback used to transmit CAN Message
	{
		canTxStruct.asms_state = (ASMS_out == 0) ? 0 : 1;
		canTxStruct.tsms_out = (TSMS_out_NOT == 0) ? 1 : 0;
		canTxStruct.asb_led = (ASB_LED_visibleCheckFlag || ((EBS_activated == true) && isMissionSelectedAutonomous())) ? 1 : 0; 
		canTxStruct.initial_checked = (initialChecked == false) ? 0 : 1;
		canTxStruct.service_brake_status = servoStatus;
		canTxStruct.ebs_status = ebsState;
		canTxStruct.initial_check_step = initialCheckStep;
		canTxStruct.monitor_tank_pressure = monitor.tankPressureOK;
		canTxStruct.monitor_brake_pressure = monitor.brakePressureOK;
		canTxStruct.monitor_servo_check = monitor.servoTransferFunctionCheckOK && monitor.servoSupplyCheckOK && monitor.servoInterlockCheckOK;
		canTxStruct.monitor_apu = monitor.apuTimeoutOK;
		canTxStruct.ebs_tank_pressure = tank_pressure * 100;
		can_mcu_asb_pack(txData, &canTxStruct, sizeof(txData));
			
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);								// Wait until a mailbox is available
		HAL_CAN_AddTxMessage(&hcan1, &asbStatusMsgTx, txData, &txMailbox);	// Write data to the first free mailbox and activate the corresponding transmission request
	}
	else if(htim->Instance == TIM12)																			// Timer callback used for ASB LED visible check on startup
	{
		if(ASB_LED_visibleCheckFlag == true)
			ASB_LED_visibleCheckFlag_counter++;
		if(ASB_LED_visibleCheckFlag_counter >= ASB_LED_INITIAL_CHECK_MS)
		{
			ASB_LED_visibleCheckFlag = false;
			HAL_TIM_Base_Stop_IT(&htim12);
		}
	}
}

// Incoming message from ECU: AS_mode == 1 means running WITH DRIVER. AS_mode == 0 means running DRIVERLESS.
bool isMissionSelectedAutonomous()
{
	if((canECUBoolsStruct.as_mode == false) && (canStateMissionStruct.as_mission >= CAN_MCU_APU_STATE_MISSION_AS_MISSION_ACCELERATION_CHOICE) && (canStateMissionStruct.as_mission <= CAN_MCU_APU_STATE_MISSION_AS_MISSION_INSPECTION_CHOICE))
		return true;
	else
		return false;
}

bool isMissionSelectedManual()
{
	if((canECUBoolsStruct.as_mode == true) || canStateMissionStruct.as_mission == CAN_MCU_APU_STATE_MISSION_AS_MISSION_MANUAL_DRIVING_CHOICE)
		return true;
	else
		return false;
}

void supervisorInit()
{
	//Servo calibration (EBS must be unavailable to calibrate servo)
//	for(i = 1800 ; i > 450 ; i -= 100){
//		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, i);
//		HAL_Delay(200);
//		if(canBrakePressureStruct.brake_pressure == 0){
////			SERVO_0_PERCENT_PWM_VALUE = i;
////			SERVO_100_PERCENT_PWM_VALUE = i + 500;
//			return;
//		}
//	}
	
	variablesInit();
	
	// CAN Init section
	CAN_initialize();														// Initialize parameters
	if(HAL_CAN_Start(&hcan1) != HAL_OK)					// Start CAN
		Error_Handler();
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)	// Activate message receive irq
		Error_Handler();
	
	// Begin Timer Interrupts
	HAL_TIM_Base_Start_IT(&htim5);							// Start GPIO control, servo control and EBS state check timer
	HAL_TIM_Base_Start_IT(&htim12);							// Start ASB LED visible check timer
	HAL_TIM_Base_Start_IT(&htim7);							// Start CAN transmit timer
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);		// Start toggling watchdog
	HAL_TIM_Base_Start(&htim8);									// Start ADC conversion timer
//	if (isMissionSelectedAutonomous())					// Wait for APU to boot and send mission selected from AMI
		HAL_TIM_Base_Start_IT(&htim2);						// Start continuous monitoring timer
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, SERVO_MIN_POS_PWM_VALUE);		// Set servo's PWM duty cycle to match min pos
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);																// then enable PWM generation
	
}

void variablesInit()
{
	delay_counter = 0;
	Vadc = 0;
	Vout = 0;
	state = 0;
	initialCheckStep = -1;

	monitor.period_ms = (uint16_t)(1000 / ((2*HAL_RCC_GetPCLK1Freq() / ((uint32_t)htim2.Init.Prescaler+1)) / ((uint32_t)htim2.Init.Period+1)));
	monitor.allOK = true;
	monitor.watchdogInitialCheckBypass = false;
	monitor.tankPressureOK = true;
	monitor.tankPressureErrorCounter = 0;
	monitor.tankPressureTimeoutCounter = 0;
	monitor.brakePressureOK = true;
	monitor.brakePressureErrorCounter = 0;
	monitor.brakePressureTimeoutCounter = 0;
	monitor.servoTransferFunctionCheckOK = true;
	monitor.servoTransferFunctionErrorCounter = 0;
	monitor.servoInterlockCheckOK = true;
	monitor.servoInterlockErrorCounter = 0;
	monitor.servoSupplyCheckOK = true;
	monitor.servoSupplyCheckErrorCounter = 0;
	
	monitor.apuTimeoutOK = true;
	monitor.apuTimeoutCounter = 0;
	
	initialChecked = false;
	servoStatus = SERVICE_BRAKE_DISENGAGED;
	
	ASB_LED_visibleCheckFlag = true;
	ASB_LED_visibleCheckFlag_counter = 0;

	canStateMissionStruct.as_mission = CAN_MCU_APU_STATE_MISSION_AS_MISSION_NO_MISSION_CHOICE;
	canStateMissionStruct.as_state = CAN_MCU_APU_STATE_MISSION_AS_STATE_AS_OFF_CHOICE;

	ASMS_out = 0;
	TSMS_out_NOT = 1;
	AS_relay_signal_buffered = 0;
	
	K2_RES = 0;
	K3_RES = 0;
	EBS_is_armed = 0;
	AS_finished = 0;
	Error = 0;
	
	Redundancy_Supply_OK = 1;
	Servo_Interlock = 1;
	
	Brake_engage_NOT = 1;
	HAL_GPIO_WritePin(Brake_engage_NOT_GPIO_Port, Brake_engage_NOT_Pin, GPIO_PIN_SET);
	EBS_set_armed = 0;
	HAL_GPIO_WritePin(EBS_set_armed_GPIO_Port, EBS_set_armed_Pin, GPIO_PIN_RESET);
	AS_driving_mode = 0;
	HAL_GPIO_WritePin(AS_driving_mode_GPIO_Port, AS_driving_mode_Pin, GPIO_PIN_RESET);
	Set_finish_state = 0;
	HAL_GPIO_WritePin(Set_finish_state_GPIO_Port, Set_finish_state_Pin, GPIO_PIN_RESET);
	AS_close_SDC = 0;
	HAL_GPIO_WritePin(AS_close_SDC_GPIO_Port, AS_close_SDC_Pin, GPIO_PIN_RESET);

	tank_pressure = 0;
	
	for(int i = 0; i < 8; i++)
	{
		txData[i] = 0;
		rxData[i] = 0;
	}
}

void CAN_initialize()
{
	// Initialize sent message header
	asbStatusMsgTx.StdId 							= CAN_MCU_ASB_FRAME_ID;
	asbStatusMsgTx.IDE 	 							= CAN_MCU_ASB_IS_EXTENDED;
	asbStatusMsgTx.RTR 	 							= CAN_RTR_DATA;
	asbStatusMsgTx.DLC 	 							= CAN_MCU_ASB_LENGTH;
	asbStatusMsgTx.TransmitGlobalTime = DISABLE;
	
	// CAN filter settings
	CAN_FilterTypeDef filterConfig;
	filterConfig.FilterIdHigh 				= 0x0;
	filterConfig.FilterIdLow 					= 0x0;
	filterConfig.FilterMaskIdHigh			= 0x0;
	filterConfig.FilterMaskIdLow			= 0x0;
	filterConfig.FilterFIFOAssignment	= CAN_RX_FIFO0;
	filterConfig.FilterBank						= 0;
	filterConfig.FilterMode						= CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale					= CAN_FILTERSCALE_16BIT;
	filterConfig.FilterActivation			= ENABLE;
	filterConfig.SlaveStartFilterBank	= 0;
	if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK)
		Error_Handler();
}

void delay_ms(uint32_t millis)
{
	delay_counter = 0;														// Start counting timer ticks from zero
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);		// Clear timer interrupt
	HAL_TIM_Base_Start_IT(&htim4);								// activate TIM4 timmer
	while(delay_counter < millis)									// wait till number of 100ms passed
		__WFI();
	HAL_TIM_Base_Stop_IT(&htim4);									// deactivate TIM4 timer
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //__disable_irq();
	for(int i = 0; i < 5; i++)
	{
		HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_SET);
		delay_ms(500);
		HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);
		delay_ms(500);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
