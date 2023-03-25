/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*********************************************************************************
								Definitions
**********************************************************************************/
// HC-SR04 definitions
#define TRIG_PIN				GPIO_PIN_15
#define TRIG_PORT				GPIOC
#define STOP_RANGE_HCSR			70 				// if HC-SR04 sees this (in cms), stop the motors

#define HCSR1_timer_handler		htim1
#define HCSR1_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR1_TIM_IT_CC			TIM_IT_CC1		// Timer Capture Compare 1 Interrupt
#define HCSR1_TIMER_ADDRESS		TIM1			// Base Address of HCSR1's Timer

#define HCSR2_timer_handler		htim2
#define HCSR2_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR2_TIM_IT_CC			TIM_IT_CC1
#define HCSR2_TIMER_ADDRESS		TIM2

#define HCSR3_timer_handler		htim2
#define HCSR3_TIMER_CHANNEL		TIM_CHANNEL_2
#define HCSR3_TIM_IT_CC			TIM_IT_CC2
#define HCSR3_TIMER_ADDRESS		TIM2

#define HCSR4_timer_handler		htim14
#define HCSR4_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR4_TIM_IT_CC			TIM_IT_CC1
#define HCSR4_TIMER_ADDRESS		TIM14

#define HCSR5_timer_handler		htim17
#define HCSR5_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR5_TIM_IT_CC			TIM_IT_CC1
#define HCSR5_TIMER_ADDRESS		TIM17

// HC-SR04 Enables (change to 0 to disable. Else, 1.)
#define HCSR1_EN	1
#define HCSR2_EN	1
#define HCSR3_EN	1
#define HCSR4_EN	1
#define HCSR5_EN	1

// Motor Controller (PWM) definitions
#define pwm_timer_handler 			htim3
#define PWM_LEFT_MOTOR_CHANNEL 		TIM_CHANNEL_1
#define PWM_RIGHT_MOTOR_CHANNEL 	TIM_CHANNEL_2

// Motor Enables (change to 0 to disable. Else, 1.)
#define MOTOR_EN	1

// AoA thresholds
#define LEFT_LIMIT 	-160 // If the tag is more left than this angle, turn
#define RIGHT_LIMIT  160 // If the tag is more right than this angle, turn

// AoA Macros
#define IS_VALID_ANGLE(_angle) (((_angle) >= -179) && ((_angle) <= 179))
#define ISTURN(_angle) (((_angle) > LEFT_LIMIT) && ((_angle) < RIGHT_LIMIT))
#define ISLEFT(_angle) (((_angle) > LEFT_LIMIT) && ((_angle) < 0))
#define ISRIGHT(_angle) (((_angle) >= 0) && ((_angle) < RIGHT_LIMIT))
#define ISSTOP(_hcsr_dist_1, _hcsr_dist_2, _hcsr_dist_3, _hcsr_dist_4, _hcsr_dist_5) (((_hcsr_dist_1) < STOP_RANGE_HCSR) || ((_hcsr_dist_2) < STOP_RANGE_HCSR) || ((_hcsr_dist_3) < STOP_RANGE_HCSR) || ((_hcsr_dist_4) < STOP_RANGE_HCSR) || ((_hcsr_dist_5) < STOP_RANGE_HCSR))

// AoA Enables (change to 0 to disable. Else, 1.)
#define AOA_EN	1

// AoA Error Handling (Important)
#define ANGLE_SAMPLING_HALTED_COUNT 4 // If the angle is the same for 4 consecutive times in the super loop, assume that UART sampling is halted and request it again

// AoA USART defines
#define AOA_USART_NUM_BYTES 2 // Change to 3 if adding AoA distance into USART
#define aoa_buffer			rx

/*********************************************************************************
								Global Variables
**********************************************************************************/
// State machine variables
// The following 2 variables are the commands to stop and turn the DOLL-E.
// They indicate that DOLL-E should turn, but not that it is turning yet.
uint8_t stop = 1;
uint8_t turn = 0;

// The following 4 states actually represent the current state of DOLL-E (what it is currently doing).
uint8_t stopped = 1;
uint8_t turningLeft = 0;
uint8_t turningRight = 0;
uint8_t movingStraight = 0;

// AoA variables
int16_t angle = 178; // Angle of asset tag (azimuth) relative to the antenna board
int16_t temp_angle; // For checking the validity of angle before assigning it to variable "angle"
uint8_t rx[AOA_USART_NUM_BYTES]; // Buffer to receive the angle and distance from Pi

// AoA Error Handling variables (IMPORTANT)
uint8_t angleCounter = 0; // Counts how many times the angle hasn't been updated in the while loop
int16_t prevAngle = 0;    // Holds the previous angle value

// HC-SR04 variables
uint32_t IC_VAL1_HCSR1 = 0; // Rising edge time stamp of HCSR1's ECHO pin
uint32_t IC_VAL2_HCSR1 = 0; // Falling edge time stamp of HCSR1's ECHO pin

uint32_t IC_VAL1_HCSR2 = 0;
uint32_t IC_VAL2_HCSR2 = 0;

uint32_t IC_VAL1_HCSR3 = 0;
uint32_t IC_VAL2_HCSR3 = 0;

uint32_t IC_VAL1_HCSR4 = 0;
uint32_t IC_VAL2_HCSR4 = 0;

uint32_t IC_VAL1_HCSR5 = 0;
uint32_t IC_VAL2_HCSR5 = 0;

uint32_t Difference_1 = 0;  // Difference in time between rising and falling edge of HCSR1's ECHO pin
uint32_t Difference_2 = 0;  // Difference in time between rising and falling edge of HCSR2's ECHO pin
uint32_t Difference_3 = 0;
uint32_t Difference_4 = 0;
uint32_t Difference_5 = 0;

uint8_t Is_First_Captured_1 = 0;  // Indicates whether the rising edge is captured for HCSR1's ECHO pin
uint8_t Is_First_Captured_2 = 0;  // Indicates whether the rising edge is captured for HCSR2's ECHO pin
uint8_t Is_First_Captured_3 = 0;
uint8_t Is_First_Captured_4 = 0;
uint8_t Is_First_Captured_5 = 0;

uint16_t HCSR_Distance_1  = 101; // HCSR04 Distance Sensor #1 output
uint16_t HCSR_Distance_2  = 101; // HCSR04 Distance Sensor #2 output
uint16_t HCSR_Distance_3  = 101; // HCSR04 Distance Sensor #3 output
uint16_t HCSR_Distance_4  = 101; // HCSR04 Distance Sensor #4 output
uint16_t HCSR_Distance_5  = 101; // HCSR04 Distance Sensor #5 output

// USART variables
HAL_StatusTypeDef USART_State = HAL_ERROR;

/*********************************************************************************
								Macro Functions Begin
**********************************************************************************/
#define HCSR_INPUT_HANDLE(_htim, _HCSR_TIMER_ADDRESS, _HCSR_TIMER_CHANNEL, _HCSR_TIM_IT_CC, _Is_First_Captured, _IC_VAL1_HCSR, _IC_VAL2_HCSR, _Difference, _HCSR_Distance) \
		if (_htim->Instance == _HCSR_TIMER_ADDRESS) { \
				if (_Is_First_Captured == 0) { \
					/* Take the time stamp of when the rising edge occurs */ \
					_IC_VAL1_HCSR = HAL_TIM_ReadCapturedValue(_htim, _HCSR_TIMER_CHANNEL); \
					_Is_First_Captured = 1; /* set the first captured as true */ \
					/* Now change the polarity to falling edge to be able to catch it when it happens */ \
					__HAL_TIM_SET_CAPTUREPOLARITY(_htim, _HCSR_TIMER_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING); \
				} \
				else if (_Is_First_Captured == 1) { \
					/* Take the time stamp of when the falling edge occurs */ \
					_IC_VAL2_HCSR = HAL_TIM_ReadCapturedValue(_htim, _HCSR_TIMER_CHANNEL); \
		            /* __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter, kept here just in case */ \
					if (_IC_VAL2_HCSR > _IC_VAL1_HCSR) { \
						_Difference = _IC_VAL2_HCSR - _IC_VAL1_HCSR; \
					} \
					else if (_IC_VAL1_HCSR > _IC_VAL2_HCSR) { \
						_Difference = (0xffff - _IC_VAL1_HCSR) + _IC_VAL2_HCSR; \
					} \
					_HCSR_Distance = _Difference * .034/2; \
					_Is_First_Captured = 0; /* set it back to false */ \
					/* set polarity to rising edge */ \
					__HAL_TIM_SET_CAPTUREPOLARITY(_htim, _HCSR_TIMER_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING); \
					/* Disable the interrupt after the distance calculation is done (or the noise can still pull the line up and down and cause interrupts) */ \
					__HAL_TIM_DISABLE_IT(_htim, _HCSR_TIM_IT_CC); \
				} \
			}
/*********************************************************************************
								Macro Functions End
**********************************************************************************/

void delay_in_us(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	while (__HAL_TIM_GET_COUNTER(&htim14) < time);
}

/*********************************************************************************
								HC-SR04 Functions Begin
**********************************************************************************/
// Timer Input Capture Interrupt Callback for HC-SR04
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
#if HCSR1_EN
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Timer 1 channel 1
	{
		HCSR_INPUT_HANDLE(htim, HCSR1_TIMER_ADDRESS, HCSR1_TIMER_CHANNEL, HCSR1_TIM_IT_CC,
					      Is_First_Captured_1, IC_VAL1_HCSR1, IC_VAL2_HCSR1, Difference_1,
					      HCSR_Distance_1);
	}
#endif
#if HCSR2_EN
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Timer 2 channel 1
	{
		HCSR_INPUT_HANDLE(htim, HCSR2_TIMER_ADDRESS, HCSR2_TIMER_CHANNEL, HCSR2_TIM_IT_CC,
						  Is_First_Captured_2, IC_VAL1_HCSR2, IC_VAL2_HCSR2, Difference_2,
						  HCSR_Distance_2);
	}
#endif
#if HCSR3_EN
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // Timer 2 channel 2
	{
		HCSR_INPUT_HANDLE(htim, HCSR3_TIMER_ADDRESS, HCSR3_TIMER_CHANNEL, HCSR3_TIM_IT_CC,
						  Is_First_Captured_3, IC_VAL1_HCSR3, IC_VAL2_HCSR3, Difference_3,
						  HCSR_Distance_3);
	}
#endif
#if HCSR4_EN
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Timer 14 channel 1
	{
		HCSR_INPUT_HANDLE(htim, HCSR4_TIMER_ADDRESS, HCSR4_TIMER_CHANNEL, HCSR4_TIM_IT_CC,
						  Is_First_Captured_4, IC_VAL1_HCSR4, IC_VAL2_HCSR4, Difference_4,
						  HCSR_Distance_4);
	}
#endif
#if HCSR5_EN
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Timer 17 channel 1
	{
		HCSR_INPUT_HANDLE(htim, HCSR5_TIMER_ADDRESS, HCSR5_TIMER_CHANNEL, HCSR5_TIM_IT_CC,
						  Is_First_Captured_5, IC_VAL1_HCSR5, IC_VAL2_HCSR5, Difference_5,
						  HCSR_Distance_5);
	}
#endif
}

// Pulls the trigger pin high for 10us to start the calculation
void HCSR04_Read (void)
{
#if (HCSR1_EN || HCSR2_EN || HCSR3_EN || HCSR4_EN || HCSR5_EN)
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
#endif

	// Enable the interrupt for capturing how long the ECHO pin stays high in order to calculate the distance
#if HCSR1_EN
	__HAL_TIM_ENABLE_IT(&HCSR1_timer_handler, HCSR1_TIM_IT_CC);
#endif
#if HCSR2_EN
	__HAL_TIM_ENABLE_IT(&HCSR2_timer_handler, HCSR2_TIM_IT_CC);
#endif
#if HCSR3_EN
	__HAL_TIM_ENABLE_IT(&HCSR3_timer_handler, HCSR3_TIM_IT_CC);
#endif
#if HCSR4_EN
	__HAL_TIM_ENABLE_IT(&HCSR4_timer_handler, HCSR4_TIM_IT_CC);
#endif
#if HCSR5_EN
	__HAL_TIM_ENABLE_IT(&HCSR5_timer_handler, HCSR5_TIM_IT_CC);
#endif
}
/*********************************************************************************
								HC-SR04 Functions End
**********************************************************************************/

/*********************************************************************************
						Interrupt Callbacks (ISR) Begin
**********************************************************************************/

// UART transmit callback: gets called as soon as STM32 transmits through UART
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   // do nothing here
}

// This is for preventing the edge case where if the user is standing in the threshold of left and right
// (around 0 in this case), then DOLL-E will constantly move left and right. This buffer will act based on
// previous movements. This will make the new angle output relative the recent angle of the tag.
#define ANGLE_BUFF_SIZE 5
int16_t angle_buff[ANGLE_BUFF_SIZE] = {179}; // Ring buffer (replaces the oldest angle with the new output at each iteration)
uint8_t angle_buff_index = 0;

// UART receive callback: gets called as soon as STM32 receives through UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// parse angle from rx buffer
	temp_angle = ((int16_t *)rx)[0];

	// If the angle is valid, then assign it to the global variable
	if (IS_VALID_ANGLE(temp_angle))
	{
		angle = temp_angle;
	}
	else // Else, stop the motors and stop the turning and set the global variable so that DOLL-E thinks it is facing the user.
	{
		stop = 1;
		turn = 0;
		angle = 179;
	}

	// NOTE FOR SELF: if communication stops while turning or moving forward, make an error handling to stop motors.
	// Maybe a timer of 5 seconds -> if within 5 seconds no UART receive, then stop motors

// _____________________________________ RELATIVE RECENT ANGLE CALCULATION ____________________________________
// UNCOMMENT THE FOLLOWING BOX WHEN POSSIBLE. IT APPLIES AVERAGING OF THE RECENT ANGLES AND GETS RID OF THE
// EDGE CASE WHERE IF THE USER IS STANDING IN THE THRESHOLD OF LEFT AND RIGHT (AROUND 0 IN OUR CASE), THEN
// DOLL-E WILL KEEP MOVING LEFT AND RIGHT WITHOUT BEING ABLE TO FACE THE USER. IF YOU UNCOMMENT THIS BOX,
// MAKE SURE TO COMMENT THE LINE WHERE "angle" IS BEING ASSIGNED.
/**************************************************************************************************************

	// insert angle into angle buffer
	angle_buff[angle_buff_index] = IS_VALID_ANGLE(temp_angle) ? temp_angle : 179; // if not valid, stop DOLL-E
	angle_buff_index = (angle_buff_index + 1) % ANGLE_BUFF_SIZE; // update in a ring buffer style

	// calculate output angle based on recent relative angle of the tag
	uint8_t i = 0;
	int16_t recent_angles_sum = 0;
	for (i = 0; i < ANGLE_BUFF_SIZE; i++)
	{
		recent_angles_sum += angle_buff[i];
	}

	// Round angle efficiently based on its range [-179, 179]. It is ensured through error
	// handling that the average angle calculated will be within the valid angle range.
	// Source of the rounding formula: https://www.cs.cmu.edu/~rbd/papers/cmj-float-to-int.html
	angle = (((int16_t) (((float)recent_angles_sum/ANGLE_BUFF_SIZE) + 179.5f)) - 179);

**************************************************************************************************************/

	// call the interrupt function again to keep the interrupts going
	USART_State = HAL_UART_Receive_IT(&huart1, aoa_buffer, AOA_USART_NUM_BYTES);
}

// Callback for when you have a constant timer interrupt after a while (e.g. Distance sensor reading every 0.5 seconds)
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//
//}
/*********************************************************************************
						Interrupt Callbacks (ISR) End
**********************************************************************************/

/*********************************************************************************
					   Motor Controller Functions Begin
**********************************************************************************/
void speed(int l, int r) // range from -250 to 250
{
	int set_l = (l) + 750;
	if (l == 0)
		set_l = 0;

	int set_r = (r) + 750;
	if (r == 0)
		set_r = 0;

	__HAL_TIM_SET_COMPARE(&pwm_timer_handler, PWM_LEFT_MOTOR_CHANNEL, set_l);
	__HAL_TIM_SET_COMPARE(&pwm_timer_handler, PWM_RIGHT_MOTOR_CHANNEL, set_r);
}

// Input: Left wheel absolute speed
void turnLeft(int speed_val)
{
	// Less than 20 is there because the friction on the left wheel is more.
	if (speed_val < 20 || speed_val > 250)
	{
		speed(0, 0);
		return;
	}

	turningLeft = 1;
	turningRight = 0;
	stopped = 0;
	movingStraight = 0;
	speed(-speed_val, speed_val - 20);
}

// Input: Left wheel absolute speed (since left wheel has the larger absolute value at all times)
void turnRight(int speed_val)
{
	// Less than 20 is there because the friction on the left wheel is more.
	if (speed_val < 20 || speed_val > 250)
	{
		speed(0, 0);
		return;
	}

	turningRight = 1;
	turningLeft = 0;
	stopped = 0;
	movingStraight = 0;
	speed(speed_val, -(speed_val - 20));
}

// Input: Left wheel absolute speed
void goStraight(int speed_val)
{
	// Less than 20 is there because the friction on the left wheel is more.
	if (speed_val <= 20 || speed_val > 250)
	{
		speed(0, 0);
		return;
	}

	turningLeft = 0;
	turningRight = 0;
	stopped = 0;
	movingStraight = 1;
	speed(speed_val, speed_val - 5);
}

void turnDOLLE(void)
{
	// TURN TOWARDS THE USER UNTIL THE USER IS IN FRONT OF DOLL-E
	do
	{
		if (ISLEFT(angle))
		{
			turnLeft(50);
        }
	    else if(ISRIGHT(angle))
	    {
	   	    turnRight(50);
		}
	    else
	    {
	 	    speed(0, 0);
		}
	    HAL_Delay(40);
	} while(ISTURN(angle));
}
/*********************************************************************************
					   Motor Controller Functions End
**********************************************************************************/

/*********************************************************************************
					   Error Handling Functions Begin
**********************************************************************************/
// Reinitializes the AOA USART
void reinitalize_aoa_UART()
{
	HAL_UART_DeInit(&huart1); // Deinitialize USART1
    MX_USART1_UART_Init();    // Reinitialize USART1

    // Call UART interrupt to sample again before it tries to deinitialize again.
    // If it doesn't work, give some delay after it to wait for the new angle to arrive.
    USART_State = HAL_UART_Receive_IT(&huart1, aoa_buffer, AOA_USART_NUM_BYTES);
}
/*********************************************************************************
					   Error Handling Functions End
**********************************************************************************/

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // ********** Timer Interrupts Start for HC-SR04 Distance Sensor **********
#if HCSR1_EN
  HAL_TIM_IC_Start_IT(&HCSR1_timer_handler, HCSR1_TIMER_CHANNEL);
#endif
#if HCSR2_EN
  HAL_TIM_IC_Start_IT(&HCSR2_timer_handler, HCSR2_TIMER_CHANNEL);
#endif
#if HCSR3_EN
  HAL_TIM_IC_Start_IT(&HCSR3_timer_handler, HCSR3_TIMER_CHANNEL);
#endif
#if HCSR4_EN
  HAL_TIM_IC_Start_IT(&HCSR4_timer_handler, HCSR4_TIMER_CHANNEL);
#endif
#if HCSR5_EN
  HAL_TIM_IC_Start_IT(&HCSR5_timer_handler, HCSR5_TIMER_CHANNEL);
#endif

  // ********** PWM Start for Controlling the Motors **********
#if MOTOR_EN
  HAL_TIM_PWM_Start(&pwm_timer_handler, PWM_LEFT_MOTOR_CHANNEL); // Start the PWM for left motor
  HAL_TIM_PWM_Start(&pwm_timer_handler, PWM_RIGHT_MOTOR_CHANNEL); // Start the PWM for right motor
#endif

  // ********** UART Rx Interrupt Start for Receiving Angles from Raspberry Pi **********
#if AOA_EN
  USART_State = HAL_UART_Receive_IT(&huart1, aoa_buffer, AOA_USART_NUM_BYTES);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Send the command to read from the distance sensors.
	  HCSR04_Read();

	  // Update the state.
	  stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_3, HCSR_Distance_4, HCSR_Distance_5);
	  turn = ISTURN(angle);

	  // Interpret the results to send motor commands.
	  if (turn)
	  {
		  if (ISLEFT(angle) && !turningLeft)
		  {
			  turnLeft(50);
		  }
		  else if (ISRIGHT(angle) && !turningRight)
		  {
			  turnRight(50);
		  }
	  }
	  else if (stop && !stopped)
	  {
		  speed(0, 0);
		  turningLeft = 0;
		  turningRight = 0;
		  stopped = 1;
		  movingStraight = 0;
	  }
	  else if (!stop && !movingStraight)
	  {
		  goStraight(50);
	  }

	  HAL_Delay(40); // Small delay to avoid sending commands to motors too frequently.

	  // ***** Rest of the while loop is error handling *****

	  // Count how many times we went through the while loop without angle being updated.
	  if (prevAngle == angle)
	  {
		  angleCounter++;
		  if (angleCounter == 0)
			  angleCounter = 255;
	  }
	  else
	  {
		  angleCounter = 0;
	  }

	  prevAngle = angle; // Update previous angle value

	  // If the angle has been the same value for a while, call the UART Receive interrupt again.
	  if (angleCounter >= ANGLE_SAMPLING_HALTED_COUNT)
	  {
		  USART_State = HAL_UART_Receive_IT(&huart1, aoa_buffer, AOA_USART_NUM_BYTES);
	  }

	  // If the angle is outside the valid range, restart the UART.
      if (!IS_VALID_ANGLE(temp_angle))
      {
	     reinitalize_aoa_UART();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Prescaler = 32;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim14, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 16-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_LED_Pin */
  GPIO_InitStruct.Pin = TEST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SCL_Pin PA12 */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
