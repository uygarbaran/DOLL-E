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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
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
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*********************************************************************************
								Debugging
**********************************************************************************/
//int _write(int file, char *ptr, int len)
//{
//	int i = 0;
//	for (i = 0; i < len; i++)
//		ITM_SendChar((*ptr++));
//
//	return len;
//}


/*********************************************************************************
								Definitions
**********************************************************************************/
// States in the state machine
#define STOP_STATE 	0
#define TURN_STATE 	1
#define MOVE_STATE 	2

// HC-SR04 definitions
#define TRIG_PIN 	GPIO_PIN_9
#define TRIG_PORT 	GPIOA

#define TRIG2_PIN 	GPIO_PIN_7
#define TRIG2_PORT 	GPIOC
//
#define TRIG5_PIN 	GPIO_PIN_0
#define TRIG5_PORT 	GPIOB

#define STOP_RANGE_HCSR  70 // if HC-SR04 sees 50cms or less, stop DOLL-E from moving forward (turns are still allowed)
#define STOP_RANGE_TURN  31 // If HC-SR04 sees this range or less, stops DOLL-E from turning left or right

#define HCSR1_timer_handler		htim1
#define HCSR1_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR1_TIM_IT_CC			TIM_IT_CC1		// Timer Capture Compare 1 Interrupt
#define HCSR1_TIMER_ADDRESS		TIM1			// Base Address of HCSR1's Timer

#define HCSR2_timer_handler		htim14
#define HCSR2_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR2_TIM_IT_CC			TIM_IT_CC1
#define HCSR2_TIMER_ADDRESS		TIM14

#define HCSR3_timer_handler		htim15
#define HCSR3_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR3_TIM_IT_CC			TIM_IT_CC1
#define HCSR3_TIMER_ADDRESS		TIM15

#define HCSR4_timer_handler		htim15
#define HCSR4_TIMER_CHANNEL		TIM_CHANNEL_2
#define HCSR4_TIM_IT_CC			TIM_IT_CC2
#define HCSR4_TIMER_ADDRESS		TIM15

#define HCSR5_timer_handler		htim17
#define HCSR5_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR5_TIM_IT_CC			TIM_IT_CC1
#define HCSR5_TIMER_ADDRESS		TIM17

// HC-SR04 Enables (change to 0 to disable. Else, 1.)
#define HCSR1_EN	1
#define HCSR2_EN	1

// AoA thresholds
#define LEFT_LIMIT 	-160 // If the tag is more left than this angle, turn
#define RIGHT_LIMIT  160 // If the tag is more right than this angle, turn
#define STOP_RANGE_AOA 1 // If the tag is strictly less than 1 meter, then stop
#define IS_VALID_ANGLE(_angle) (((_angle) >= -179) && ((_angle) <= 179))
#define ISTURN(_angle) (((_angle) > LEFT_LIMIT) && ((_angle) < RIGHT_LIMIT))
#define ISLEFT(_angle) (((_angle) > LEFT_LIMIT) && ((_angle) < 0))
#define ISRIGHT(_angle) (((_angle) >= 0) && ((_angle) < RIGHT_LIMIT))
//#define ISTURN(_angle) (((_angle) < LEFT_LIMIT) && ((_angle) > RIGHT_LIMIT))
//#define ISLEFT(_angle) (((_angle) < LEFT_LIMIT) && ((_angle) > 0))
//#define ISRIGHT(_angle) (((_angle) <= 0) && ((_angle) > RIGHT_LIMIT))
#define ISSTOP(_hcsr_dist_1, _hcsr_dist_2, _hcsr_dist_3) (((_hcsr_dist_1) < STOP_RANGE_HCSR) || ((_hcsr_dist_2) < STOP_RANGE_HCSR) || ((_hcsr_dist_3) < STOP_RANGE_HCSR))
#define IS_STOP_TURN(_hcsr_dist_1, _hcsr_dist_2, _hcsr_dist_3) (((_hcsr_dist_1) < STOP_RANGE_TURN) || ((_hcsr_dist_2) < STOP_RANGE_TURN) || ((_hcsr_dist_3) < STOP_RANGE_TURN))

// AoA Error Handling (Important)
#define ANGLE_SAMPLING_HALTED_COUNT 4 // If the angle is the same for ANGLE_SAMPLING_HALTED_COUNT consecutive times in the super loop, assume that UART sampling is halted and request it again

// AoA USART defines
#define AOA_USART_NUM_BYTES 2 // Change to 3 if adding AoA distance into USART

// Motor Controller PWM defines
#define FRICTION_OFFSET			4		// Right motor has different friction than the left one. This accounts for that
#define MAX_MOTOR_SPEED_USED	200 	// This is the maximum motor speed we want to reach. More than this could be too fast.

#define ABS(_ang_) ((_ang_) < 0 ? -(_ang_) : (_ang_))
#define ANGLE_TO_SPEED(__angle__) (MAX_MOTOR_SPEED_USED - ((int32_t)(ABS(__angle__))))

/*********************************************************************************
								Global Variables
**********************************************************************************/
// State machine variables
uint8_t curState = STOP_STATE;
uint8_t turningLeft = 0;
uint8_t turningRight = 0;
uint8_t stopped = 1;
uint8_t movingStraight = 0;

// AoA variables
int16_t angle = 178; // Angle of asset tag (azimuth) relative to the antenna board
uint8_t AOA_Distance = 1; // in meters
uint8_t rx[AOA_USART_NUM_BYTES]; // Buffer to receive the angle and distance from Pi
uint8_t stop = 1; // Stop from moving forward
uint8_t stopTurn = 1; // Stop from turning
uint8_t turn = 0;
uint8_t move = 0;
uint8_t aoa_uart_received = 0;

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

uint8_t Distance1_flag = 0; // These flags are set to 1 when distance is calculated.
uint8_t Distance2_flag = 0;
uint8_t Distance3_flag = 0;
uint8_t Distance4_flag = 0;
uint8_t Distance5_flag = 0;

uint16_t HCSR_Distance_1  = 101; // HCSR04 Distance Sensor #1 output
uint16_t HCSR_Distance_2  = 101; // HCSR04 Distance Sensor #2 output
uint16_t HCSR_Distance_3  = 101; // HCSR04 Distance Sensor #3 output
uint16_t HCSR_Distance_4  = 101; // HCSR04 Distance Sensor #4 output
uint16_t HCSR_Distance_5  = 101; // HCSR04 Distance Sensor #5 output

uint32_t millis = 0;

// USART variables
HAL_StatusTypeDef USART_State = HAL_ERROR;

/*********************************************************************************
								Macro Functions Begin
**********************************************************************************/
#define HCSR_INPUT_HANDLE(_htim, _HCSR_TIMER_ADDRESS, _HCSR_TIMER_CHANNEL, _HCSR_TIM_IT_CC, _Is_First_Captured, _IC_VAL1_HCSR, _IC_VAL2_HCSR, _Difference, _HCSR_Distance, _Dist_Flag) \
		if ((_htim)->Instance == (_HCSR_TIMER_ADDRESS)) { \
				if ((_Is_First_Captured) == 0) { \
					/* Take the time stamp of when the rising edge occurs */ \
					_IC_VAL1_HCSR = HAL_TIM_ReadCapturedValue((_htim), (_HCSR_TIMER_CHANNEL)); \
					_Is_First_Captured = 1; /* set the first captured as true */ \
					/* Now change the polarity to falling edge to be able to catch it when it happens */ \
					__HAL_TIM_SET_CAPTUREPOLARITY((_htim), (_HCSR_TIMER_CHANNEL), TIM_INPUTCHANNELPOLARITY_FALLING); \
				} \
				else if ((_Is_First_Captured) == 1) { \
					/* Take the time stamp of when the falling edge occurs */ \
					_IC_VAL2_HCSR = HAL_TIM_ReadCapturedValue((_htim), (_HCSR_TIMER_CHANNEL)); \
		            /* __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter, kept here just in case */ \
					if ((_IC_VAL2_HCSR) > (_IC_VAL1_HCSR)) { \
						_Difference = (_IC_VAL2_HCSR) - (_IC_VAL1_HCSR); \
					} \
					else if ((_IC_VAL1_HCSR) > (_IC_VAL2_HCSR)) { \
						_Difference = (0xffff - (_IC_VAL1_HCSR)) + (_IC_VAL2_HCSR); \
					} \
					_HCSR_Distance = (_Difference) * .034/2; \
					_Is_First_Captured = 0; /* set it back to false */ \
					_Dist_Flag = 1; /* Set the distance flag to 1 to indicate that the distance is calculated.*/ \
					/* set polarity to rising edge */ \
					__HAL_TIM_SET_CAPTUREPOLARITY((_htim), (_HCSR_TIMER_CHANNEL), TIM_INPUTCHANNELPOLARITY_RISING); \
					/* Disable the interrupt after the distance calculation is done (or the noise can still pull the line up and down and cause interrupts) */ \
					__HAL_TIM_DISABLE_IT((_htim), (_HCSR_TIM_IT_CC)); \
				} \
			}
/*********************************************************************************
								Macro Functions End
**********************************************************************************/

void delay_in_us(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < time);
}

/*********************************************************************************
								HC-SR04 Functions Begin
**********************************************************************************/
//uint8_t dist_tx_buff[3];

uint32_t intCounter = 0, intCounter2 = 0;

// Timer Interrupt Callback for HC-SR04
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // CHECK WHETHER THE PWM STUFF GETS IN HERE
{
	// We land here from the sleep mode. Resume the tick interrupt.
//	HAL_ResumeTick();
	intCounter2++;
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		intCounter++;
		if (Is_First_Captured_1 == 0)
		{
			/* Take the time stamp of when the rising edge occurs */
			IC_VAL1_HCSR1 = HAL_TIM_ReadCapturedValue(htim, HCSR1_TIMER_CHANNEL);
			Is_First_Captured_1 = 1; /* set the first captured as true */
			/* Now change the polarity to falling edge to be able to catch it when it happens */
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, HCSR1_TIMER_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured_1 == 1)
		{
			/* Take the time stamp of when the falling edge occurs */
			IC_VAL2_HCSR1 = HAL_TIM_ReadCapturedValue(htim, HCSR1_TIMER_CHANNEL);
			/* __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter, kept here just in case */
			if (IC_VAL2_HCSR1 > IC_VAL1_HCSR1)
			{
				Difference_1 = IC_VAL2_HCSR1 - IC_VAL1_HCSR1;
			}
			else if (IC_VAL1_HCSR1 > IC_VAL2_HCSR1)
			{
				Difference_1 = (0xffff - IC_VAL1_HCSR1) + IC_VAL2_HCSR1;
			}
			HCSR_Distance_1 = Difference_1 * .034/2;
			Is_First_Captured_1 = 0; /* set it back to false */
			Distance1_flag = 1; /* Set the distance flag to 1 to indicate that the distance is calculated.*/
			/* set polarity to rising edge */
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, HCSR1_TIMER_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
			/* Disable the interrupt after the distance calculation is done (or the noise can still pull the line up and down and cause interrupts) */
			__HAL_TIM_DISABLE_IT(&htim1, HCSR1_TIM_IT_CC);
		}
	}

//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Timer 1 channel 1
//	{
////		intCounter++;
//		HCSR_INPUT_HANDLE(htim, HCSR1_TIMER_ADDRESS, HCSR1_TIMER_CHANNEL, HCSR1_TIM_IT_CC,
//					      Is_First_Captured_1, IC_VAL1_HCSR1, IC_VAL2_HCSR1, Difference_1,
//					      HCSR_Distance_1, Distance1_flag);
//	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Timer 14 channel 1
	{
		HCSR_INPUT_HANDLE(htim, HCSR2_TIMER_ADDRESS, HCSR2_TIMER_CHANNEL, HCSR2_TIM_IT_CC,
						  Is_First_Captured_2, IC_VAL1_HCSR2, IC_VAL2_HCSR2, Difference_2,
						  HCSR_Distance_2, Distance2_flag);
	}
////
//////	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Timer 15 channel 1
//////	{
//////		HCSR_INPUT_HANDLE(htim, HCSR3_TIMER_ADDRESS, HCSR3_TIMER_CHANNEL, HCSR3_TIM_IT_CC,
//////						  Is_First_Captured_3, IC_VAL1_HCSR3, IC_VAL2_HCSR3, Difference_3,
//////						  HCSR_Distance_3, Distance3_flag);
//////	}
//////
//////	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // Timer 15 channel 2
//////	{
//////		HCSR_INPUT_HANDLE(htim, HCSR4_TIMER_ADDRESS, HCSR4_TIMER_CHANNEL, HCSR4_TIM_IT_CC,
//////						  Is_First_Captured_4, IC_VAL1_HCSR4, IC_VAL2_HCSR4, Difference_4,
//////						  HCSR_Distance_4, Distance4_flag);
//////	}
////
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Timer 17 channel 1
	{
		HCSR_INPUT_HANDLE(htim, HCSR5_TIMER_ADDRESS, HCSR5_TIMER_CHANNEL, HCSR5_TIM_IT_CC,
						  Is_First_Captured_5, IC_VAL1_HCSR5, IC_VAL2_HCSR5, Difference_5,
						  HCSR_Distance_5, Distance5_flag);
	}
}

// Pulls the trigger pin high for 10us to start the calculation
void HCSR04_Read (void)
{
	Distance1_flag = 0;
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	// Enable the interrupt for capturing how long the ECHO pin stays high in order to calculate the distance
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
//	__HAL_TIM_ENABLE_IT(&HCSR2_timer_handler, HCSR2_TIM_IT_CC);
////	__HAL_TIM_ENABLE_IT(&HCSR3_timer_handler, HCSR3_TIM_IT_CC);
////	__HAL_TIM_ENABLE_IT(&HCSR4_timer_handler, HCSR4_TIM_IT_CC);
//	__HAL_TIM_ENABLE_IT(&HCSR5_timer_handler, HCSR5_TIM_IT_CC);

}

// Pulls the trigger pin high for 10us to start the calculation
//void HCSR1_Read (void)
//{
//	Distance1_flag = 0;
//	HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
//	delay_in_us(10);  // wait for 10 us
//	HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
//
//	// Enable the interrupt for capturing how long the ECHO pin stays high in order to calculate the distance
//	__HAL_TIM_ENABLE_IT(&HCSR1_timer_handler, HCSR1_TIM_IT_CC);
//}
//
// Pulls the trigger pin high for 10us to start the calculation
void HCSR2_Read (void)
{
	Distance2_flag = 0;
	HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	// Enable the interrupt for capturing how long the ECHO pin stays high in order to calculate the distance
	__HAL_TIM_ENABLE_IT(&HCSR2_timer_handler, HCSR2_TIM_IT_CC);
}
//
//// Pulls the trigger pin high for 10us to start the calculation
void HCSR5_Read (void)
{
	Distance5_flag = 0;
	HAL_GPIO_WritePin(TRIG5_PORT, TRIG5_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG5_PORT, TRIG5_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	// Enable the interrupt for capturing how long the ECHO pin stays high in order to calculate the distance
	__HAL_TIM_ENABLE_IT(&HCSR5_timer_handler, HCSR5_TIM_IT_CC);
}


/*********************************************************************************
								HC-SR04 Functions End
**********************************************************************************/

void stateMachine(void)
{
	// For the first condition, here is the plan:
	// Have a timer that runs which gets reset each time you get into the HAL_UART_RxCpltCallback() ISR function.
	// If this timer exceeds a certain amount of time, this means we haven't got into that ISR function for a while.
	// If this is the case, send a command to the Pi so that it restarts the application.
	// FOR NOW, THIS WILL NOT BE IMPLEMENTED. THIS IS FOR THE FUTURE. DO THE NECESSARY PARTS FIRST.

	// If DOLL-E needs to turn, move will go to 0 but motors won't stop until stop is 1. This is why we check for !move.
//	stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2);
	turn = 0;// ISTURN(angle);
	move = (!stop && !turn);

//	if (stop)
//	{
//		// CALL THE FUNCTION TO STOP THE MOTORS
//
////		// Normally we would check the first condition. But since it is not implemented, we will straight up check the second.
////		if (1) // First condition
////		{
////			// UYGAR_LOG: THESE MIGHT NOT BE NEEDED
////			if (ISTURN(angle))
////				curState = TURN_STATE;
////		}
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//	}
//	else
//	{
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 850);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 850);
//	}

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);

//	if (turn)
//	{
//		// Call the function to turn.
//	}
//
//	// If no need to turn and no need to stop, then move.
//	if (move)
//	{
//		// CALL THE FUNCTION TO MOVE THE MOTORS FORWARD
//	}


}
//const uint8_t ack = 32; // ack in our case is a space (32)

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
int16_t temp_angle;

// UART receive callback: gets called as soon as STM32 receives through UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// parse distance of tag from the antenna board (3rd byte)
//	AOA_Distance = rx[2];

	// parse angle from rx buffer
	temp_angle = ((int16_t *)rx)[0];

	// HAL_StatusTypeDef stat = HAL_UART_Transmit(&huart2, rx, 3, 1000);

	// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	if (stat == HAL_OK)
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	// If the angle is valid, then assign it to the global variable
	if (IS_VALID_ANGLE(temp_angle))
	{
		angle = temp_angle;
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	else // Else, stop the motors and stop the turning and set the global variable so that DOLL-E thinks it is facing the user.
	{
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		stop = 1;
		turn = 0;
		angle = 179;
	}

	// CALL UART IT BACK AGAIN IN CASE A NEW ANGLE COMES
//    USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);

	// TESTING CODE

//	if (angle < -130 && angle > -140)
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

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

/******** DEBUG CODE, DELETE LATER ****************************
//	if (angle >= 90 && angle <= 100)
//	if (angle >= 40 && angle <= 50)
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	// HAL_UART_Transmit_IT(&huart2, rx, 2); // Send it over to monitor (ST-LINK)
//	HAL_UART_Transmit_IT(&huart1, &ack, 1);
//	HAL_UART_Transmit_IT(&huart1, &rx, 1);
 *
***************************************************************************/

	// UNCOMMENT THIS LATER IF NEEDED!
	// call the interrupt function again to keep the interrupts going
//	USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
}

// Distance sensor reading every 0.5 seconds
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	HCSR04_Read();
//	if (HCSR_Distance_1 < STOP_RANGE_HCSR)
//	{
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//	}
//	else
//	{
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
//	}
}
/*********************************************************************************
						Interrupt Callbacks (ISR) End
**********************************************************************************/

#define R 90
#define L -90
int16_t angle_arr[17] = {L, L, L, 0, R, R, R, 0, L, 0, R, 0, L, 0, R, 0, L};

void turnFunc(int32_t l, int32_t r) // range from -100 to 100
{
	uint32_t set_l = (int32_t)(l * 2.5) + 750;
	if (l == 0)
		set_l = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, set_l);

	uint32_t set_r = (int32_t)(r * 2.5) + 750;
	if (r == 0)
		set_r = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, set_r);
}

void speed(int32_t l, int32_t r) // range from -250 to 250
{
	if (l < -250 || l > 250 || r < -250 || r > 250)
	{
		l = 0;
		r = 0;
	}

	uint32_t set_l = (l) + 750;
	if (l == 0)
		set_l = 0;

	uint32_t set_r = (r) + 750;
	if (r == 0)
		set_r = 0;

	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, set_l);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, set_r);
}

// Input: Left wheel absolute speed
void turnLeft(int32_t speed_val)
{
	// Less than FRICTION_OFFSET is there because the friction on the left wheel is more.
	if (speed_val < FRICTION_OFFSET || speed_val > 250)
	{
		speed(0, 0);
		return;
	}

	turningLeft = 1;
	turningRight = 0;
	stopped = 0;
	movingStraight = 0;
	speed(-speed_val, speed_val - FRICTION_OFFSET);
}

//void turn_left_based_on_angle()
//{
//	int16_t
//	if (angle <)
//}

// Input: Left wheel absolute speed (since left wheel has the larger absolute value at all times)
void turnRight(int32_t speed_val)
{
	// Less than 20 is there because the friction on the left wheel is more.
	if (speed_val < FRICTION_OFFSET || speed_val > 250)
	{
		speed(0, 0);
		return;
	}

	turningRight = 1;
	turningLeft = 0;
	stopped = 0;
	movingStraight = 0;
	speed(speed_val, -(speed_val - FRICTION_OFFSET));
}

// Input: Left wheel absolute speed
void goStraight(int32_t speed_val)
{
	// Less than FRICTION_OFFSET is there because the friction on the left wheel is more.
	if (speed_val <= FRICTION_OFFSET || speed_val > 250)
	{
		speed(0, 0);
		return;
	}

	turningLeft = 0;
	turningRight = 0;
	stopped = 0;
	movingStraight = 1;
	speed(speed_val, speed_val - FRICTION_OFFSET);
}

int32_t angle_to_speed(int16_t angle_val)
{
	int32_t ang = (int32_t)angle_val; // cast to 32 bits since the functions with speed values represent speed in 32 bits.

	// If angle is negative, make it positive first.
	if (ang < 0)
	{
		ang = ang * -1;
	}

	angle_to_speed(MAX_MOTOR_SPEED_USED - ((int32_t)ang < 0 ? (int32_t)-ang : (int32_t)ang));

	return (MAX_MOTOR_SPEED_USED - ang);
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



// Reinitializes the USART used to communicate with Raspberry Pi
void reinitalize_USART1(void)
{
	HAL_UART_DeInit(&huart1); // Deinitialize USART 1
    MX_USART1_UART_Init();    // Reinitialize USART 1

    // Call UART interrupt to sample again before it tries to deinitialize again.
    // If it doesn't work, give some delay after it to wait for the new angle to arrive.
    USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
}

void makeDecision()
{
	 // Update the state.
	  stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_5);
//	  stopTurn = IS_STOP_TURN(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_5);
	  turn = ISTURN(angle);

	  // Interpret the results to send motor commands.
	  if (turn)
	  {
//		  if (stopTurn)
//		  {
//		  if (!stopped)
//		  {
//			  speed(0, 0);
//			  turningLeft = 0;
//			  turningRight = 0;
//			  stopped = 1;
//			  movingStraight = 0;
//		  }
//		  }
		  if (ISLEFT(angle) && !turningLeft) // if (ISLEFT(angle)  && !turningLeft)
		  {
			   turnLeft(50);
//			  turnLeft(ANGLE_TO_SPEED(angle));
		  }
		  else if (ISRIGHT(angle) && !turningRight) // else if (ISRIGHT(angle) && !turningRight)
		  {
			   turnRight(50);

//			  turnRight(ANGLE_TO_SPEED(angle));
		  }
	  }
	  else if (stop)
	  {
		  if (!stopped)
		  {
			  speed(0, 0);
			  turningLeft = 0;
			  turningRight = 0;
			  stopped = 1;
			  movingStraight = 0;
		  }
	  }
	  else if (!stop)
	  {
		  if (!movingStraight)
			  goStraight(80);
	  }

	  // If the angle is outside the valid range, restart the UART.
	  if (!IS_VALID_ANGLE(temp_angle))
	  {
		  reinitalize_USART1(); // GOTTA TAKE OUT THE USART IT CALL IF WE ARE CALLING IT ALREADY IN THE LOOP
	  }

	  if (stopped)
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  else
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

uint32_t time = 0;
uint64_t wCounter = 0;
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
//  dist_tx_buff[1] = '\n';
//  dist_tx_buff[2] = '\r';


  // ********** Timer Interrupts Start for HC-SR04 Distance Sensor **********
//  HAL_TIM_Base_Start_IT(&htim14); // Start the timer interrupt (called every 0.1 seconds) for distance sensor
//#if HCSR1_EN
//  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // Start the input capture for TIM1 Channel 1 interrupt for HC-SR04
//#endif
//#if HCSR2_EN
//  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3); // Start the input capture for TIM1 Channel 3 interrupt for HC-SR04
//#endif

  HAL_TIM_IC_Start_IT(&HCSR1_timer_handler, HCSR1_TIMER_CHANNEL);
  HAL_TIM_IC_Start_IT(&HCSR2_timer_handler, HCSR2_TIMER_CHANNEL);
////  HAL_TIM_IC_Start_IT(&HCSR3_timer_handler, HCSR3_TIMER_CHANNEL);
////  HAL_TIM_IC_Start_IT(&HCSR4_timer_handler, HCSR4_TIMER_CHANNEL);
  HAL_TIM_IC_Start_IT(&HCSR5_timer_handler, HCSR5_TIMER_CHANNEL);


  // ********** PWM Start for Controlling the Motors **********
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Start the PWM for left motor
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Start the PWM for right motor

  // ********** UART Rx Interrupt Start for Receiving Angles from Raspberry Pi **********
//  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES); // UYGAR_LOG: call the interrupt function for UART communication with Pi

  // ********** Timer Interrupt Start for Calling the State Machine ********** (WE MIGHT NOT DO THIS THRU INTERRUPT)
//  HAL_TIM_Base_Start_IT(&htim6); // State machine interrupt

  // ********** Timer Interrupt Start for Receiving Bytes from ST-LINK (PC) ********** (UNCOMMENT WHEN DEBUGGING UART Rx)
//   HAL_UART_Receive_IT(&huart2, &rx, 1); // UYGAR_LOG: call the interrupt function

  // UYGAR_LOG: At the beginning, check the distance to begin with maybe?


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t X = 0;
//  char MSG[100];
//  const uint8_t *rx_fail = "Rx failed\n\r";
////  const char *rx_success = "Rx succeeded\n\r";
//  uint8_t Rx_data[100];
//  Rx_data[0] = '\0';
//  uint8_t tx = 0;

  //  HAL_OK       = 0x00U,
//  HAL_ERROR    = 0x01U,
//  HAL_BUSY     = 0x02U,
//  HAL_TIMEOUT  = 0x03U
//  HAL_StatusTypeDef HAL_stat = HAL_ERROR;
//  HAL_StatusTypeDef send_stat = HAL_ERROR;
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
//  		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  // Snapshots for the volatile variables
//  uint16_t dist1 = 0, dist2 = 0, dist3 = 0, dist4 = 0, dist5 = 0;
//  int16_t angle_snapshot = 179, temp_angle_snapshot = 179;
  speed(0, 0);
//  HAL_Delay(5000);
  uint32_t HCSR_block_timeout = 75; // MAKE LARGER IF NEEDED
  uint8_t hcsr_pick = 0;
  //goStraight(240);
  while (1)
  {
//	  hcsr_pick = (hcsr_pick + 1) % 3;
//	  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);

//	  if (hcsr_pick == 0)
//	  {
//		  HCSR04_Read();
//		  millis = HAL_GetTick();
//		  while ((Distance1_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
//	  }
//	  else if (hcsr_pick == 1)
//	  {
//		  HCSR2_Read();
//		  millis = HAL_GetTick();
//		  while ((Distance2_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
//	  }
//	  else
////	  {
////		  HCSR5_Read();
////		  millis = HAL_GetTick();
////		  while ((Distance5_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
////	  }
//	  // Sets the distance flag to 0. The flag will be set to 1 when HCSR_Distance_1 is calculated.
//	  HCSR04_Read();
//	  millis = HAL_GetTick(); // Get the time in milliseconds
//
//	  // While the flag is still 0 AND the timeout hasn't been reached, keep waiting for HCSR1 output.
//	  // We can do sleep mode instead of this (gotta watch out for UART interrupting it), but for now try with polling.
//	  while ((Distance1_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
//////	  while (Distance1_flag == 0) {}
//////	  time = HAL_GetTick() - millis;
////
//	  HCSR2_Read();
//	  millis = HAL_GetTick();
//	  while ((Distance2_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
//////	  time = HAL_GetTick() - millis;
////
//	  HCSR5_Read();
//	  millis = HAL_GetTick();
//	  while ((Distance5_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
////	  time = HAL_GetTick() - millis;

//	  HAL_Delay(1000);

	  // Go into sleep mode waiting for the HCSR1 Reading. Ensure that we do not get into sleeping mode after the sampling of the distance, or we will never get out of it.
	  // Sleep while waiting for rising edge from HCSR.
////	  HAL_SuspendTick();
////	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//
//	  // After the rising edge, if the falling edge is not captured yet, then get into sleep mode again waiting for it.
//	  if (Is_First_Captured_1 == 1)
//	  {
//		  HAL_SuspendTick();
//		  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//	  }

//	  HCSR2_Read();
//	  millis = HAL_GetTick();
//	  while (Distance2_flag == 0 && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
////	  while (Distance2_flag == 0) {}
//	  time = HAL_GetTick() - millis;
//
//	  HCSR5_Read();
//	  millis = HAL_GetTick();
//	  while (Distance5_flag == 0 && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
////	  while (Distance5_flag == 0) {}
//	  time = HAL_GetTick() - millis;

	  // STRAIGHT GOING CODE

//	  HCSR04_Read();
//	  millis = HAL_GetTick();
//	  while ((Distance1_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
//
//	  stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_5);
//	  if (stop)
//	  {
//		  if (!stopped)
//		  {
//			  speed(0, 0);
//			  turningLeft = 0;
//			  turningRight = 0;
//			  stopped = 1;
//			  movingStraight = 0;
//		  }
//	  }
//	  else
//		  goStraight(200);
//
//	  HCSR2_Read();
//  	  millis = HAL_GetTick();
//	  while ((Distance2_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
//
//	  stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_5);
//	  if (stop)
//	  {
//		  if (!stopped)
//		  {
//			  speed(0, 0);
//			  turningLeft = 0;
//			  turningRight = 0;
//			  stopped = 1;
//			  movingStraight = 0;
//		  }
//	  }
//	  else
//		  goStraight(200);
//
//	  HCSR5_Read();
//	  millis = HAL_GetTick();
//	  while ((Distance5_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}
//
//	  stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_5);
//	  if (stop)
//	  {
//		  if (!stopped)
//		  {
//			  speed(0, 0);
//			  turningLeft = 0;
//			  turningRight = 0;
//			  stopped = 1;
//			  movingStraight = 0;
//		  }
//	  }
//	  else
//		  goStraight(200);

	  // STRAIGHT GOING CODE END


	  // NEW CODE START
	  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);

	  HCSR04_Read();
	  millis = HAL_GetTick();
	  while ((Distance1_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}

	  makeDecision();

	  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);

	  HCSR2_Read();
	  millis = HAL_GetTick();
	  while ((Distance2_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}

	  makeDecision();

	  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);

	  HCSR5_Read();
	  millis = HAL_GetTick();
	  while ((Distance5_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_block_timeout)) {}

	  makeDecision();

	  // NEW CODE END


	  // FOR NOW, SINCE WE ARE WAITING WITH POLLING FOR DISTANCE SENSORS, DO NOT DO THIS.
	  // HAL_Delay(10); // Small delay to avoid sending commands to motors too frequently.

	  // ***** Rest of the while loop is error handling *****

	  // WE ALREADY CALL UART RECEIVE IT EVERY TIME AT THE TOP OF THE LOOP. NOT NEEDED AT THE MOMENT.
	  // Count how many times we went through the while loop without angle being updated.
//	  if (prevAngle == angle)
//	  {
//		  angleCounter++;
//		  if (angleCounter == 0)
//			  angleCounter = 255;
//	  }
//	  else
//	  {
//		  angleCounter = 0;
//	  }
//
//	  prevAngle = angle; // Update previous angle value
//
//	  // If the angle has been the same value for a while, call the UART Receive interrupt again.
//	  if (angleCounter >= ANGLE_SAMPLING_HALTED_COUNT) // CURRENTLY 8, BRING IT DOWN IF NEEDED
//	  {
//		  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
//	  }



//	  wCounter++;

















	      // Send the command to read from the distance sensors.
//	 	  HCSR04_Read();
//
//		  // Take snapshots of the volatile variables.
////		  dist1 = HCSR_Distance_1;	dist2 = HCSR_Distance_2;	dist5 = HCSR_Distance_5;
////		  angle_snapshot = angle;	temp_angle_snapshot = temp_angle;
//
//	 	  // Update the state.
//	 	  stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_5);
//	 	  turn = ISTURN(angle);
//
//	 	  // Interpret the results to send motor commands.
//	 	  if (turn)
//	 	  {
//	 		  if (ISLEFT(angle) && !turningLeft)
//	 		  {
//	 			  turnLeft(50);
//	 		  }
//	 		  else if (ISRIGHT(angle) && !turningRight)
//	 		  {
//	 			  turnRight(50);
//	 		  }
//	 	  }
//	 	  else if (stop && !stopped)
//	 	  {
//	 		  speed(0, 0);
//	 		  turningLeft = 0;
//	 		  turningRight = 0;
//	 		  stopped = 1;
//	 		  movingStraight = 0;
//	 	  }
//	 	  else if (!stop && !movingStraight)
//	 	  {
//	 		  goStraight(50);
//	 	  }
//
//	 	  HAL_Delay(10); // Small delay to avoid sending commands to motors too frequently.
//
//	 	  // ***** Rest of the while loop is error handling *****
//
//	 	  // Count how many times we went through the while loop without angle being updated.
//	 	  if (prevAngle == angle)
//	 	  {
//	 		  angleCounter++;
//	 		  if (angleCounter == 0)
//	 			  angleCounter = 255;
//	 	  }
//	 	  else
//	 	  {
//	 		  angleCounter = 0;
//	 	  }
//
//	 	  prevAngle = angle; // Update previous angle value
//
//	 	  // If the angle has been the same value for a while, call the UART Receive interrupt again.
//	 	  if (angleCounter >= ANGLE_SAMPLING_HALTED_COUNT)
//	 	  {
//	 		  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
//	 	  }
//
//	 	  // If the angle is outside the valid range, restart the UART.
//	      if (!IS_VALID_ANGLE(temp_angle))
//	      {
//	    	  reinitalize_USART1();
//	      }
//
//	      if (stopped)
//	    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	      else
//	    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);












//	  HCSR04_Read();
//	  HAL_Delay(30);
//	  if (AOA_Distance < STOP_RANGE_AOA)
//	  {
////		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	  }
//	  else
//	  {
////		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//	  }

//	  HAL_Delay(1000);
//
//	  printf("Dist 1: %u, Dist 2: %u\n", HCSR_Distance_1, HCSR_Distance_2);
//
//	  HAL_Delay(1000);

//	  HCSR04_Read();
//	  HAL_Delay(200);
//	  stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2); // (HCSR_Distance_2 < STOP_RANGE_HCSR);
//	  // Every 100ms or so (could be 200 or so), update the speed based on angle
//	  if (ISTURN(angle))
//	  {
//		  if (ISLEFT(angle) && !turningLeft)
//		  {
//			  turnLeft(50);
////			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//		  }
//		  else if (ISRIGHT(angle) && !turningRight)
//		  {
//			  turnRight(50);
////			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		  }
//		  // The following if statement doesn't work well cause it will constantly stop and turn us if we constantly have to turn.
////		  else if (stop && !stopped) // we get here if angle updates right after the initial if statement of: ISTURN(angle)
////		  {
////			  speed(0, 0);
////			  turningLeft = 0;
////			  turningRight = 0;
////			  stopped = 1;
////			  movingStraight = 0;
////		  }
//	  }
//	  else if (stop && !stopped)
//	  {
//		  speed(0, 0);
//		  turningLeft = 0;
//		  turningRight = 0;
//		  stopped = 1;
//		  movingStraight = 0;
//	  }
//	  else if (!stop && !movingStraight)
//	  {
//		  goStraight(50);
//	  }
//
//	  HAL_Delay(40);
//
//	  // count how many times we went through the while loop without angle being updated
//	  if (prevAngle == angle)
//	  {
//		  angleCounter++;
//		  if (angleCounter == 0)
//			  angleCounter = 255;
//	  }
//	  else
//	  {
//		  angleCounter = 0;
//	  }
//
//	  prevAngle = angle; // Update previous angle value
//
//	  // ************* THE FOLLOWING CODE SOLVES YOUR PROBLEM!!!!!!!!! **************
//	  // If the angle counter is larger than a certain threshold or the temp_angle is not within valid range,
//	  // then there is a problem with UART IT. Turn the LED on and start UART IT again.
//	  // I believe the problem might be the fact that UART callback of pi might be interrupted by the callback of
//	  // the HCSR distance sensors. And maybe it never returns since the function times out or something.
//	  if (angleCounter >= ANGLE_SAMPLING_HALTED_COUNT)
//	  {
//		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		  // No issues calling this frequently even though it is called in the callback.
//		  // Will just return HAL_BUSY when not available.
//		  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
//	  }
//
//	  // NOTE: In the bug where temp_angle is way out of the valid range constantly (happens when the STM32 is started earlier and then the
//	  // Pi is started. Initially angle stays constant until suddenly it is sampled to be out of its valid range), find a function to disable
//	  // and enable the UART interrupt!!!
//	  // UPDATE: Tried this and it didn't work. Once deinitialized, the angle never gets sampled again.
//	  // UPDATE: IT WORKEDDD OMG! I needed to call the interrupt function again to get a new, valid angle. Otherwise it keeps deinitializing.
//	   if (!IS_VALID_ANGLE(temp_angle))
//	   {
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		  reinitalize_USART1();
//	   }

//	  	  HCSR04_Read();
//	  	  if (HCSR_Distance_1 <= STOP_RANGE_HCSR || HCSR_Distance_2 <= STOP_RANGE_HCSR)
//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	  	  else
//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//
//	  	  HAL_Delay(40);


// UNCOMMENT AFTERRRR ************************************************
//	  HCSR04_Read();
//	  HAL_Delay(24); // Wait for the sound to hit something a surface and come back
//	  turn = ISTURN(angle);
//	  stop = (HCSR_Distance_1 < 30 || HCSR_Distance_2 <= STOP_RANGE_HCSR || AOA_Distance < STOP_RANGE_AOA);// ISSTOP(HCSR_Distance_1, HCSR_Distance_2, AOA_Distance);
//
//	  // Turn dominates stop because it already stops DOLL-E to begin with. And note that even if there is an object,
//	  // DOLL-E is still supposed to turn to face the user. This logic could be changed later on but then the user needs to
//	  // find a way to either remove the object or move DOLL-E.
//	  if (turn)
//	  {
//		  turnDOLLE(); // Turn DOLL-E constantly until it is facing the user
//	  }
//	  else if (stop)
//	  {
//		  speed(0, 0);
//		  HAL_Delay(20);
//	  }
//	  else
//	  {
//		  goStraight(50);
//		  HAL_Delay(20);
//	  }
// UNCOMMENT UNTIL HEREEEE ************************************************
//	  // STOP OR NOT
//	  if (stop)
//	  {
//		  speed(0, 0);
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		  HAL_Delay(40);
//	  }
//
//	  // TURN TOWARDS THE USER UNTIL THE USER IS IN FRONT OF DOLL-E
//	  do
//	  {
//		  if (ISLEFT(angle))
//		  {
//			  turnLeft(50);
//		  }
//		  else if(ISRIGHT(angle))
//		  {
//			  turnRight(50);
//		  }
//		  else // impossible to get to, but just in case
//		  {
//			  speed(0, 0);
//		  }
//		  HAL_Delay(40);
//	  } while(ISTURN(angle));
//
//
////	  goStraight(120);
//	  HAL_Delay(40);

//
//	  HCSR04_Read();
//	  HAL_Delay(100);

//	  if (HCSR_Distance_1 < STOP_RANGE_HCSR)
//	  {
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//	  }
//	  else
//	  {
//		  speed(70, 50);
//	  }

//	  int i;
//	  for (i = 0; i < 17; i++)
//	  {
//		  if (angle_arr[i] == L)
//		  {
////			  turnFunc(30, -30);
////			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 8);
////			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//			  speed(-100, 80);
//		  }
//		  else if (angle_arr[i] == R)
//		  {
////			  turnFunc(-30, 30);
////			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
////			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//			  speed(100, -80);
//		  }
//		  else
//		  {
////			  turnFunc(0, 0);
////			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
////			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//
//			  speed(0, 0);
//		  }
//
//		  HAL_Delay(1000);
//	  }

//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	// printf("HEY WORLD\n");
     //sprintf(MSG, "X = %d\r\n");
//	  memcpy(send, RX_);
//	  tx = 'a';
//      send_stat = HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//      HAL_Delay(1);
	  //HAL_stat = HAL_UART_Receive(&huart2, &rx, 1, HAL_MAX_DELAY);

//	  enterToNull(Rx_data, 100);

//	  if (HAL_stat != HAL_OK)
//	  {
//		  HAL_UART_Transmit(&huart2, rx_fail, strlen(rx_fail), 100);
//	  }
//	  else
//	  {
//		  HAL_UART_Transmit(&huart2, Rx_data, 1, 100);
//	  }

//	  int index;
//	  for (index = 500; index <= 1000; index++)
//	  {
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, index);
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, index);
//
////		  HAL_Delay(10);
//	  }
////	  HAL_Delay(1000);
//	  for (index = 1000; index >= 500; index--)
//	  {
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, index);
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, index);
//
////		  HAL_Delay(10);
//	  }

	  // Change this to whether you wanna check for receive or transmit
//	  HAL_StatusTypeDef stat_to_check = HAL_stat;
//	  if (stat_to_check == HAL_ERROR)
//	  {
////		  tx = 'e';
////		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
//	  else if (stat_to_check == HAL_TIMEOUT)
//	  {
////		  tx = 't';
////		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
//	  else if (stat_to_check ==HAL_OK)
//	  {
////		  tx = 'o';
////		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//
//		  if (Rx_data[0] == 'a')
//		  {
//			  HAL_Delay(1000);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  }
//	  }
//	  else if (stat_to_check == HAL_BUSY)
//	  {
////		  tx = 'b';
////		  		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		HAL_Delay(1000);
//		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  				  		  HAL_Delay(1000);
//		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
//	  else
//	  {
////		  tx = '?';
////		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		HAL_Delay(1000);
//		  		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  				  		  HAL_Delay(1000);
//		  		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  				  	HAL_Delay(1000);
//		  		  				  			  		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  				  			  		  				  		  HAL_Delay(1000);
//		  		  				  			  		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
    //HAL_UART_Transmit(&huart2, Rx_data, sizeof(Rx_data), 100);
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	HAL_Delay(2000);
	//X++;
//	send[0]++;
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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 16-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|TRIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG3_Pin */
  GPIO_InitStruct.Pin = TRIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG1_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG2_Pin */
  GPIO_InitStruct.Pin = TRIG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG2_GPIO_Port, &GPIO_InitStruct);

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
