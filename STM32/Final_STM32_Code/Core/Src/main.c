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
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM17_Init(void);
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
#define LED_PIN					GPIO_PIN_5
#define LED_PORT				GPIOA

// HC-SR04 definitions
#define TRIG1_PIN				GPIO_PIN_15
#define TRIG1_PORT				GPIOC

#define TRIG2_PIN				GPIO_PIN_4
#define TRIG2_PORT				GPIOA

#define TRIG3_PIN				GPIO_PIN_1
#define TRIG3_PORT				GPIOA

#define STOP_RANGE_HCSR			70 				// Forward movement stop. If HC-SR04 sees this or less (in cms), stop the motors
#define STOP_RANGE_TURN			25				// Turning is stopped when the HC-SR04 sees this or less in cms

#define HCSR1_timer_handler		htim1
#define HCSR1_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR1_TIM_IT_CC			TIM_IT_CC1		// Timer Capture Compare 1 Interrupt
#define HCSR1_TIMER_ADDRESS		TIM1			// Base Address of HCSR1's Timer
#define HCSR1_ACTIVE_CHANNEL	HAL_TIM_ACTIVE_CHANNEL_1

#define HCSR2_timer_handler		htim2
#define HCSR2_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR2_TIM_IT_CC			TIM_IT_CC1
#define HCSR2_TIMER_ADDRESS		TIM2
#define HCSR2_ACTIVE_CHANNEL	HAL_TIM_ACTIVE_CHANNEL_1

#define HCSR3_timer_handler		htim17
#define HCSR3_TIMER_CHANNEL		TIM_CHANNEL_1
#define HCSR3_TIM_IT_CC			TIM_IT_CC1
#define HCSR3_TIMER_ADDRESS		TIM17
#define HCSR3_ACTIVE_CHANNEL	HAL_TIM_ACTIVE_CHANNEL_1

#define HCSR_BLOCK_TIMEOUT		75

//#define HCSR4_timer_handler		htim14
//#define HCSR4_TIMER_CHANNEL		TIM_CHANNEL_1
//#define HCSR4_TIM_IT_CC			TIM_IT_CC1
//#define HCSR4_TIMER_ADDRESS		TIM14
//#define HCSR4_ACTIVE_CHANNEL	HAL_TIM_ACTIVE_CHANNEL_1
//
//#define HCSR5_timer_handler		htim17
//#define HCSR5_TIMER_CHANNEL		TIM_CHANNEL_1
//#define HCSR5_TIM_IT_CC			TIM_IT_CC1
//#define HCSR5_TIMER_ADDRESS		TIM17
//#define HCSR5_ACTIVE_CHANNEL	HAL_TIM_ACTIVE_CHANNEL_1

// HC-SR04 Enables (change to 0 to disable. Else, 1.)
#define HCSR1_EN	1
#define HCSR2_EN	1
#define HCSR3_EN	1

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
#define ISSTOP(_hcsr_dist_1, _hcsr_dist_2, _hcsr_dist_3) (((_hcsr_dist_1) < STOP_RANGE_HCSR) || ((_hcsr_dist_2) < STOP_RANGE_HCSR) || ((_hcsr_dist_3) < STOP_RANGE_HCSR))
#define IS_STOP_TURN(_hcsr_dist_1, _hcsr_dist_2, _hcsr_dist_3) (((_hcsr_dist_1) < STOP_RANGE_TURN) || ((_hcsr_dist_2) < STOP_RANGE_TURN) || ((_hcsr_dist_3) < STOP_RANGE_TURN))

// AoA Enables (change to 0 to disable. Else, 1.)
#define AOA_EN	1

// AoA Error Handling (Important)
#define ANGLE_SAMPLING_HALTED_COUNT 12 // If the angle is the same for 4 consecutive times in the super loop, assume that UART sampling is halted and request it again

// AoA USART defines
#define AOA_USART_NUM_BYTES 2 // Change to 3 if adding AoA distance into USART
#define aoa_buffer			rx

// Motor Controller PWM defines
#define FRICTION_OFFSET			5
// Right motor has different friction than the left one. This accounts for that.
#define MAX_TURN_SPEED			RIGHT_LIMIT + 30
// This is the maximum motor speed we want to reach. More than this could be too fast.
#define TURN_SPEED				55		// Turning speed if it is a constant.
#define FORWARD_SPEED			140		// Moving forward speed if it is a constant.
#define MAX_FORWARD_SPEED		160		// This speed is the maximum speed that DOLL-E will move forward.

// Operation Macros
#define MIN(_a, _b, _c) (((_a) < (_b)) ? \
						 ((_a) < (_c)) ? \
						 (_a) : (_c) : ((_b) < (_c)) ? \
						 (_b) : (_c))
#define ABS(_ang_) ((_ang_) < 0 ? -(_ang_) : (_ang_))

// Speed conversion macros
#define ANGLE_TO_SPEED(__angle__) (MAX_TURN_SPEED - ((int32_t)(ABS(__angle__))))
#define HCSR_DIST_TO_SPEED(_d1, _d2, _d3) (MIN(_d1, _d2, _d3) - STOP_RANGE_HCSR)

/*********************************************************************************
								Global Variables
**********************************************************************************/
// State machine variables
// The following 3 variables are the commands to stop and turn the DOLL-E.
// They indicate that DOLL-E should turn, but not that it is turning yet.
uint8_t stop = 1;
uint8_t turn = 0;
uint8_t stopTurn = 1; // Stop from turning

// The following 4 states actually represent the current state of DOLL-E (what it is currently doing).
uint8_t stopped = 1;
uint8_t turningLeft = 0;
uint8_t turningRight = 0;
uint8_t movingStraight = 0;

// AoA variables
int16_t angle = 178; // Angle of asset tag (azimuth) relative to the antenna board
int16_t temp_angle = 178; // For checking the validity of angle before assigning it to variable "angle"
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

uint32_t Difference_1 = 0;  // Difference in time between rising and falling edge of HCSR1's ECHO pin
uint32_t Difference_2 = 0;  // Difference in time between rising and falling edge of HCSR2's ECHO pin
uint32_t Difference_3 = 0;

uint8_t Is_First_Captured_1 = 0;  // Indicates whether the rising edge is captured for HCSR1's ECHO pin
uint8_t Is_First_Captured_2 = 0;  // Indicates whether the rising edge is captured for HCSR2's ECHO pin
uint8_t Is_First_Captured_3 = 0;

uint8_t Distance1_flag = 0; // These flags are set to 1 when distance is calculated.
uint8_t Distance2_flag = 0;
uint8_t Distance3_flag = 0;

uint16_t HCSR_Distance_1  = 101; // HCSR04 Distance Sensor #1 output
uint16_t HCSR_Distance_2  = 101; // HCSR04 Distance Sensor #2 output
uint16_t HCSR_Distance_3  = 101; // HCSR04 Distance Sensor #3 output

// USART variables
HAL_StatusTypeDef USART_State = HAL_ERROR;

// Timer variables
uint32_t millis = 0;

/*********************************************************************************
								Macro Functions Begin
**********************************************************************************/
#define HCSR_INPUT_HANDLE(_htim, _HCSR_TIMER_ADDRESS, _HCSR_ACTIVE_CHANNEL, _HCSR_TIMER_CHANNEL, _HCSR_TIM_IT_CC, _Is_First_Captured, _IC_VAL1_HCSR, _IC_VAL2_HCSR, _Difference, _HCSR_Distance, _Dist_Flag) \
		if ((_htim)->Instance == (_HCSR_TIMER_ADDRESS) && (_htim)->Channel == (_HCSR_ACTIVE_CHANNEL)) { \
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
	TIM_HandleTypeDef *htim = NULL;

#if HCSR3_EN
	htim = &HCSR3_timer_handler;
#elif HCSR2_EN
	htim = &HCSR2_timer_handler;
#elif HCSR1_EN
	htim = &HCSR1_timer_handler;
#endif

#if (HCSR3_EN || HCSR2_EN || HCSR1_EN)
	if (htim != NULL)
	{
		__HAL_TIM_SET_COUNTER(htim, 0);
		while (__HAL_TIM_GET_COUNTER(htim) < time);
	}
#endif
}

/*********************************************************************************
								HC-SR04 Functions Begin
**********************************************************************************/
// Timer Input Capture Interrupt Callback for HC-SR04
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
#if HCSR1_EN
	HCSR_INPUT_HANDLE(htim, HCSR1_TIMER_ADDRESS, HCSR1_ACTIVE_CHANNEL, HCSR1_TIMER_CHANNEL,
					  HCSR1_TIM_IT_CC, Is_First_Captured_1, IC_VAL1_HCSR1, IC_VAL2_HCSR1,
					  Difference_1, HCSR_Distance_1, Distance1_flag);
#endif
#if HCSR2_EN
	HCSR_INPUT_HANDLE(htim, HCSR2_TIMER_ADDRESS, HCSR2_ACTIVE_CHANNEL, HCSR2_TIMER_CHANNEL,
					  HCSR2_TIM_IT_CC, Is_First_Captured_2, IC_VAL1_HCSR2, IC_VAL2_HCSR2,
					  Difference_2, HCSR_Distance_2, Distance2_flag);
#endif
#if HCSR3_EN
	HCSR_INPUT_HANDLE(htim, HCSR3_TIMER_ADDRESS, HCSR3_ACTIVE_CHANNEL, HCSR3_TIMER_CHANNEL,
					  HCSR3_TIM_IT_CC, Is_First_Captured_3, IC_VAL1_HCSR3, IC_VAL2_HCSR3,
					  Difference_3, HCSR_Distance_3, Distance3_flag);
#endif
}

// Pulls the trigger pin high for 10us to start the calculation
void HCSR1_Read (void)
{
#if HCSR1_EN
	Distance1_flag = 0;
	HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	// Enable the interrupt for capturing how long the ECHO pin stays high in order to calculate the distance
	__HAL_TIM_ENABLE_IT(&HCSR1_timer_handler, HCSR1_TIM_IT_CC);
#endif
}

// Pulls the trigger pin high for 10us to start the calculation
void HCSR2_Read (void)
{
#if HCSR2_EN
	Distance2_flag = 0;
	HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	// Enable the interrupt for capturing how long the ECHO pin stays high in order to calculate the distance
	__HAL_TIM_ENABLE_IT(&HCSR2_timer_handler, HCSR2_TIM_IT_CC);
#endif
}

// Pulls the trigger pin high for 10us to start the calculation
void HCSR3_Read (void)
{
#if HCSR3_EN
	Distance3_flag = 0;
	HAL_GPIO_WritePin(TRIG3_PORT, TRIG3_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG3_PORT, TRIG3_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	// Enable the interrupt for capturing how long the ECHO pin stays high in order to calculate the distance
	__HAL_TIM_ENABLE_IT(&HCSR3_timer_handler, HCSR3_TIM_IT_CC);
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
int16_t angle_buff[ANGLE_BUFF_SIZE] = {179, 179, 179, 179, 179}; // Ring buffer (replaces the oldest angle with the new output at each iteration)
uint8_t angle_buff_index = 0;
uint16_t angle_buff_total = 179 * ANGLE_BUFF_SIZE;

// UART receive callback: gets called as soon as STM32 receives through UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// THE NORMAL CODE BEGIN ********************
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
	// THE NORMAL CODE END **********************

	// NOTE FOR SELF: if communication stops while turning or moving forward, make an error handling to stop motors.
	// Maybe a timer of 5 seconds -> if within 5 seconds no UART receive, then stop motors

// _____________________________________ RELATIVE RECENT ANGLE CALCULATION ____________________________________
// UNCOMMENT THE FOLLOWING BOX WHEN POSSIBLE. IT APPLIES AVERAGING OF THE RECENT ANGLES AND GETS RID OF THE
// EDGE CASE WHERE IF THE USER IS STANDING IN THE THRESHOLD OF LEFT AND RIGHT (AROUND 0 IN OUR CASE), THEN
// DOLL-E WILL KEEP MOVING LEFT AND RIGHT WITHOUT BEING ABLE TO FACE THE USER. IF YOU UNCOMMENT THIS BOX,
// MAKE SURE TO COMMENT THE LINE WHERE "angle" IS BEING ASSIGNED.
//
//	// insert angle into angle buffer
//	temp_angle = ((int16_t *)rx)[0];
//	int16_t val_replaced = angle_buff[angle_buff_index]; // Store the value to be replaced before it is gone.
//
//	if (IS_VALID_ANGLE(temp_angle))
//	{
//		angle_buff[angle_buff_index] = temp_angle;
//	}
//	else
//	{
//		stop = 1;
//		turn = 0;
//		angle_buff[angle_buff_index] = 179;
//	}
//
//	angle_buff_total = angle_buff_total - val_replaced + angle_buff[angle_buff_index]; // Update the sum of angle buff
//	angle_buff_index = (angle_buff_index + 1) % ANGLE_BUFF_SIZE; // update the index in a ring buffer style
//
//	// Round angle efficiently based on its range [-179, 179]. It is ensured through error
//	// handling that the average angle calculated will be within the valid angle range.
//	// Source of the rounding formula: https://www.cs.cmu.edu/~rbd/papers/cmj-float-to-int.html
//	angle = (((int16_t) (((float)angle_buff_total/ANGLE_BUFF_SIZE) + 179.5f)) - 179);
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

//	__HAL_TIM_SET_COUNTER(&pwm_timer_handler, 0);
	__HAL_TIM_SET_COMPARE(&pwm_timer_handler, PWM_LEFT_MOTOR_CHANNEL, set_l);
	__HAL_TIM_SET_COMPARE(&pwm_timer_handler, PWM_RIGHT_MOTOR_CHANNEL, set_r);
}

// Input: Left wheel absolute speed
void turnLeft(int32_t speed_val)
{
	turningLeft = 1;
	turningRight = 0;
	stopped = 0;
	movingStraight = 0;

	// Less than FRICTION_OFFSET is there because the friction on the left wheel is more.
	if (speed_val < 0 || speed_val > 250)
	{
		speed(0, 0);
		return;
	}
	else if (speed_val < FRICTION_OFFSET)
	{
		speed_val += FRICTION_OFFSET;
	}

	speed(-1*speed_val, speed_val - FRICTION_OFFSET);
}

// Input: Left wheel absolute speed (since left wheel has the larger absolute value at all times)
void turnRight(int32_t speed_val)
{
	turningRight = 1;
	turningLeft = 0;
	stopped = 0;
	movingStraight = 0;

	// Less than FRICTION_OFFSET is there because the friction on the left wheel is more.
	if (speed_val < 0 || speed_val > 250)
	{
		speed(0, 0);
		return;
	}
	else if (speed_val < FRICTION_OFFSET)
	{
		speed_val += FRICTION_OFFSET;
	}

	speed(speed_val, -1*(speed_val - FRICTION_OFFSET));
}

void turnLeftAnalog(int32_t speed_val, int32_t maxSpeed)
{
	turningLeft = 1;
	turningRight = 0;
	stopped = 0;
	movingStraight = 0;

	if (speed_val < 0)
	{
		speed(0, 0);
		return;
	}
	else if (speed_val > (maxSpeed - FRICTION_OFFSET))
	{
		speed_val = maxSpeed;
	}
	else
	{
		speed_val += FRICTION_OFFSET;
	}

	speed(-1*speed_val, speed_val - FRICTION_OFFSET);
}

void turnRightAnalog(int32_t speed_val, int32_t maxSpeed)
{
	turningRight = 1;
	turningLeft = 0;
	stopped = 0;
	movingStraight = 0;

	if (speed_val < 0)
	{
		speed(0, 0);
		return;
	}
	else if (speed_val > (maxSpeed - FRICTION_OFFSET))
	{
		speed_val = maxSpeed;
	}
	else
	{
		speed_val += FRICTION_OFFSET;
	}

	speed(speed_val, -1*(speed_val - FRICTION_OFFSET));
}

// This function ensures that DOLL-E starts slowing down from (-LEFT_LIMIT - TURN_SPEED) to (-LEFT_LIMIT - 1). If the angle is
// after a certain threshold, DOLL-E caps at the maximum speed of TURN_SPEED.
void turnLeftCapped(int16_t inputAngle)
{
	turningLeft = 1;
	turningRight = 0;
	stopped = 0;
	movingStraight = 0;
	int32_t speed_val = 0;

		// Cap the turning speed at TURN_SPEED (TURN_SPEED is the maximum speed it could achieve while turning)
		if (inputAngle >= (-1*LEFT_LIMIT))
		{
			speed(0, 0);
			return;
		}
		else if (inputAngle >= ((-1*LEFT_LIMIT) - TURN_SPEED + FRICTION_OFFSET))
		{
			speed_val = (-1*LEFT_LIMIT) - inputAngle + FRICTION_OFFSET;
		}
		else
		{
			speed_val = TURN_SPEED;
		}

	speed(-1*speed_val, speed_val - FRICTION_OFFSET);
}

// This function ensures that DOLL-E starts slowing down from (RIGHT_LIMIT - TURN_SPEED) to (RIGHT_LIMIT - 1). If the angle is
// after a certain threshold, DOLL-E caps at the maximum speed of TURN_SPEED.
void turnRightCapped(int16_t inputAngle) // Assumes the angle is positive (absolute)
{
	turningRight = 1;
	turningLeft = 0;
	stopped = 0;
	movingStraight = 0;
	int32_t speed_val = 0;

	// Cap the turning speed at TURN_SPEED (TURN_SPEED is the maximum speed it could achieve while turning)
	if (inputAngle >= RIGHT_LIMIT)
	{
		speed(0, 0);
		return;
	}
	else if (inputAngle >= (RIGHT_LIMIT - TURN_SPEED + FRICTION_OFFSET))
	{
		speed_val = RIGHT_LIMIT - inputAngle + FRICTION_OFFSET;
	}
	else
	{
		speed_val = TURN_SPEED;
	}

	speed(speed_val, -1*(speed_val - FRICTION_OFFSET));
}



// Input: Left wheel absolute speed
// NOTE: Do not use it with analog speed. Assumes that if the speed is larger than max, there is an issue with the system.
void goStraight(int32_t speed_val)
{
	turningLeft = 0;
	turningRight = 0;
	stopped = 0;
	movingStraight = 1;

	// Speed value shouldn't equal 0 when going forward. if it does, make sure to make both wheels 0.
	if (speed_val <= 0 || speed_val > (250 - FRICTION_OFFSET))
	{
		speed(0, 0);
		return;
	}

	speed_val += FRICTION_OFFSET;

	speed(speed_val, speed_val - FRICTION_OFFSET);
}

// Maximum speed is maxSpeed.
void goStraightAnalog(int32_t speed_val, int32_t maxSpeed)
{
	turningLeft = 0;
	turningRight = 0;
	stopped = 0;
	movingStraight = 1;

	// Speed value shouldn't equal 0 when going forward. if it does, make sure to make both wheels 0.
	if (speed_val <= 0)
	{
		speed(0, 0);
		return;
	}
	else if (speed_val > (maxSpeed - FRICTION_OFFSET)) // Theoretically, max speed_val is (400 - HCSR_DIST_RANGE)
	{
		speed_val = maxSpeed;
	}
	else
	{
		speed_val += FRICTION_OFFSET;
	}

	speed(speed_val, speed_val - FRICTION_OFFSET);
}

// After the cap value, the speed is set to FORWARD_SPEED. So, the maximum speed is FORWARD_SPEED, not capVal.
// How many ticks after the STOP_RANGE_HCSR would you like to go in FORWARD_SPEED? That is capVal.
void goStraightAnalogCapped(int32_t speed_val, int32_t capVal)
{
	turningLeft = 0;
	turningRight = 0;
	stopped = 0;
	movingStraight = 1;

	// Speed value shouldn't equal 0 when going forward. if it does, make sure to make both wheels 0.
	if (speed_val <= 0)
	{
		speed(0, 0);
		return;
	}
	else if (speed_val > capVal)
	{
		speed_val = FORWARD_SPEED;
	}
	else
	{
		speed_val += FRICTION_OFFSET;
	}

	speed(speed_val, speed_val - FRICTION_OFFSET);
}

void goStraightWithAdjustments(int32_t speed_val, int16_t angle)
{
	turningLeft = 0;
	turningRight = 0;
	stopped = 0;
	movingStraight = 1;

	if (speed_val <= FRICTION_OFFSET || speed_val > 250)
	{
		speed(0, 0);
		return;
	}

	if (angle < 0 && angle > -170)
	{
		int u = angle + 179;
		speed(speed_val - (int32_t)(2 * u), speed_val);
	}
	else if (angle >= 0 && angle < 170)
	{
		int u = -1 * (angle - 179);
		speed(speed_val, speed_val - (int32_t)(2 * u));
	}
	else
	{
		speed(speed_val, speed_val - FRICTION_OFFSET);
	}
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
    USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
}
/*********************************************************************************
					   Error Handling Functions End
**********************************************************************************/

void makeDecision()
{
	 // Update the state.
	  stop = ISSTOP(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_3);
	  stopTurn = IS_STOP_TURN(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_3);
	  turn = ISTURN(angle);

	  // Interpret the results to send motor commands.
	  if (turn)
	  {
		  if (stopTurn)
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
		  else if (ISLEFT(angle))// && !turningLeft) // if (ISLEFT(angle)  && !turningLeft)
		  {
//			   turnLeft(TURN_SPEED);		   							// Turn in a constant speed
			   turnLeftAnalog(ANGLE_TO_SPEED(angle), MAX_TURN_SPEED); 	// Turn in spectrum
//			   turnLeftCapped(ABS(angle));	   							// Turn in limited spectrum (TURN_SPEED is the maximum speed)
		  }
		  else if (ISRIGHT(angle))// && !turningRight) // else if (ISRIGHT(angle) && !turningRight)
		  {
//			   turnRight(TURN_SPEED); 									// Turn in a constant speed
			   turnRightAnalog(ANGLE_TO_SPEED(angle), MAX_TURN_SPEED);	// Turn in spectrum, maximum speed is second argument
//			   turnRightCapped(ABS(angle)); 							// Turn in limited spectrum (TURN_SPEED is the maximum speed)
		  }
	  }
	  else if (stop)
	  {
		  if (!stopped)
		  {
			  turningLeft = 0;
			  turningRight = 0;
			  stopped = 1;
			  movingStraight = 0;
			  speed(0, 0);
		  }
	  }
	  else if (!stop)
	  {
		  // TO DO: HAVE AN ARRAY OF HISTORY FOR EACH DISTANCE SENSOR. HAVE THEIR LAST 5 OR 3 VALUES IN THE ARRAY. EACH DISTANCE
		  // SENSOR'S CURRENT VALUE WILL BE THE AVERAGE OF THEIR LAST 3 OR 5 VALUES. THIS MIGHT NOT NECESSARILY HELP BECAUSE OF
		  // THE WAY OUR FEET MOVE. BUT TRY IT IF YOU HAVE TIME.

//		  if (!movingStraight)
//			  goStraight(FORWARD_SPEED);

//		   goStraightAnalog(HCSR_DIST_TO_SPEED(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_3), MAX_FORWARD_SPEED);

//		  goStraightAnalogCapped(HCSR_DIST_TO_SPEED(HCSR_Distance_1, HCSR_Distance_2, HCSR_Distance_3), 40);
		  goStraightWithAdjustments(FORWARD_SPEED ,angle);
	  }

#if AOA_EN
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

	  // If the angle is outside the valid range, restart the UART.
	  if (!IS_VALID_ANGLE(temp_angle) || angleCounter >= ANGLE_SAMPLING_HALTED_COUNT)
	  {
		  reinitalize_aoa_UART(); // GOTTA TAKE OUT THE USART IT CALL IF WE ARE CALLING IT ALREADY IN THE LOOP
	  }
#endif

	  //
	  //	  // If the angle has been the same value for a while, call the UART Receive interrupt again.
	  //	  if (angleCounter >= ANGLE_SAMPLING_HALTED_COUNT) // CURRENTLY 8, BRING IT DOWN IF NEEDED
	  //	  {
	  //		  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
	  //	  }

	  if (stopped)
		  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
	  else
		  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

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
  MX_TIM17_Init();
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

  // ********** PWM Start for Controlling the Motors **********
#if MOTOR_EN
  HAL_TIM_PWM_Start(&pwm_timer_handler, PWM_LEFT_MOTOR_CHANNEL); // Start the PWM for left motor
  HAL_TIM_PWM_Start(&pwm_timer_handler, PWM_RIGHT_MOTOR_CHANNEL); // Start the PWM for right motor
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  speed(0, 0);
  int iter = 0;
  for (iter = 0; iter < 23; iter++)
  {
	  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
	  HAL_Delay(1000);
  }
  while (1)
  {
#if AOA_EN
	  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
#endif

#if HCSR1_EN
	  HCSR1_Read();
	  millis = HAL_GetTick();
	  while ((Distance1_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_BLOCK_TIMEOUT)) {}
#endif

	  makeDecision();

#if AOA_EN
	  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
#endif

#if HCSR2_EN
	  HCSR2_Read();
	  millis = HAL_GetTick();
	  while ((Distance2_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_BLOCK_TIMEOUT)) {}
#endif

	  makeDecision();

#if AOA_EN
	  USART_State = HAL_UART_Receive_IT(&huart1, rx, AOA_USART_NUM_BYTES);
#endif

#if HCSR3_EN
	  HCSR3_Read();
	  millis = HAL_GetTick();
	  while ((Distance3_flag == 0) && ((HAL_GetTick() - millis) <= HCSR_BLOCK_TIMEOUT)) {}
#endif

	  makeDecision();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  htim1.Init.Prescaler = 64-1;
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
  htim2.Init.Prescaler = 64-1;
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
  htim3.Init.Prescaler = 128-1;
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
  htim17.Init.Prescaler = 64-1;
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
  HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG3_Pin|TRIG2_Pin|TEST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TRIG1_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG3_Pin TRIG2_Pin TEST_LED_Pin */
  GPIO_InitStruct.Pin = TRIG3_Pin|TRIG2_Pin|TEST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
