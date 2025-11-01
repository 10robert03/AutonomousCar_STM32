/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Autonomous STM32 Car (PWM Motor + Ultrasonic Sensors)
 *
 * @description
 * This firmware controls a 4-wheel autonomous car using:
 *  - 8 PWM outputs (for motor direction & speed via H-Bridge)
 *  - 3 ultrasonic sensors (front, left, right) for obstacle detection
 *  - STM32 HAL and DWT (Data Watchpoint & Trace) for precise microsecond timing
 *
 * The car drives forward by default. If an obstacle is detected,
 * it decides whether to turn left, right, or drive backward until clear.
 *
 * Developed and tested on STM32 with HAL drivers.
 *
 * Author: Robert Deines
 * Date: 2025-10-31
 ******************************************************************************
 */

#include "main.h"
#include "tim.h"
#include "gpio.h"

// Core system setup
void SystemClock_Config(void);
void DWT_Delay_Init(void);
void delay_us(uint32_t us);

// Ultrasonic functions
uint8_t isObjectFront(void);
uint8_t isObjectLeft(void);
uint8_t isObjectRight(void);
uint32_t pulseIn(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState state, uint32_t timeout_us);

// Motor control
void forward(uint16_t speed);
void backward(uint16_t speed);
void turnLeft(void);
void turnRight(void);
void brake(void);
void noPower(void);

/**
 * @brief  Main program entry point.
 */
int main(void)
{
  // Initialize HAL library and system clock
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  DWT_Delay_Init();   // Enable microsecond timing via DWT

  // Start all PWM channels (TIM1 + TIM2)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  // --- Main control loop ---
  while (1)
  {
    if (!isObjectFront()) {
      forward(500); // Move forward with ~50% PWM duty
    }
    else {
      brake();
      noPower();

      if (isObjectRight() && !isObjectLeft()) {
        turnLeft();
      }
      else if (!isObjectRight() && isObjectLeft()) {
        turnRight();
      }
      else {
        // Both sides blocked -> reverse until clear
        while (isObjectRight() && isObjectLeft()) {
          backward(500);
        }

        brake();
        noPower();

        if (!isObjectLeft()) turnLeft();
        else if (!isObjectRight()) turnRight();
      }
    }
  }
}

/* ============================================================
 *  MICROSECOND DELAY + PULSE MEASUREMENT
 * ============================================================
 */

/**
 * @brief  Initializes DWT (Data Watchpoint and Trace) for µs precision.
 */
void DWT_Delay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // Enable trace block
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // Enable CPU cycle counter
}

uint32_t pulseIn(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState state, uint32_t timeout_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t timeout_cycles = (SystemCoreClock / 1000000U) * timeout_us;

  // Wait for pin to reach the desired state
  while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != state) {
    if ((DWT->CYCCNT - start) > timeout_cycles) return 0;
  }

  // Record start time
  uint32_t t1 = DWT->CYCCNT;

  // Wait for pin to leave the state
  while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == state) {
    if ((DWT->CYCCNT - start) > timeout_cycles) return 0;
  }

  // Record end time and convert to microseconds
  uint32_t t2 = DWT->CYCCNT;
  return (t2 - t1) / (SystemCoreClock / 1000000U);
}

/**
 * @brief  Blocks execution for a given number of microseconds.
 */
void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks);
}

/* ============================================================
 *  MOTOR CONTROL FUNCTIONS
 * ============================================================
 * Motors are controlled via PWM on TIM1 and TIM2 channels.
 * Each motor direction is toggled via paired PWM pins.
 */

void forward(uint16_t speed)
{
  if (speed > 999) speed = 999;

  // Front motors
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed); // IN1 for H-Bridge
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // IN2
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed); // IN3
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // IN4

  // Rear motors
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); // IN1BACK
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // IN2BACK
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed); // IN3BACK
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // IN4BACK
}

void backward(uint16_t speed)
{
  if (speed > 999) speed = 999;

  // Front motors (reverse)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);

  // Rear motors (reverse)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed);
}

void turnLeft(void)
{
  // Left turn: left wheels backward, right wheels forward
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 999);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 999);

  HAL_Delay(300);
}

void turnRight(void)
{
  // Right turn: right wheels backward, left wheels forward
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 999);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  HAL_Delay(300);
}

void brake(void)
{
  // Apply electrical brake (short both motor leads)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 999);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 999);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 999);
  HAL_Delay(200);
}

void noPower(void)
{
  // Disable all PWM outputs (motors off)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

/* ============================================================
 *  ULTRASONIC SENSOR FUNCTIONS
 * ============================================================
 * Each HC-SR04 sensor is triggered with a 10 µs pulse.
 * The echo pulse duration determines the distance.
 * Formula: distance (cm) = duration (µs) * 0.034 / 2
 */

uint8_t isObjectFront(void)
{
  HAL_GPIO_WritePin(SENDULTRASONICFRONT_GPIO_Port, SENDULTRASONICFRONT_Pin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(SENDULTRASONICFRONT_GPIO_Port, SENDULTRASONICFRONT_Pin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(SENDULTRASONICFRONT_GPIO_Port, SENDULTRASONICFRONT_Pin, GPIO_PIN_RESET);

  uint32_t duration = pulseIn(RECEIVEULTRASONICFRONT_GPIO_Port, RECEIVEULTRASONICFRONT_Pin, GPIO_PIN_SET, 25000);
  if (duration == 0) return 0;

  float distance = duration * 0.034f / 2;
  return (distance < 50);
}

uint8_t isObjectRight(void)
{
  HAL_GPIO_WritePin(SENDULTRASONICRIGHT_GPIO_Port, SENDULTRASONICRIGHT_Pin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(SENDULTRASONICRIGHT_GPIO_Port, SENDULTRASONICRIGHT_Pin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(SENDULTRASONICRIGHT_GPIO_Port, SENDULTRASONICRIGHT_Pin, GPIO_PIN_RESET);

  uint32_t duration = pulseIn(RECEIVEULTRASONICRIGHT_GPIO_Port, RECEIVEULTRASONICRIGHT_Pin, GPIO_PIN_SET, 25000);
  if (duration == 0) return 0;

  float distance = duration * 0.034f / 2;
  return (distance < 20);
}

uint8_t isObjectLeft(void)
{
  HAL_GPIO_WritePin(SENDULTRASONICLEFT_GPIO_Port, SENDULTRASONICLEFT_Pin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(SENDULTRASONICLEFT_GPIO_Port, SENDULTRASONICLEFT_Pin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(SENDULTRASONICLEFT_GPIO_Port, SENDULTRASONICLEFT_Pin, GPIO_PIN_RESET);

  uint32_t duration = pulseIn(RECEIVEULTRASONICLEFT_GPIO_Port, RECEIVEULTRASONICLEFT_Pin, GPIO_PIN_SET, 25000);
  if (duration == 0) return 0;

  float distance = duration * 0.034f / 2;
  return (distance < 20);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
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
