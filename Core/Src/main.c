/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "MPU9250.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef StaticTask_t osStaticThreadDef_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG 57.2957786
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t KalmanTaskHandle;
uint32_t KalmanTaskBuffer[ 3000 ];
osStaticThreadDef_t KalmanTaskControlBlock;
const osThreadAttr_t KalmanTask_attributes = {
  .name = "KalmanTask",
  .cb_mem = &KalmanTaskControlBlock,
  .cb_size = sizeof(KalmanTaskControlBlock),
  .stack_mem = &KalmanTaskBuffer[0],
  .stack_size = sizeof(KalmanTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};

float angle_pitch = 0.0f; // Reset the angle
float angle_roll = 0.0f; // Reset the angle
float bias_pitch = 0.0f; // Reset bias
float bias_roll = 0.0f; // Reset bias

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

// Tasks
void StartDefaultTask(void *argument);
void KalmanTask(void *argument);


bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  MPU9250_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  //KalmanTaskHandle = osThreadNew(KalmanTask, NULL, &KalmanTask_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  
  /* USER CODE BEGIN StartDefaultTask */
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart3,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  //int return_ping = (RMW_RET_OK == rmw_uros_ping_agent(1, 10));

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__); 
  }

  // micro-ROS app

  rcl_publisher_t publisher;
  std_msgs__msg__Int32 msg;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  allocator = rcl_get_default_allocator();

  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "cubemx_node", "", &support);

  // create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cubemx_publisher");

  msg.data = 0;

  for(;;)
  {
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__); 
    }
    
    msg.data++;
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the KalmanTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void KalmanTask(void *argument)
{
  // Kalman Initialization
  float Q_angle = 0.001f;
  float Q_bias = 0.003f;
  float R_measure = 0.03f;
  
  angle_pitch = 0.0f; // Reset the angle
  angle_roll = 0.0f; // Reset the angle
  bias_pitch = 0.0f; // Reset bias
  bias_roll = 0.0f; // Reset bias
  float rate_pitch;
  float rate_roll;

  float P_pitch[2][2]; // Error covariance matrix - This is a 2x2 matrix  
  P_pitch[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
  P_pitch[0][1] = 0.0f;
  P_pitch[1][0] = 0.0f;
  P_pitch[1][1] = 0.0f;

  float P_roll[2][2]; // Error covariance matrix - This is a 2x2 matrix  
  P_roll[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
  P_roll[0][1] = 0.0f;
  P_roll[1][0] = 0.0f;
  P_roll[1][1] = 0.0f;

  int16_t AccData[3], GyroData[3], MagData[3];
  float dt = 0.01f; // FIXME: delta_t from ticks?
  
  MPU9250_GetData(AccData, GyroData, MagData);
  MPU9250_GetData(AccData, GyroData, MagData);
  
  for (;;)
    {

      MPU9250_GetData(AccData, GyroData, MagData);

      // FIXME: Obtener angulos de pitch y roll
      
      //HAL_GPIO_TogglePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin);
      

      double roll  = atan2(AccData[1], AccData[2]) * RAD_TO_DEG;                                                                                                                                                                                              
      double pitch = atan(-AccData[0] / sqrt(AccData[1] * AccData[1] + AccData[2] * AccData[2])) * RAD_TO_DEG;   

      // Discrete Kalman filter time update equations - Time Update ("Predict")
      // Update xhat - Project the state ahead
      /* Step 1 */
      rate_pitch = (GyroData[0]/131.0) - bias_pitch; // Warning: harcoded constant gyro config dependent
      rate_roll = (GyroData[1]/131.0) - bias_roll;   // Warning: harcoded constant gyro config dependent
      angle_pitch += dt * rate_pitch;
      angle_roll += dt * rate_roll;

      // Update estimation error covariance - Project the error covariance ahead
      /* Step 2 */
      P_pitch[0][0] += dt * (dt*P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
      P_pitch[0][1] -= dt * P_pitch[1][1];
      P_pitch[1][0] -= dt * P_pitch[1][1];
      P_pitch[1][1] += Q_bias * dt;

      P_roll[0][0] += dt * (dt*P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
      P_roll[0][1] -= dt * P_roll[1][1];
      P_roll[1][0] -= dt * P_roll[1][1];
      P_roll[1][1] += Q_bias * dt;

      // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
      // Calculate Kalman gain - Compute the Kalman gain
      /* Step 4 */
      float S_pitch = P_pitch[0][0] + R_measure; // Estimate error
      float S_roll = P_roll[0][0] + R_measure; // Estimate error
      /* Step 5 */
      float K_pitch[2]; // Kalman gain - This is a 2x1 vector
      K_pitch[0] = P_pitch[0][0] / S_pitch;
      K_pitch[1] = P_pitch[1][0] / S_pitch;

      float K_roll[2]; // Kalman gain - This is a 2x1 vector
      K_roll[0] = P_roll[0][0] / S_roll;
      K_roll[1] = P_roll[1][0] / S_roll;

      // Calculate angle and bias - Update estimate with measurement zk (newAngle)
      /* Step 3 */
      float y_pitch = pitch - angle_pitch; // Angle difference
      float y_roll  = roll - angle_roll; // Angle difference
      /* Step 6 */
      angle_pitch += K_pitch[0] * y_pitch;
      bias_pitch += K_pitch[1] * y_pitch;
      angle_roll += K_roll[0] * y_roll;
      bias_roll += K_roll[1] * y_roll;

      // Calculate estimation error covariance - Update the error covariance
      /* Step 7 */
      float P00_temp_pitch = P_pitch[0][0];
      float P01_temp_pitch = P_pitch[0][1];

      float P00_temp_roll = P_roll[0][0];
      float P01_temp_roll = P_roll[0][1];

      P_pitch[0][0] -= K_pitch[0] * P00_temp_pitch;
      P_pitch[0][1] -= K_pitch[0] * P01_temp_pitch;
      P_pitch[1][0] -= K_pitch[1] * P00_temp_pitch;
      P_pitch[1][1] -= K_pitch[1] * P01_temp_pitch;

      P_roll[0][0] -= K_roll[0] * P00_temp_roll;
      P_roll[0][1] -= K_roll[0] * P01_temp_roll;
      P_roll[1][0] -= K_roll[1] * P00_temp_roll;
      P_roll[1][1] -= K_roll[1] * P01_temp_roll;

      
      osDelayUntil(10);
    }  
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
