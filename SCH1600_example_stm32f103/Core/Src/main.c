/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hw.h"
#include "SCH1.h"
#include "Timer.h"
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RING_BUFFER_SIZE    4   // Amount of samples to store in ring buffer
SCH1_result DataSCH16;
// stdout_init is required for IO-redirection in Keil uVision, see
// https://www.keil.com/pack/doc/compiler/RetargetIO/html/_retarget__examples__u_a_r_t.html


    // htim2 (TIM2) is used as sampling timer for SCH1600
static volatile bool SCH1_error_available = false;
 SCH1_raw_data SCH1_summed_data_buffer[RING_BUFFER_SIZE];
 uint32_t ring_buffer_idx_wr = 0;
 uint32_t ring_buffer_idx_rd = 0;


// Function prototypes
static void SystemClock_Config(void);
static void sampling_callback(void);
static bool new_summed_data_available(void);
static bool inc_ring_buffer_wr_idx(void);
static void inc_ring_buffer_rd_idx(void);

typedef struct{
	int64_t Ax;
	int64_t Ay;
	int64_t Az;
	int64_t Gx;
	int64_t Gy;
	int64_t Gz;
	int64_t temp;
}Axis_t;

Axis_t SCH16TData;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char serial_num[15];
uint8_t flgTimer = 0;
SCH1_result Data;
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

	    int  init_status;
	    SCH1_filter         Filter;
	    SCH1_sensitivity    Sensitivity;
	    SCH1_decimation     Decimation;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Initialize IO-redirection
      //stdout_init();

      // Initialize demo board HW (STM Nucleo-F401RE)
      hw_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // SCH1600 filter settings
     Filter.Rate12 = FILTER_RATE;
     Filter.Acc12  = FILTER_ACC12;
     Filter.Acc3   = FILTER_ACC3;

     // SCH1600 sensitivity settings
     Sensitivity.Rate1 = SENSITIVITY_RATE1;
     Sensitivity.Rate2 = SENSITIVITY_RATE2;
     Sensitivity.Acc1  = SENSITIVITY_ACC1;
     Sensitivity.Acc2  = SENSITIVITY_ACC2;
     Sensitivity.Acc3  = SENSITIVITY_ACC3;

     // SCH1600 decimation settings (for Rate2 and Acc2 channels).
     Decimation.Rate2 = DECIMATION_RATE;
     Decimation.Acc2  = DECIMATION_ACC;

     // Initialize the sensor
     init_status = SCH1_init(Filter, Sensitivity, Decimation, false);
     if (init_status != SCH1_OK) {
      //   printf("ERROR: SCH1_init failed with code: %d\r\nApplication halted\r\n", init_status);
         HAL_Delay(10);
         while (true);
     }

      //Read serial number from the sensor.
      strcpy(serial_num, SCH1_getSnbr());
   //  printf("Serial number: %s\r\n\r\n", serial_num);

     // Start sampling timer at 1000 Hz
     //--------------------------------

     // With 1000 Hz sample rate and 10x averaging we get Output Data Rate (ODR) of 100 Hz.
      HAL_TIM_Base_Start_IT(&htim2);
     // sample_timer_init(sampling_callback, 1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Check if sampling_callback has new summed raw data for us to process.
	          if (new_summed_data_available())
	          {
	              // Start critical section

	              // Convert summed raw data from the ring buffer & calculate averages.

	              SCH1_convert_data(&SCH1_summed_data_buffer[ring_buffer_idx_rd], &Data);
	             // SCH1_convert_data(&SCH1_summed_data_buffer[ring_buffer_idx_rd], &DataSCH16);

	        	  //SCH1_convert_data(&SCH1_summed_data_buffer[ring_buffer_idx_rd], &DataSCH16);


	              // End critical section
	              inc_ring_buffer_rd_idx();



	              // Send converted data to UART
	              /*printf("GYRO(X,Y,Z)[dps]:%+.3f,%+.3f,%+.3f\t"
	                     "ACC(X,Y,Z)[m/s2]:%+.3f,%+.3f,%+.3f\t"
	                     "T[C]:%+.1f\r\n",
	                     Data.Rate1[AXIS_X], Data.Rate1[AXIS_Y], Data.Rate1[AXIS_Z],
	                     Data.Acc1[AXIS_X], Data.Acc1[AXIS_Y], Data.Acc1[AXIS_Z],
	                     Data.Temp);*/
	          }

	          // Handle possible sensor errors
	          if (SCH1_error_available) {

	              // Error from sensor. Stop sample timer to avoid race condition when reading status.
	              sample_timer_stop();
	              SCH1_error_available = false;

	              // Read SCH1600 status registers
	              SCH1_status Status;
	              SCH1_getStatus(&Status);

	              // Send status data to UART. Status registers are read in sampling_callback()
	             /* printf("STATUS:\tSUM:%4x\tSUM_SAT:%4x\tCOM:%4x\t"
	                     "RATE_COM:%4x\tRATE_X:%4x\tRATE_Y:%4x\tRATE_Z:%4x\t"
	                     "ACC_X:%4x\tACC_Y:%4x\tACC_Z:%4x\r\n",
	                  Status.Summary, Status.Summary_Sat, Status.Common,
	                  Status.Rate_Common, Status.Rate_X, Status.Rate_Y, Status.Rate_Z,
	                  Status.Acc_X, Status.Acc_Y, Status.Acc_Z);*/

	              // Restart sampling timer
	              sample_timer_start();
	          }

	          if(flgTimer)
	          {
	        	  flgTimer = 0;
	        	  sampling_callback();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

static void sampling_callback(void)
{

	static SCH1_raw_data SCH1_data;
	static SCH1_raw_data SCH1_data_2;
    static uint32_t num_samples = 0;

    SCH1_getData(&SCH1_data);
    SCH1_getData_2(&SCH1_data_2);

    // Result averaging is done in main(), here we just sum up the raw data from sensor.
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate1_raw[AXIS_X] += SCH1_data.Rate1_raw[AXIS_X];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate1_raw[AXIS_Y] += SCH1_data.Rate1_raw[AXIS_Y];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate1_raw[AXIS_Z] += SCH1_data.Rate1_raw[AXIS_Z];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc1_raw[AXIS_X]  += SCH1_data.Acc1_raw[AXIS_X];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc1_raw[AXIS_Y]  += SCH1_data.Acc1_raw[AXIS_Y];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc1_raw[AXIS_Z]  += SCH1_data.Acc1_raw[AXIS_Z];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Temp_raw          += SCH1_data.Temp_raw;

    // Result averaging is done in main(), here we just sum up the raw data from sensor.
       SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate2_raw[AXIS_X] += SCH1_data_2.Rate2_raw[AXIS_X];
       SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate2_raw[AXIS_Y] += SCH1_data_2.Rate2_raw[AXIS_Y];
       SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate2_raw[AXIS_Z] += SCH1_data_2.Rate2_raw[AXIS_Z];
       SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc2_raw[AXIS_X]  += SCH1_data_2.Acc2_raw[AXIS_X];
       SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc2_raw[AXIS_Y]  += SCH1_data_2.Acc2_raw[AXIS_Y];
       SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc2_raw[AXIS_Z]  += SCH1_data_2.Acc2_raw[AXIS_Z];
       SCH1_summed_data_buffer[ring_buffer_idx_wr].Temp_raw          += SCH1_data_2.Temp_raw;

    if (SCH1_data.frame_error)
        SCH1_error_available = true;

    if (++num_samples >= AVG_FACTOR) {
        num_samples = 0;
        if (inc_ring_buffer_wr_idx() == true) {
            // Clear data at next write index of summed raw data
            memset(&SCH1_summed_data_buffer[ring_buffer_idx_wr], 0x00, sizeof(SCH1_summed_data_buffer) / RING_BUFFER_SIZE);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {


    // htim2 (TIM2) is the sampling timer
    if (htim == &htim2)
    {
    	flgTimer = 1;
    }
    	  // Call sampling function
}
/**
 * @brief Check if there is new data available for main() to process, by comparing ring buffer
 *        read and write pointers. Called by main()
 *
 * @param None
 *
 * @return true = new data available
 *         false = no new data available
 */
static bool new_summed_data_available(void)
{
    volatile uint32_t wr_idx_tmp = ring_buffer_idx_wr;
    if (wr_idx_tmp != ring_buffer_idx_rd)
        return true;

    return false;
}


/**
 * @brief Increment summed data ring buffer write pointer. Called by sampling_callback().
 *
 * @param None
 *
 * @return true = successfully incremented pointer
 *         false = couldn't increment pointer due to main() being late
 */
static bool inc_ring_buffer_wr_idx(void)
{
    volatile uint32_t new_idx = ring_buffer_idx_wr;

    new_idx++;
    if (new_idx >= RING_BUFFER_SIZE)
        new_idx = 0;

    // Don't add data to ring buffer if there is no free space
    if (new_idx == ring_buffer_idx_rd) {
        return false;
    }

    ring_buffer_idx_wr = new_idx;
    return true;
}


/**
 * @brief Increment summed data ring buffer read pointer. Called by main()
 *
 * @param None
 *
 * @return None
 */
static void inc_ring_buffer_rd_idx(void)
{
    volatile uint32_t new_idx = ring_buffer_idx_rd;

    new_idx++;
    if (new_idx >= RING_BUFFER_SIZE)
        new_idx = 0;

    ring_buffer_idx_rd = new_idx;
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
