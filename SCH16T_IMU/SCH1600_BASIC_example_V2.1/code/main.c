//******************************************************************************
// Murata SCH1600 sensor family basic demonstrator.
//
// This software is released under the BSD license as follows.
// Copyright (c) 2024, Murata Electronics Oy.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following 
// conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
//    3. Neither the name of Murata Electronics Oy nor the names of its    
//       contributors may be used to endorse or promote products derived    
//       from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//***************************************************************************************************
//
// Revision history
//
// 2022 Dec 9                V1.0         Initial release. 48-bit SPI-frames used for all operations.
//
// 2024 Jan 17               V2.0         Modified to use STM32F4xx HAL.
//
// 2024 Jun 1                V2.1         Added support for filter bypass mode. Replaced use of   
//                                        HAL_Delay() with hw_delay() in SCH1.c.
//                                        Corrected REQ_SOFTRESET request frame.
//                                                
//***************************************************************************************************

#include "main.h"
#include <stdio.h>
#include <string.h>

#include "hw.h"
#include "SCH1.h"
#include "Timer.h"


// Sampling_callback stores summed raw data. Ring buffer is used for storing 
// the data to reduce probability of main() missing out samples.
#define RING_BUFFER_SIZE    4   // Amount of samples to store in ring buffer

// stdout_init is required for IO-redirection in Keil uVision, see 
// https://www.keil.com/pack/doc/compiler/RetargetIO/html/_retarget__examples__u_a_r_t.html
extern int stdout_init (void);


TIM_HandleTypeDef htim2;        // htim2 (TIM2) is used as sampling timer for SCH1600
static volatile bool SCH1_error_available = false;
static SCH1_raw_data SCH1_summed_data_buffer[RING_BUFFER_SIZE];
static uint32_t ring_buffer_idx_wr = 0;
static uint32_t ring_buffer_idx_rd = 0;


// Function prototypes
static void SystemClock_Config(void);
static void sampling_callback(void);
static bool new_summed_data_available(void);
static bool inc_ring_buffer_wr_idx(void);
static void inc_ring_buffer_rd_idx(void);


/**
  * @brief  Main program
  *
  * @param  None
  *
  * @return None
  */
int main(void)
{
    char serial_num[15];
    int  init_status;
    SCH1_filter         Filter;
    SCH1_sensitivity    Sensitivity;
    SCH1_decimation     Decimation;

    // STM32F4xx HAL initialization
    HAL_Init();

    // Configure system clock to 80 MHz.
    SystemClock_Config();
    SystemCoreClockUpdate();
        
    // Initialize IO-redirection
    stdout_init();

    // Initialize demo board HW (STM Nucleo-F401RE)
    hw_init();
    
    
    // SCH1600 settings and initialization
    //------------------------------------
    
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
        printf("ERROR: SCH1_init failed with code: %d\r\nApplication halted\r\n", init_status);
        HAL_Delay(10);
        while (true);           
    }
  
    // Read serial number from the sensor.
    strcpy(serial_num, SCH1_getSnbr());
    printf("Serial number: %s\r\n\r\n", serial_num);        

    // Start sampling timer at 1000 Hz
    //--------------------------------
    
    // With 1000 Hz sample rate and 10x averaging we get Output Data Rate (ODR) of 100 Hz.
    sample_timer_init(sampling_callback, 1000);
    
    // Main loop
    while (true)
    {
        // Check if sampling_callback has new summed raw data for us to process.
        if (new_summed_data_available()) {
            // Start critical section

            // Convert summed raw data from the ring buffer & calculate averages.
            SCH1_result Data;
            SCH1_convert_data(&SCH1_summed_data_buffer[ring_buffer_idx_rd], &Data);
                        
            // End critical section
            inc_ring_buffer_rd_idx();
                                    
            // Send converted data to UART
            printf("GYRO(X,Y,Z)[dps]:%+.3f,%+.3f,%+.3f\t"
                   "ACC(X,Y,Z)[m/s2]:%+.3f,%+.3f,%+.3f\t"
                   "T[C]:%+.1f\r\n",
                   Data.Rate1[AXIS_X], Data.Rate1[AXIS_Y], Data.Rate1[AXIS_Z],
                   Data.Acc1[AXIS_X], Data.Acc1[AXIS_Y], Data.Acc1[AXIS_Z], 
                   Data.Temp);
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
            printf("STATUS:\tSUM:%4x\tSUM_SAT:%4x\tCOM:%4x\t"
                   "RATE_COM:%4x\tRATE_X:%4x\tRATE_Y:%4x\tRATE_Z:%4x\t"
                   "ACC_X:%4x\tACC_Y:%4x\tACC_Z:%4x\r\n",     
                Status.Summary, Status.Summary_Sat, Status.Common,    
                Status.Rate_Common, Status.Rate_X, Status.Rate_Y, Status.Rate_Z,
                Status.Acc_X, Status.Acc_Y, Status.Acc_Z);
            
            // Restart sampling timer
            sample_timer_start();
        }     
    } // while (true)
}


/**
 * @brief Callback function for data sampling via SPI. Called from timer interrupt handler.
 *
 * @param None
 * 
 * @return None
 */
static void sampling_callback(void)
{        

    static SCH1_raw_data SCH1_data;
    static uint32_t num_samples = 0;
    
    SCH1_getData(&SCH1_data);
    
    // Result averaging is done in main(), here we just sum up the raw data from sensor.
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate1_raw[AXIS_X] += SCH1_data.Rate1_raw[AXIS_X];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate1_raw[AXIS_Y] += SCH1_data.Rate1_raw[AXIS_Y];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Rate1_raw[AXIS_Z] += SCH1_data.Rate1_raw[AXIS_Z];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc1_raw[AXIS_X]  += SCH1_data.Acc1_raw[AXIS_X];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc1_raw[AXIS_Y]  += SCH1_data.Acc1_raw[AXIS_Y];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Acc1_raw[AXIS_Z]  += SCH1_data.Acc1_raw[AXIS_Z];
    SCH1_summed_data_buffer[ring_buffer_idx_wr].Temp_raw          += SCH1_data.Temp_raw;
    
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


/**
 * @brief  Configure SystemCoreClock using HSI (HSE is not populated on Nucleo board)
 *         System clock frequency = 80 MHz. 
 *         80 MHz SYSCLK is used to achieve 10 MHz SPI SCK frequency (See file hw.c function SPI_Init()).
 *
 * @param  None
 *
 * @return None
  */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initialize system clock
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;    // HSI is the system clock
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;          // PLL source is HSI
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 80;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                   // SYSCLK = 16 MHz / 8 * 80 / 2 = 80 MHz
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while(1) {}
    }

    /** Initialize CPU, AHB and APB bus clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        while(1) {}
    }

    /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported */
    if (HAL_GetREVID() == 0x1001)
    {
        /* Enable the Flash prefetch */
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}
