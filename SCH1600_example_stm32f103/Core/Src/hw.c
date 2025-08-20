//****************************************************************************************
// @file    hw.c
// @brief   Hardware related functions.
//
// @attention
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
//****************************************************************************************

#include "hw.h"
#include "usart.h"
#include "spi.h"



// Internal function prototypes
static void GPIO_init(void);
static void UART_Init(void);
static void SPI_Init(void);
static void TIM_Init(void);


/**
  * @brief  Hardware initialization
  *
  * @param  None
  *
  * @return None
  */
void hw_init() {

    GPIO_init();    // IO-pin initializations
    UART_Init();    // UART channel for PC connection (IO-redirection)
    SPI_Init();     // SPI channel for SCH1600
    //TIM_Init();     // SCH1600 sampling timer initialization (TIM2)
    
    // Wait for power supply to stabilize
    HAL_Delay(100);
}


/**
  * @brief  GPIO initialization for SCH1600
  *
  * @param  None
  *
  * @return None
  */
static void GPIO_init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // EXTRESN pin
    GPIO_InitStruct.Pin   = EXTRESN_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(EXTRESN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(EXTRESN_PORT, EXTRESN_PIN, GPIO_PIN_SET); // Set EXTRESN inactive (high)

    // TA9 pin
    GPIO_InitStruct.Pin   = TA9_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TA9_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(TA9_PORT, TA9_PIN, GPIO_PIN_RESET);       // Set TA9 = 0

    // TA8 pin
    GPIO_InitStruct.Pin   = TA8_PIN;                                 
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TA8_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(TA8_PORT, TA8_PIN, GPIO_PIN_RESET);       // Set TA8 = 0
    
    // DRY_SYNC pin
    GPIO_InitStruct.Pin   = GPIO_PIN_9;                                 
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                        // DRY_SYNC pin shall be left floating if not used.
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/**
  * @brief  UART initialization (USART2 is used for connection to PC)
  *
  * @param  None
  *
  * @return None
  */
static void UART_Init(void)
{
    // Serial bus parameters 460800�8-N-1
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 460800;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        while(1) {}
    }
}


/**
  * @brief  SPI bus initialization (SPI1 is used for SCH1600). Also configures SPI pins.
  *
  * @param  None
  *
  * @return None
  */
static void SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_SPI1_CLK_ENABLE(); 
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // GPIO Init for CS pin
    // Software chip select is used for SPI CS
  /*  GPIO_InitStruct.Pin   = SPI1_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI1_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPI1_CS_PORT, SPI1_CS_PIN, GPIO_PIN_SET); */   // Set SPI CS inactive (high)

    // GPIO Init for SCK, MISO and MOSI pins
    // PA5     ------> SPI1_SCK
    // PA6     ------> SPI1_MISO
    // PA7     ------> SPI1_MOSI
    GPIO_InitStruct.Pin = SPI1_SCK_PIN|SPI1_MISO_PIN|SPI1_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         // SPI1 Alternate Function mapping
    HAL_GPIO_Init(SPI1_PORT, &GPIO_InitStruct);
    
    // SPI1 initialization
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_16BIT;               // 16-bit transfer
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;              // - SPI-mode 0
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;                  // /
    hspi1.Init.NSS = SPI_NSS_SOFT;                          // Software chip select
    // SYSCLK = 80 MHz, prescaler sets SPI 
    // SCK frequency to 10 MHz (80 MHz / 8)
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;                 // Shift MSB out first.
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        while(1) {}
    }
}


/**
  * @brief  Sampling timer initialization (TIM2 is the sampling timer)
  *
  * @param  None
  *
  * @return None
  */
void TIM_Init(void)
{

    TIM_ClockConfigTypeDef  sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    // Enable TIM2 peripheral clock
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Initialize sampling timer (TIM2) to initial 100 Hz.
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 80;  // APB1 clock is 80 MHz, timer prescaler is used to divide it to 1 MHz.
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 10000;  // 1 MHz is divided by 10000 to get 100 Hz timer interrupt. This is
                                // just an initial value and can be modified later with hw_timer_setFreq()
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        while(1) {}
    }
    // Set timer clock source
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;      // Timer clock source is internal clock (APB1).
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        while(1) {}
    }
    // Set trigger output
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;    // Trigger output is disabled.
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        while(1) {}
    }
    
    // Set TIM2 global interrupt priority to 1 with sub-priority 0.
    // SYSTICK timer priority is 0 so TIM2 needs to have lower priority for not to disturb SYSTICK.
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    
    // Enable TIM2 global interrupt.
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}


/**
  * @brief  Set SCH1600 EXTRESN pin high
  *
  * @param  None
  *
  * @return None
  */
void hw_EXTRESN_High(void)
{
    HAL_GPIO_WritePin(EXTRESN_PORT, EXTRESN_PIN, GPIO_PIN_SET);
}


/**
  * @brief  Set SCH1600 EXTRESN pin low
  *
  * @param  None
  *
  * @return None
  */
void hw_EXTRESN_Low(void)
{
    HAL_GPIO_WritePin(EXTRESN_PORT, EXTRESN_PIN, GPIO_PIN_RESET);   
}


/**
  * @brief  Set SPI chip select high
  *
  * @param  None
  *
  * @return None
  */
void hw_CS_High(void)
{
    HAL_GPIO_WritePin(SPI_CS_PIN_GPIO_Port, SPI_CS_PIN_Pin, GPIO_PIN_SET);
}


/**
  * @brief  Set SPI chip select low
  *
  * @param  None
  *
  * @return None
  */
void hw_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI_CS_PIN_GPIO_Port, SPI_CS_PIN_Pin, GPIO_PIN_RESET);
}


/**
 * @brief  Hardware delay function
 *                
 * @params ms - Delay time in milliseconds
 *
 * @return None
 */
void hw_delay(uint32_t ms)
{
    HAL_Delay(ms);
}


/**
  * @brief  Send 48bit request to SPI device, receive 48bit response simultaneously (off-frame protocol).
  *
  * @param  Request - 48 bit MOSI data
  *
  * @return ReceivedData - 48 bit MISO data
  */

uint64_t hw_SPI48_Send_Request(uint64_t Request)
{

    uint64_t ReceivedData = 0;
    uint16_t txBuffer[3];
    uint16_t rxBuffer[3];
    uint8_t index;
    uint8_t size = 3;   // 48-bit SPI-transfer consists of three 16-bit transfers.
    
    // Split Request qword (MOSI data) to tx buffer.
    for (index = 0; index < size; index++)
    {
        txBuffer[size - index - 1] = (Request >> (index << 4)) & 0xFFFF;
    }

    // Send tx buffer and receive rx buffer simultaneously.
    hw_CS_Low();
    HAL_Delay(5);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)txBuffer, (uint8_t*)rxBuffer, size, 10);
    HAL_Delay(5);
    hw_CS_High();
    HAL_Delay(5);

    // Create ReceivedData qword from received rx buffer (MISO data).
    for (index = 0; index < size; index++)
    {
    	ReceivedData |= (uint64_t)rxBuffer[index] << ((size - index - 1) << 4);
    }

    return ReceivedData;
}


/**
  * @brief  Set sampling timer frequency
  *
  * @param  freq - Sampling frequency
  *
  * @return None
  */
void hw_timer_setFreq(uint32_t freq)
{
    // Set sample timer frequency. MCU APB1-bus frequency that clocks  
    // the timer TIM2 is set to 1 MHz in HW.c function TIM_Init().
    htim2.Init.Period = 1000000 / freq;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) 
        while(1) {}
}


/**
  * @brief  Stop sampling timer interrupts
  *
  * @param  None
  *
  * @return None
  */
void hw_timer_stopIT(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
}


/**
  * @brief  Start sampling timer interrupts
  *
  * @param  None
  *
  * @return None
  */
void hw_timer_startIT(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}
