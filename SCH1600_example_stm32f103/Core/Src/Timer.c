//****************************************************************************************
// @file    Timer.c
// @brief   Timer related functions.
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
#include "Timer.h"


/**
 * Internal function prototypes
 */          
static void (*timer_callback)(void);  // Sampling timer callback function.


/**
  * @brief  Sampling timer initialization
  *
  * @param  function - callback function for data sampling via SPI.
  * @param  freq - desired sampling frequency, valid range is 1 ... 10000 Hz
  *
  * @return true if given frequency is valid, false otherwise.
  */
bool sample_timer_init(void function(void), uint32_t freq)
{
    
    if ((freq < 1) || (freq > 10000))
        return false;
    
    timer_callback = function;    // Set data sampling callback function.
    
    // Set sample timer frequency.
   // hw_timer_setFreq(freq);
    
    hw_timer_startIT();     // Start sample timer interrupt
    
    return true;
}


/**
 * @brief Sampling timer start
 *
 * @param None
 * 
 * @return None                       
 */
void sample_timer_start(void)
{
     hw_timer_startIT();     // Start sample timer interrupt
}


/**
 * @brief Sampling timer stop
 *
 * @param None
 * 
 * @return None                       
 */
void sample_timer_stop(void)
{
    hw_timer_stopIT();     // Stop sample timer interrupt
}


/**
  * @brief  Sampling timer interrupt handler
  *
  * @param  htim - pointer to timer handle
  *
  * @return None
  */



