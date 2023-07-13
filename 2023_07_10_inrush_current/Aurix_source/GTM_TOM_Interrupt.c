/**********************************************************************************************************************
 * \file GTM_TOM_Interrupt.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 *
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are solely in the form of
 * machine-executable object code generated by a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "IfxPort.h"
#include "IfxGtm_Tom_Timer.h"
#include "Bsp.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define ISR_PRIORITY_TOM1    3                       /* Interrupt priority number                                    */
#define ISR_PRIORITY_TOM2    2                       /* Interrupt priority number                                    */

#define TOM_FREQ            4.0f                    /* TOM frequency                                                */
#define TOM_FREQ2           8.0f                    /* TOM frequency                                                */

#define LED                 &MODULE_P00, 5          /* LED which will be toggled in Interrupt Service Routine (ISR)  &MODULE_P00, 5, test2*/
#define Test_PIN1           &MODULE_P02, 0
#define Test_PIN1_MID       &MODULE_P02, 1

#define Test_PIN2           &MODULE_P02, 3
//#define Test_PIN2_MID       &MODULE_P02, 5

#define WAIT_TIME           1                       /* 1ms*/


int flag1 = 0;
int flag2 = 0;

int interrupt = 0;

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxGtm_Tom_Timer g_timerDriver1;                     /* TOM driver                                                   */
IfxGtm_Tom_Timer g_timerDriver2;                     /* TOM driver                                                   */

/*********************************************************************************************************************/
/*--------------------------------------------Function Implementations-----------------------------------------------*/
/*********************************************************************************************************************/
/* Macro to define the Interrupt Service Routine. */
IFX_INTERRUPT(interruptGtmTom1, 0, ISR_PRIORITY_TOM1);

IFX_INTERRUPT(interruptGtmTom2, 0, ISR_PRIORITY_TOM2);

void make_500ms_pulse_high(void){
    IfxPort_setPinState(Test_PIN1  ,  IfxPort_State_high);
//    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
}

void make_500ms_pulse_low(void){
    IfxPort_setPinState(Test_PIN1  ,  IfxPort_State_low);
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
}

void make_500ms_pulse_mid(void){
    IfxPort_setPinState(Test_PIN1_MID  ,  IfxPort_State_high);
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
    IfxPort_setPinState(Test_PIN1_MID  ,  IfxPort_State_low);
}

void make_pulse_interrupt(void){
    IfxPort_setPinState(Test_PIN1_MID  ,  IfxPort_State_high);
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
    IfxPort_setPinState(Test_PIN1_MID  ,  IfxPort_State_low);
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
}


/* Interrupt Service Routine of the TOM */
void interruptGtmTom1(void)
{
    IfxGtm_Tom_Timer_acknowledgeTimerIrq(&g_timerDriver1);                       /* Clear the timer event            */
    //IfxPort_togglePin(LED);                                                     /* Toggle the LED                   */

    if(flag1 % 4 == 0){
        make_500ms_pulse_high();
        for (int i = 0 ; i < 10 ; i++){
                make_pulse_interrupt();
        }
        flag1++;
    }
    else if(flag1 % 4 == 1){
        make_500ms_pulse_mid();
        flag1++;
    }
    else if(flag1 % 4 == 2){
        make_500ms_pulse_low();
        flag1++;
    }
    else{
        flag1++;
    }
}

/* This function initializes the TOM */
void initGtmTom1(void)
{
    IfxGtm_enable(&MODULE_GTM); /* Enable GTM */

    IfxGtm_Tom_Timer_Config timerConfig1;                                        /* Timer configuration              */
    IfxGtm_Tom_Timer_initConfig(&timerConfig1, &MODULE_GTM);                     /* Initialize timer configuration   */

    timerConfig1.base.frequency       = TOM_FREQ;                                /* Set timer frequency              */
    timerConfig1.base.isrPriority     = ISR_PRIORITY_TOM1;                        /* Set interrupt priority           */
    timerConfig1.base.isrProvider     = IfxSrc_Tos_cpu0;                         /* Set interrupt provider           */
    timerConfig1.tom                  = IfxGtm_Tom_1;                            /* Define the timer used            */
    timerConfig1.timerChannel         = IfxGtm_Tom_Ch_0;                         /* Define the channel used          */
    timerConfig1.clock                = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk3;          /* Define the CMU clock used        */

    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);               /* Enable the CMU clock             */
    IfxGtm_Tom_Timer_init(&g_timerDriver1, &timerConfig1);                        /* Initialize the TOM               */

    //IfxPort_setPinModeOutput(LED, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);  /* Set pin mode         */
    IfxPort_setPinModeOutput(Test_PIN1, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);  /* Set pin mode         */
    IfxPort_setPinModeOutput(Test_PIN1_MID, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);  /* Set pin mode         */

    IfxGtm_Tom_Timer_run(&g_timerDriver1); /* Start the TOM */
}


void make_200ms_pulse_high(void){
    IfxPort_setPinState(Test_PIN2  ,  IfxPort_State_high);
//    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
}

void make_200ms_pulse_low(void){
    IfxPort_setPinState(Test_PIN2  ,  IfxPort_State_low);
    //waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
}

void make_200ms_pulse_mid(void){
    IfxPort_setPinState(Test_PIN1_MID  ,  IfxPort_State_high);
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
    IfxPort_setPinState(Test_PIN1_MID  ,  IfxPort_State_low);
}



/* Interrupt Service Routine of the TOM */
void interruptGtmTom2(void)
{
    IfxGtm_Tom_Timer_acknowledgeTimerIrq(&g_timerDriver2);                       /* Clear the timer event            */
    //IfxPort_togglePin(LED);                                                     /* Toggle the LED                   */

    if(flag2 % 8 == 1){
        waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
        waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
        make_200ms_pulse_high();
        flag2++;
    }

    else if(flag2 % 8 == 2){
            make_200ms_pulse_mid();
            flag2++;
    }

    else if(flag2 % 8 == 3){
                make_200ms_pulse_low();
             flag2++;
    }
    else{
        flag2++;
    }
}

/* This function initializes the TOM */
void initGtmTom2(void)
{
    IfxGtm_enable(&MODULE_GTM); /* Enable GTM */

    IfxGtm_Tom_Timer_Config timerConfig2;                                        /* Timer configuration              */
    IfxGtm_Tom_Timer_initConfig(&timerConfig2, &MODULE_GTM);                     /* Initialize timer configuration   */

    timerConfig2.base.frequency       = TOM_FREQ2;                                /* Set timer frequency              */
    timerConfig2.base.isrPriority     = ISR_PRIORITY_TOM2;                        /* Set interrupt priority           */
    timerConfig2.base.isrProvider     = IfxSrc_Tos_cpu0;                         /* Set interrupt provider           */
    timerConfig2.tom                  = IfxGtm_Tom_1;                            /* Define the timer used            */
    timerConfig2.timerChannel         = IfxGtm_Tom_Ch_8;                         /* Define the channel used          */
    timerConfig2.clock                = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk3;          /* Define the CMU clock used        */

    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);               /* Enable the CMU clock             */
    IfxGtm_Tom_Timer_init(&g_timerDriver2, &timerConfig2);                        /* Initialize the TOM               */

    //IfxPort_setPinModeOutput(LED, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);  /* Set pin mode         */
    IfxPort_setPinModeOutput(Test_PIN2, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);  /* Set pin mode         */
    IfxPort_setPinModeOutput(Test_PIN1_MID, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);  /* Set pin mode         */

    IfxGtm_Tom_Timer_run(&g_timerDriver2); /* Start the TOM */
}