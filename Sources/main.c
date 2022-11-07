/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/* ###################################################################
 **     Filename    : main.c
 **     Project     : flexcan_mpc5744p
 **     Processor   : MPC5744P_144
 **     Version     : Driver 01.00
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2017-11-07, 18:04, # CodeGen: 3
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 01.00
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */
#include "Cpu.h"
#include "can.h"
#include "canCom1.h"
#include "clockMan1.h"
#include "commonly_used.h"
#include "dmaController1.h"
#include "pin_mux.h"
#include "uart_queue.h"
#if CPU_INIT_CONFIG
#include "Init_Config.h"
#endif
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_PORT  PTC
#define LED0      11U
#define LED1      12U
#define LED2      13U
#define BTN_PORT  PTF
#define BTN0_PIN  12U
#define BTN1_PIN  13U
#define BTN0_EIRQ 30U
#define BTN1_EIRQ 31U
/* Use this define to specify if the application runs as master or slave */
#define MASTER
/* #define SLAVE */
/* Definition of the TX and RX message buffers depending on the bus role */
#if defined(MASTER)
#define TX_MAILBOX (1UL)
#define TX_MSG_ID  (1UL)
#define RX_MAILBOX (0UL)
#define RX_MSG_ID  (0x18000000UL)
#elif defined(SLAVE)
#define TX_MAILBOX (0UL)
#define TX_MSG_ID  (2UL)
#define RX_MAILBOX (1UL)
#define RX_MSG_ID  (1UL)
#endif

#define MAX_DUTY_VAL         8000
#define CALCULTAE_TIME_MS(A) ((A < 1) ? 10 - 1 : 10 * A - 1)
/******************************************************************************
 * Private typedef
 ******************************************************************************/
typedef struct {
    unsigned int cnt;
    bool         flag;
} TMR_DATA_t;

typedef enum {
    APP_STATE_EEPROM_READ = 0,
    APP_STATE_INIT,
    APP_STATE_SERVICE_TASKS,
    APP_STATE_PWROFF_TASKS,
    APP_STATE_SYSTEM_OFF,
} APP_STATUS_e;
/******************************************************************************
 * Global variables
 ******************************************************************************/
struct {
    TMR_DATA_t _1ms;
    TMR_DATA_t _5ms;
    TMR_DATA_t _10ms;
    TMR_DATA_t _20ms;
    TMR_DATA_t _100ms;
    TMR_DATA_t _500ms;
} tmrData;

struct {
    uint32_t         period;
    uint32_t         duty;
    ic_option_mode_t state;
    bool             xferDone;
} captureData;

struct {
    APP_STATUS_e   state;
    unsigned       mainPWR : 1;
    unsigned short bootTimeCount;
} appData;

volatile bool     groupConvDone    = false;
volatile uint32_t resultLastOffset = 0;
uint16_t          dutyCycle        = MAX_DUTY_VAL;
uint32_t          retResult        = 0U;
UART_MSG_BUFF_t   uartTxMsg;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void BTN_IRQCallback(void);
void SYS_Initialize(void);
void GPIO_Initialize(void);
/******************************************************************************
 * Functions
 ******************************************************************************/
void PIT_Ch0_IRQHandler(void) {
    PIT_DRV_ClearStatusFlags(INST_PIT1, pit1_ChnConfig0.hwChannel); /* Clear channel 0 interrupt flag */
    if (tmrData._1ms.cnt++ >= CALCULTAE_TIME_MS(1)) {
        tmrData._1ms.cnt  = 0;
        tmrData._1ms.flag = true;
    }
    if (tmrData._5ms.cnt++ >= CALCULTAE_TIME_MS(5)) {
        tmrData._5ms.cnt  = 0;
        tmrData._5ms.flag = true;
    }
    if (tmrData._10ms.cnt++ >= CALCULTAE_TIME_MS(10)) {
        tmrData._10ms.cnt  = 0;
        tmrData._10ms.flag = true;
    }
    if (tmrData._20ms.cnt++ >= CALCULTAE_TIME_MS(20)) {
        tmrData._20ms.cnt  = 0;
        tmrData._20ms.flag = true;
    }
    if (tmrData._100ms.cnt++ >= CALCULTAE_TIME_MS(100)) {
        tmrData._100ms.cnt  = 0;
        tmrData._100ms.flag = true;
    }
    if (tmrData._500ms.cnt++ >= CALCULTAE_TIME_MS(500)) {
        tmrData._500ms.cnt  = 0;
        tmrData._500ms.flag = true;
    }
    if (appData.bootTimeCount) {
        appData.bootTimeCount--;
    }
}

void adc_pal1_callback00(const adc_callback_info_t* const callbackInfo, void* userData) {
    (void)userData;
    groupConvDone    = true;
    resultLastOffset = callbackInfo->resultBufferTail;
}
void ic_pal1_callback00(ic_event_t event, void* userData) {
    static uint8_t captureCnt = 0;

    captureCnt++;
    captureData.xferDone = true;
}
void BTN_IRQCallback(void) {
    /* Check if one of the buttons was pressed */
    bool              button0     = PINS_DRV_GetPinExIntFlag(BTN0_EIRQ);
    bool              button1     = PINS_DRV_GetPinExIntFlag(BTN1_EIRQ);
    static uint8_t    ISR_counter = 0;
    flexcan_msgbuff_t canTxMsg;

    canTxMsg.msgId   = 0x18FF4520;
    canTxMsg.dataLen = 8;
    canTxMsg.data[0] = ISR_counter;
    canTxMsg.data[1] = 0;
    canTxMsg.data[2] = 0;
    canTxMsg.data[3] = 0;
    canTxMsg.data[4] = 0;
    canTxMsg.data[5] = 0;
    canTxMsg.data[6] = 0;
    canTxMsg.data[7] = 0;

    /* Set FlexCAN TX value according to the button pressed */
    if (button0 != 0) {
        ISR_counter++;
        CAN_PushTxQueue(CAN_MODULE_1, &canTxMsg);
        /* Clear interrupt flag */
        PINS_DRV_ClearPinExIntFlag(BTN0_EIRQ);
    } else if (button1 != 0) {
        ISR_counter--;
        CAN_PushTxQueue(CAN_MODULE_1, &canTxMsg);
        /* Clear interrupt flag */
        PINS_DRV_ClearPinExIntFlag(BTN1_EIRQ);
    } else {
        PINS_DRV_ClearExIntFlag();
    }
}
/*
 * @brief : Initialize clocks, pins and power modes
 */
void SYS_Initialize(void) {
    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   Configure FlexCAN clock, GPIO, LPSPI
     *  -   see clock manager component for more details
     */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);
    /* Initialize DMA */
    EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, &edmaChnStateArray, &edmaChnConfigArray, 2);
    EDMA_DRV_StartChannel(dmaController1Chn0_Config.virtChnConfig);
    EDMA_DRV_StartChannel(dmaController1Chn1_Config.virtChnConfig);
    /* Initialize pins */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
    /* Initialize periodic interrupt timer */
    PIT_DRV_InitChannel(INST_PIT1, &pit1_ChnConfig0);
    PIT_DRV_StartChannel(INST_PIT1, 0U);
    /* Initializes given instance of the CAN peripheral. */
    FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
    /* Initializes given instance of the ADC peripheral */
    ADC_Init(&adc_pal1_instance, &adc_pal1_InitConfig0);
    ADC_StartGroupConversion(&adc_pal1_instance, 0);
    /* Initializes given instance of the FlexPWM peripheral */
    FLEXPWM_DRV_SetupPwm(INST_FLEXPWM1, 0U, &flexPWM1_flexpwm_module_setup_t0,
                         &flexPWM1_flexpwm_module_signal_setup_t0);
    FLEXPWM_DRV_CounterStart(INST_FLEXPWM1, 0U);
    FLEXPWM_DRV_UpdatePulseWidth(INST_FLEXPWM1, 0U, 300, 0UL, FlexPwmEdgeAligned);
    FLEXPWM_DRV_LoadCommands(INST_FLEXPWM1, (1UL << 0));
    /* Initializes given instance of the input capture peripheral */
    IC_Init(&ic_pal1_instance, &ic_pal1_InitConfig);
    IC_SetChannelMode(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId, IC_MEASURE_PULSE_HIGH);
    captureData.state = IC_MEASURE_PULSE_HIGH;
    IC_EnableNotification(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId);
    IC_StartChannel(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId);
}
/*
 * @brief Function which configures the LEDs and Buttons
 */
void GPIO_Initialize(void) {
    /* Set Output value LEDs */
    PINS_DRV_SetPins(LED_PORT, (1 << LED0) | (1 << LED1) | (1 << LED2));
    SIUL2->IMCR[203] |= SIUL2_IMCR_SSS(1U);
    SIUL2->IMCR[204] |= SIUL2_IMCR_SSS(1U);
    /* Install buttons ISR */
    INT_SYS_InstallHandler(SIUL_EIRQ_24_31_IRQn, &BTN_IRQCallback, NULL);
    /* Enable buttons interrupt */
    INT_SYS_EnableIRQ(SIUL_EIRQ_24_31_IRQn);
}
volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */
/*!
 \brief The main function for the project.
 \details The startup initialization sequence is the following:
 * - __start (startup asm routine)
 * - __init_hardware()
 * - main()
 *   - PE_low_level_init()
 *     - Common_Init()
 *     - Peripherals_Init()
 */
int main(void) {
    /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
#ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT(); /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
#endif
    /*** End of Processor Expert internal initialization. ***/
    /* Do the initializations required for this application */
    flexcan_msgbuff_t canTxMsg;

    SYS_Initialize();
    GPIO_Initialize();
    CAN_Initialize();
    UART_Initialize();

    appData.state = APP_STATE_EEPROM_READ;
    while (1) {
        switch ((APP_STATUS_e)appData.state) {
            case APP_STATE_EEPROM_READ:
                appData.bootTimeCount = CALCULTAE_TIME_MS(100);
                appData.state         = APP_STATE_INIT;
                break;
            case APP_STATE_INIT:

                if (!appData.bootTimeCount) {
                    appData.state = APP_STATE_SERVICE_TASKS;
                }
                break;
            case APP_STATE_SERVICE_TASKS:
                CAN_QueueDataXfer(CAN_MODULE_1);
                CAN_QueueDataRecv();
                UART_QueueDataXfer(UART_MODULE_1);
                UART_QueueDataRecv();
                if (tmrData._1ms.flag) {
                    tmrData._1ms.flag = false;
                    if (groupConvDone) {
                        groupConvDone    = false;
                        canTxMsg.msgId   = 0x16000001;
                        canTxMsg.dataLen = 8;
                        canTxMsg.data[0] = LOW_BYTE(adc_pal1_Results00[0]);
                        canTxMsg.data[1] = HIGH_BYTE(adc_pal1_Results00[0]);
                        canTxMsg.data[2] = 0;
                        canTxMsg.data[3] = 0;
                        canTxMsg.data[4] = 0;
                        canTxMsg.data[5] = 0;
                        canTxMsg.data[6] = 0;
                        canTxMsg.data[7] = 0;
                        CAN_PushTxQueue(CAN_MODULE_1, &canTxMsg);
                    }

                    if (dutyCycle) {
                        dutyCycle--;
                    } else {
                        dutyCycle = MAX_DUTY_VAL;
                    }
                    FLEXPWM_DRV_UpdatePulseWidth(INST_FLEXPWM1, 0U, dutyCycle, MAX_DUTY_VAL - dutyCycle,
                                                 FlexPwmEdgeAligned);
                    FLEXPWM_DRV_LoadCommands(INST_FLEXPWM1, (1UL << 0));
                    UART_QueueDataRecv();
                }

                if (tmrData._5ms.flag) {
                    tmrData._5ms.flag = false;
                }

                if (tmrData._10ms.flag) {
                    tmrData._10ms.flag = false;
                    if (captureData.xferDone) {
                        captureData.xferDone = false;
                        switch (captureData.state) {
                            case IC_MEASURE_PULSE_HIGH:
                                IC_StopChannel(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId);
                                captureData.duty = IC_GetMeasurement(&ic_pal1_instance,
                                                                     ic_pal1_InitConfig.inputChConfig[0].hwChannelId);
                                IC_SetChannelMode(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId,
                                                  IC_MEASURE_PULSE_HIGH);
                                captureData.state = IC_MEASURE_RISING_EDGE_PERIOD;
                                IC_StartChannel(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId);
                                break;
                            case IC_MEASURE_RISING_EDGE_PERIOD:
                                IC_StopChannel(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId);
                                captureData.period = IC_GetMeasurement(&ic_pal1_instance,
                                                                       ic_pal1_InitConfig.inputChConfig[0].hwChannelId);

                                IC_SetChannelMode(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId,
                                                  IC_MEASURE_RISING_EDGE_PERIOD);
                                captureData.state = IC_MEASURE_PULSE_HIGH;
                                IC_StartChannel(&ic_pal1_instance, ic_pal1_InitConfig.inputChConfig[0].hwChannelId);
                                break;
                            default:
                                break;
                        }
                        canTxMsg.msgId   = 0x16000000;
                        canTxMsg.dataLen = 8;
                        canTxMsg.data[0] = LOW_BYTE(LOW_WORD(captureData.duty));
                        canTxMsg.data[1] = HIGH_BYTE(LOW_WORD(captureData.duty));
                        canTxMsg.data[2] = LOW_BYTE(HIGH_WORD(captureData.duty));
                        canTxMsg.data[3] = HIGH_BYTE(HIGH_WORD(captureData.duty));
                        canTxMsg.data[4] = LOW_BYTE(LOW_WORD(captureData.period));
                        canTxMsg.data[5] = HIGH_BYTE(LOW_WORD(captureData.period));
                        canTxMsg.data[6] = LOW_BYTE(HIGH_WORD(captureData.period));
                        canTxMsg.data[7] = HIGH_BYTE(HIGH_WORD(captureData.period));
                        CAN_PushTxQueue(CAN_MODULE_1, &canTxMsg);
                    }
                }

                if (tmrData._20ms.flag) {
                    tmrData._20ms.flag = false;
                }

                if (tmrData._100ms.flag) {
                    tmrData._100ms.flag = false;

                    // sprintf(uartTxMsg.data, "integer: %d\n", dutyCycle);
                    // uartTxMsg.dlc = strlen(uartTxMsg.data);
                    // UART_PushTxQueue(UART_MODULE_1, &uartTxMsg);
                }

                if (tmrData._500ms.flag) {
                    tmrData._500ms.flag = false;
                    PINS_DRV_TogglePins(LED_PORT, (1 << LED0));
                }
                break;
            case APP_STATE_PWROFF_TASKS:
                /* code */
                break;
            case APP_STATE_SYSTEM_OFF:
                /* code */
                break;
            default:
                break;
        }
    }
    /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
/*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
#ifdef PEX_RTOS_START
    PEX_RTOS_START(); /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
#endif
    /*** End of RTOS startup code.  ***/
    /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
    for (;;) {
        if (exit_code != 0) {
            break;
        }
    }
    return exit_code;
    /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/
/* END main */
/*!
 ** @}
 */
/*
 ** ###################################################################
 **
 **     This file was created by Processor Expert 10.1 [05.21]
 **     for the NXP C55 series of microcontrollers.
 **
 ** ###################################################################
 */
