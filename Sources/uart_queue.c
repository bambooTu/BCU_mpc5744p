/**
 * @file       uart_queue.c
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @brief      
 * @version    0.1
 * @date       2022-11-04
 * 
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 * 
 * Abbreviation: 
 * None
 */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_queue.h"

#include <stdio.h>
#include <string.h>

#include "commonly_used.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    UART_RECVING,
    UART_RECV_DONE,
    UART_RECV_STBY,
    UART_RECV_RST,
} APP_UART_STATUS_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
#define UART1
#define UART2
#define UART_RX_DONE_CNT 50
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// UART Variable
static UART_MSG_BUFF_t   txQueue[UART_NUMBER_OF_MODULE][UART_QUEUE_SIZE] = {};
static UART_MSG_BUFF_t   rxQueue[UART_NUMBER_OF_MODULE][UART_QUEUE_SIZE] = {};
static UART_MSG_BUFF_t   xferMessage[UART_NUMBER_OF_MODULE]              = {};
static UART_MSG_BUFF_t   recvMessage[UART_NUMBER_OF_MODULE]              = {};
static UART_OBJECT_t     Object[UART_NUMBER_OF_MODULE]                   = {};
static uint8_t           temp[UART_NUMBER_OF_MODULE];
static uint8_t           rxDoneCnt[UART_NUMBER_OF_MODULE];
static APP_UART_STATUS_e uartState[UART_NUMBER_OF_MODULE];
static uint32_t          uartTxRemaining = 0;
static uint32_t          uartRxRemaining = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief      UART1 Interrupt Callback
 *
 * @param      driverState
 * @param      event Define the enum of the events
 * @param      userData
 * @version    0.1
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @date       2022-11-04
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
void UART1_Callback(void* driverState, uart_event_t event, void* userData) {
    switch (event) {
        case UART_EVENT_RX_FULL:
            rxDoneCnt[UART_MODULE_1]                                          = UART_RX_DONE_CNT;
            uartState[UART_MODULE_1]                                          = UART_RECVING;
            recvMessage[UART_MODULE_1].data[recvMessage[UART_MODULE_1].dlc++] = temp[UART_MODULE_1];
            UART_ReceiveData(&uart_pal1_instance, &temp[UART_MODULE_1], 1);
            break;
        case UART_EVENT_TX_EMPTY:
            /* code */
            break;
        case UART_EVENT_END_TRANSFER:
            /* code */
            break;
        case UART_EVENT_ERROR:
            break;
        default:
            break;
    }
}
/**
 * @brief      UART2 Interrupt Callback
 *
 * @param      driverState
 * @param      event Define the enum of the events
 * @param      userData
 * @version    0.1
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @date       2022-11-04
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
void UART2_Callback(void* driverState, uart_event_t event, void* userData) {
    switch (event) {
        case UART_EVENT_RX_FULL:
            rxDoneCnt[UART_MODULE_2]                                          = UART_RX_DONE_CNT;
            uartState[UART_MODULE_2]                                          = UART_RECVING;
            recvMessage[UART_MODULE_2].data[recvMessage[UART_MODULE_2].dlc++] = temp[UART_MODULE_2];
            UART_ReceiveData(&uart_pal2_instance, &temp[UART_MODULE_2], 1);
            break;
        case UART_EVENT_TX_EMPTY:
            /* code */
            break;
        case UART_EVENT_END_TRANSFER:
            /* code */
            break;
        case UART_EVENT_ERROR:
            break;
        default:
            break;
    }
}

/**
 * @brief      Initialize the queue
 *
 * @param      instance UART Module number
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
static void UART_InitializeQueue(UART_MODULE_e instance) {
    /*UART Queue Initialize*/
    Object[instance].txQueue.pHead          = txQueue[instance];
    Object[instance].txQueue.pTail          = txQueue[instance];
    Object[instance].txQueue.Status.b.empty = true;
    Object[instance].txQueue.Status.b.full  = false;
    Object[instance].txQueue.Count          = 0;

    Object[instance].rxQueue.pHead          = rxQueue[instance];
    Object[instance].rxQueue.pTail          = rxQueue[instance];
    Object[instance].rxQueue.Status.b.empty = true;
    Object[instance].rxQueue.Status.b.full  = false;
    Object[instance].rxQueue.Count          = 0;
}

/**
 * @brief      Create a receive message link and initialize the queue
 *
 * @version    0.1
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @date       2022-11-04
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
void UART_Initialize(void) {
#if defined(UART1)
    UART_Init(&uart_pal1_instance, &uart_pal1_Config0);
    UART_SetBaudRate(&uart_pal1_instance, 115200);
    UART_ReceiveData(&uart_pal1_instance, &temp[UART_MODULE_1], 1);
    UART_InitializeQueue(UART_MODULE_1);
    uartState[UART_MODULE_1] = UART_RECV_STBY;
#endif
#if defined(UART2)
    UART_Init(&uart_pal2_instance, &uart_pal2_Config0);
    UART_SetBaudRate(&uart_pal2_instance, 115200);
    UART_ReceiveData(&uart_pal2_instance, &temp[UART_MODULE_2], 1);
    UART_InitializeQueue(UART_MODULE_2);
    uartState[UART_MODULE_2] = UART_RECV_STBY;
#endif
}

/**
 * @brief      Push message to txQueue
 *
 * @param      instance UART Module number
 * @param      txMessage The address of the message pulled from the queue
 * @return     uint8_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint8_t UART_PushTxQueue(UART_MODULE_e instance, UART_MSG_BUFF_t* txMessage) {
    // check if there is space in the queue
    if (Object[instance].txQueue.Status.b.full == false) {
        *Object[instance].txQueue.pTail = *txMessage;
        Object[instance].txQueue.pTail++;
        Object[instance].txQueue.Count++;
        // check if the end of the array is reached
        if (Object[instance].txQueue.pTail == (txQueue[instance] + UART_QUEUE_SIZE)) {
            // adjust to restart at the beginning of the array
            Object[instance].txQueue.pTail = txQueue[instance];
        }
        // since we added one item to be processed, we know
        // it is not empty, so set the empty status to false
        Object[instance].txQueue.Status.b.empty = false;
        // check if full
        if (Object[instance].txQueue.pHead == Object[instance].txQueue.pTail) {
            // it is full, set the full status to true
            Object[instance].txQueue.Status.b.full = true;
        }
    }
    return (Object[instance].txQueue.Status.b.full);
}

/**
 * @brief      Pull messages from txQueue
 *
 * @param      instance UART Module number
 * @param      txMessage The address of the message pulled from the queue
 * @return     uint8_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint8_t UART_PullTxQueue(UART_MODULE_e instance, UART_MSG_BUFF_t* txMessage) {
    if (Object[instance].txQueue.Status.b.empty != true) {
        *txMessage = *Object[instance].txQueue.pHead;
        Object[instance].txQueue.pHead++;
        Object[instance].txQueue.Count--;
        // check if the end of the array is reached
        if (Object[instance].txQueue.pHead == (txQueue[instance] + UART_QUEUE_SIZE)) {
            // adjust to restart at the beginning of the array
            Object[instance].txQueue.pHead = txQueue[instance];
        }
        // since we moved one item to be processed, we know
        // it is not full, so set the full status to false
        Object[instance].txQueue.Status.b.full = false;
        // check if the queue is empty
        if (Object[instance].txQueue.pHead == Object[instance].txQueue.pTail) {
            // it is empty so set the empty status to true
            Object[instance].txQueue.Status.b.empty = true;
        }
    }
    return (Object[instance].txQueue.Status.b.empty);
}

/**
 * @brief      Push message to rxQueue
 *
 * @param      instance UART Module number
 * @param      rxMessage The address of the received message The address of the
 * message pushed into the queue
 * @return     uint8_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint8_t UART_PushRxQueue(UART_MODULE_e instance, UART_MSG_BUFF_t* rxMessage) {
    // check if there is space in the queue
    if (Object[instance].rxQueue.Status.b.full == false) {
        *Object[instance].rxQueue.pTail = *rxMessage;
        Object[instance].rxQueue.pTail++;
        Object[instance].rxQueue.Count++;
        // check if the end of the array is reached
        if (Object[instance].rxQueue.pTail == (rxQueue[instance] + UART_QUEUE_SIZE)) {
            // adjust to restart at the beginning of the array
            Object[instance].rxQueue.pTail = rxQueue[instance];
        }
        // since we added one item to be processed, we know
        // it is not empty, so set the empty status to false
        Object[instance].rxQueue.Status.b.empty = false;
        // check if full
        if (Object[instance].rxQueue.pHead == Object[instance].rxQueue.pTail) {
            // it is full, set the full status to true
            Object[instance].rxQueue.Status.b.full = true;
        }
    }
    return (Object[instance].rxQueue.Status.b.full);
}

/**
 * @brief      Pull messages from rxQueue
 *
 * @param      instance UART Module number
 * @param      rxMessage The address of the received message The address of the
 * message pulled from the queue
 * @return     uint8_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint8_t UART_PullRxQueue(UART_MODULE_e instance, UART_MSG_BUFF_t* rxMessage) {
    if (Object[instance].rxQueue.Status.b.empty != true) {
        *rxMessage = *Object[instance].rxQueue.pHead;
        Object[instance].rxQueue.pHead++;
        Object[instance].rxQueue.Count--;
        // check if the end of the array is reached
        if (Object[instance].rxQueue.pHead == (rxQueue[instance] + UART_QUEUE_SIZE)) {
            // adjust to restart at the beginning of the array
            Object[instance].rxQueue.pHead = rxQueue[instance];
        }
        // since we moved one item to be processed, we know
        // it is not full, so set the full status to false
        Object[instance].rxQueue.Status.b.full = false;
        // check if the queue is empty
        if (Object[instance].rxQueue.pHead == Object[instance].rxQueue.pTail) {
            // it is empty so set the empty status to true
            Object[instance].rxQueue.Status.b.empty = true;
        }
    }
    return (Object[instance].rxQueue.Status.b.empty);
}

/**
 * @brief      Get txQueue count value
 *
 * @param      instance UART Module number
 * @return     uint32_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint32_t UART_GetTxQueueCount(UART_MODULE_e instance) {
    return (Object[instance].txQueue.Count);
}

/**
 * @brief      Get rxQueue count value
 *
 * @param      instance UART Module number
 * @return     uint32_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint32_t UART_GetRxQueueCount(UART_MODULE_e instance) {
    return (Object[instance].rxQueue.Count);
}

/**
 * @brief      Transfer queue data
 *
 * @param      instance UART Module number
 * @return     true
 * @return     false
 * @version    0.1
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @date       2022-10-24
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
bool UART_QueueDataXfer(UART_MODULE_e instance) {
    bool ret = false;
    if (UART_GetTransmitStatus(&uart_pal1_instance, &uartTxRemaining) != STATUS_BUSY) {
        if (UART_GetTxQueueCount(instance)) {
            UART_PullTxQueue(instance, &xferMessage[instance]);
            UART_SendData(&uart_pal1_instance, &xferMessage[instance].data[0], xferMessage[instance].dlc);
            ret = true;
        }
    }
    return ret;
}
/**
 * @brief      Receive queue data
 *
 * @version    0.1
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @date       2022-11-01
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
void UART_QueueDataRecv(void) {
#if defined(UART1)
    switch (uartState[UART_MODULE_1]) {
        case UART_RECVING:
            if (rxDoneCnt[UART_MODULE_1]) {
                rxDoneCnt[UART_MODULE_1]--;
            } else {
                uartState[UART_MODULE_1] = UART_RECV_DONE;
            }
            break;
        case UART_RECV_DONE:
            switch (UART_GetReceiveStatus(&uart_pal1_instance, &uartRxRemaining)) {
                case STATUS_UART_RX_OVERRUN:
                    uartState[UART_MODULE_1] = UART_RECV_RST;
                    break;
                case STATUS_UART_TX_UNDERRUN:
                case STATUS_UART_ABORTED:
                case STATUS_UART_FRAMING_ERROR:
                case STATUS_UART_PARITY_ERROR:
                case STATUS_UART_NOISE_ERROR:
                    break;
                default:
                    SATURATION(recvMessage[UART_MODULE_1].dlc, 65535, 1);
                    UART_PushRxQueue(UART_MODULE_1, &recvMessage[UART_MODULE_1]);
                    uartState[UART_MODULE_1] = UART_RECV_STBY;
                    break;
            }
            break;
        case UART_RECV_STBY:
            recvMessage[UART_MODULE_1].dlc = 0;
            break;
        case UART_RECV_RST:
            UART_ReceiveData(&uart_pal1_instance, &temp, 1);
            rxDoneCnt[UART_MODULE_1] = UART_RX_DONE_CNT;
            break;
        default:
            break;
    }
    if (UART_GetRxQueueCount(UART_MODULE_1)) {
        UART_PullRxQueue(UART_MODULE_1, &recvMessage[UART_MODULE_1]);
        /* USER CODE BEGING */

        /* USER CODE END */
    }
#endif
#if defined(UART2)
    switch (uartState[UART_MODULE_2]) {
        case UART_RECVING:
            if (rxDoneCnt[UART_MODULE_2]) {
                rxDoneCnt[UART_MODULE_2]--;
            } else {
                uartState[UART_MODULE_2] = UART_RECV_DONE;
            }
            break;
        case UART_RECV_DONE:
            switch (UART_GetReceiveStatus(&uart_pal2_instance, &uartRxRemaining)) {
                case STATUS_UART_RX_OVERRUN:
                    uartState[UART_MODULE_2] = UART_RECV_RST;
                    break;
                case STATUS_UART_TX_UNDERRUN:
                case STATUS_UART_ABORTED:
                case STATUS_UART_FRAMING_ERROR:
                case STATUS_UART_PARITY_ERROR:
                case STATUS_UART_NOISE_ERROR:
                    break;
                default:
                    SATURATION(recvMessage[UART_MODULE_2].dlc, 65535, 1);
                    UART_PushRxQueue(UART_MODULE_2, &recvMessage[UART_MODULE_2]);
                    uartState[UART_MODULE_2] = UART_RECV_STBY;
                    break;
            }
            break;
        case UART_RECV_STBY:
            recvMessage[UART_MODULE_2].dlc = 0;
            break;
        case UART_RECV_RST:
            UART_ReceiveData(&uart_pal2_instance, &temp, 1);
            rxDoneCnt[UART_MODULE_2] = UART_RX_DONE_CNT;
            break;
        default:
            break;
    }

    if (UART_GetRxQueueCount(UART_MODULE_2)) {
        UART_PullRxQueue(UART_MODULE_2, &recvMessage[UART_MODULE_2]);
        /* USER CODE BEGING */
        UART_PushTxQueue(UART_MODULE_1, &recvMessage[UART_MODULE_2]);
        /* USER CODE END */
    }
#endif
}
/* USER CODE END 0 */
/*******************************************************************************
 End of File
 */
