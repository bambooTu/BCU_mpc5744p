/**
 * @file       uart_queue.h
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
#ifndef _UART_H
#define _UART_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

/* Global define -------------------------------------------------------------*/
/* USER CODE BEGIN GD */

/* USER CODE END GD */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Cpu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    UART_MODULE_1,
    UART_MODULE_2,
} UART_MODULE_e;

typedef union {
    struct {
        uint8_t full     : 1;
        uint8_t empty    : 1;
        uint8_t reserved : 6;
    } b;
    uint8_t byte;
} UART_QUEUE_STATUS_t;

typedef struct {
    uint32_t dlc;
    uint8_t  data[255];
} UART_MSG_BUFF_t;

typedef struct {
    UART_MSG_BUFF_t*    pHead;
    UART_MSG_BUFF_t*    pTail;
    UART_QUEUE_STATUS_t Status;
    uint32_t            Count;
} UART_QUEUE_t;

typedef struct {
    UART_QUEUE_t txQueue;
    UART_QUEUE_t rxQueue;
    uint16_t     errorCount;
} UART_OBJECT_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_NUMBER_OF_MODULE 2
#define UART_QUEUE_SIZE       8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void UART_Initialize(void);

uint8_t UART_PushTxQueue(UART_MODULE_e instance, UART_MSG_BUFF_t* txMessage);
uint8_t UART_PushRxQueue(UART_MODULE_e instance, UART_MSG_BUFF_t* rxMessage);
uint8_t UART_PullTxQueue(UART_MODULE_e instance, UART_MSG_BUFF_t* txMessage);
uint8_t UART_PullRxQueue(UART_MODULE_e instance, UART_MSG_BUFF_t* rxMessage);

uint32_t UART_GetTxQueueCount(UART_MODULE_e instance);
uint32_t UART_GetRxQueueCount(UART_MODULE_e instance);

bool UART_QueueDataXfer(UART_MODULE_e instance);
void UART_QueueDataRecv(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _UART_H */

/*******************************************************************************
 End of File
 */
