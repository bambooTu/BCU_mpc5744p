/**
 * @file       can.c
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @brief
 * @version    0.1
 * @date       2022-08-18
 *
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 *
 * Abbreviation:
 * None
 */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
#define RX_MAILBOX (0UL)
#define TX_MAILBOX (1UL)
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// typedef union {
//     struct {
//         uint32_t cs;
//         union {
//             struct {  // J1939 Protocol
//                 uint8_t sourceAddress : 8;
//                 uint8_t pduSpecific   : 8;
//                 uint8_t pduFormat     : 8;
//                 uint8_t dataPage      : 1;
//                 uint8_t reserved      : 1;
//                 uint8_t priority      : 3;
//             } id_frame;
//             uint32_t id;
//         };
//         uint8_t data[64];
//         uint8_t dlc;
//     } J1939;
//     flexcan_msgbuff_t msgbuff;
// } CAN_MSG_t;

// CAN Variable
static flexcan_msgbuff_t txQueue[CAN_NUMBER_OF_MODULE][CAN_QUEUE_SIZE], rxQueue[CAN_NUMBER_OF_MODULE][CAN_QUEUE_SIZE];
static flexcan_msgbuff_t recvMessage[CAN_NUMBER_OF_MODULE] = {};
static can_object_t      Object[CAN_NUMBER_OF_MODULE]      = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief      CAN1 Interrupt Callback
 *
 * @param      contextHandle CAN event enumeration
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
static void CAN1_Callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t buffIdx,
                          flexcan_state_t* flexcanState) {
    switch (eventType) {
        case FLEXCAN_EVENT_RX_COMPLETE:
            FLEXCAN_DRV_Receive(instance, RX_MAILBOX, &recvMessage[instance]);
            CAN_PushRxQueue(instance, &recvMessage[instance]);
            break;
        default:
            break;
    }
}
/**
 * @brief      CAN2 Interrupt Callback
 *
 * @param      contextHandle CAN event enumeration
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
static void CAN2_Callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t buffIdx,
                          flexcan_state_t* flexcanState) {
    switch (eventType) {
        case FLEXCAN_EVENT_RX_COMPLETE:
            FLEXCAN_DRV_Receive(instance, RX_MAILBOX, &recvMessage[instance]);
            CAN_PushRxQueue(instance, &recvMessage[instance]);
            break;
        default:
            break;
    }
}
/**
 * @brief      CAN3 Interrupt Callback
 *
 * @param      instance CAN module enumeration
 * @param      eventType CAN event enumeration
 * @param      buffIdx
 * @param      flexcanState
 * @version    0.1
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @date       2022-10-31
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
static void CAN3_Callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t buffIdx,
                          flexcan_state_t* flexcanState) {
    switch (eventType) {
        case FLEXCAN_EVENT_RX_COMPLETE:
            FLEXCAN_DRV_Receive(instance, RX_MAILBOX, &recvMessage[instance]);
            CAN_PushRxQueue(instance, &recvMessage[instance]);
            break;
        default:
            break;
    }
}
/**
 * @brief      Initialize the queue
 *
 * @param      instance CAN Module number
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
static void CAN_InitializeQueue(CAN_MODULE_e instance) {
    /*CAN Queue Initialize*/
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
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
void CAN_Initialize(void) {
    flexcan_data_info_t dataInfo = {.data_length = 8U, .msg_id_type = FLEXCAN_MSG_ID_EXT};
    FLEXCAN_DRV_InstallEventCallback(CAN_MODULE_1, CAN1_Callback, NULL);
    FLEXCAN_DRV_ConfigRxMb(CAN_MODULE_1, RX_MAILBOX, &dataInfo, 0x1fffffff);
    FLEXCAN_DRV_SetRxMaskType(CAN_MODULE_1, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(CAN_MODULE_1, FLEXCAN_MSG_ID_EXT, RX_MAILBOX, 0x00000000);
    // FLEXCAN_DRV_InstallErrorCallback(CAN_MODULE_1, CAN1_Callback, NULL);

    // FLEXCAN_DRV_InstallEventCallback(CAN_MODULE_2, CAN2_Callback, NULL);
    // FLEXCAN_DRV_ConfigRxMb(CAN_MODULE_2, RX_MAILBOX, &dataInfo, 0x1fffffff);
    // FLEXCAN_DRV_SetRxMaskType(CAN_MODULE_2, FLEXCAN_RX_MASK_INDIVIDUAL);
    // FLEXCAN_DRV_SetRxIndividualMask(CAN_MODULE_2, FLEXCAN_MSG_ID_EXT, RX_MAILBOX, 0x00000000);
    // FLEXCAN_DRV_InstallErrorCallback(CAN_MODULE_2, CAN2_Callback, NULL);

    // FLEXCAN_DRV_InstallErrorCallback(CAN_MODULE_3, CAN3_Callback, NULL);
    // FLEXCAN_DRV_ConfigRxMb(CAN_MODULE_3, RX_MAILBOX, &dataInfo, 0x1fffffff);
    // FLEXCAN_DRV_SetRxMaskType(CAN_MODULE_3, FLEXCAN_RX_MASK_INDIVIDUAL);
    // FLEXCAN_DRV_SetRxIndividualMask(CAN_MODULE_3, FLEXCAN_MSG_ID_EXT, RX_MAILBOX, 0x00000000);
    // FLEXCAN_DRV_InstallEventCallback(CAN_MODULE_3, CAN3_Callback, NULL);

    //
    for (CAN_MODULE_e instance = CAN_MODULE_1; instance < CAN_NUMBER_OF_MODULE; instance++) {
        CAN_InitializeQueue(instance);
        FLEXCAN_DRV_Receive(instance, RX_MAILBOX, &recvMessage[instance]);
    }
}

/**
 * @brief      Push message to txQueue
 *
 * @param      instance CAN Module number
 * @param      txMessage The address of the message pulled from the queue
 * @return     uint8_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint8_t CAN_PushTxQueue(CAN_MODULE_e instance, flexcan_msgbuff_t* txMessage) {
    // check if there is space in the queue
    if (Object[instance].txQueue.Status.b.full == false) {
        *Object[instance].txQueue.pTail = *txMessage;
        Object[instance].txQueue.pTail++;
        Object[instance].txQueue.Count++;
        // check if the end of the array is reached
        if (Object[instance].txQueue.pTail == (txQueue[instance] + CAN_QUEUE_SIZE)) {
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
 * @param      instance CAN Module number
 * @param      txMessage The address of the message pulled from the queue
 * @return     uint8_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint8_t CAN_PullTxQueue(CAN_MODULE_e instance, flexcan_msgbuff_t* txMessage) {
    if (Object[instance].txQueue.Status.b.empty != true) {
        *txMessage = *Object[instance].txQueue.pHead;
        Object[instance].txQueue.pHead++;
        Object[instance].txQueue.Count--;
        // check if the end of the array is reached
        if (Object[instance].txQueue.pHead == (txQueue[instance] + CAN_QUEUE_SIZE)) {
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
 * @param      instance CAN Module number
 * @param      rxMessage The address of the received message The address of the
 * message pushed into the queue
 * @return     uint8_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint8_t CAN_PushRxQueue(CAN_MODULE_e instance, flexcan_msgbuff_t* rxMessage) {
    // check if there is space in the queue
    if (Object[instance].rxQueue.Status.b.full == false) {
        *Object[instance].rxQueue.pTail = *rxMessage;
        Object[instance].rxQueue.pTail++;
        Object[instance].rxQueue.Count++;
        // check if the end of the array is reached
        if (Object[instance].rxQueue.pTail == (rxQueue[instance] + CAN_QUEUE_SIZE)) {
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
 * @param      instance CAN Module number
 * @param      rxMessage The address of the received message The address of the
 * message pulled from the queue
 * @return     uint8_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint8_t CAN_PullRxQueue(CAN_MODULE_e instance, flexcan_msgbuff_t* rxMessage) {
    if (Object[instance].rxQueue.Status.b.empty != true) {
        *rxMessage = *Object[instance].rxQueue.pHead;
        Object[instance].rxQueue.pHead++;
        Object[instance].rxQueue.Count--;
        // check if the end of the array is reached
        if (Object[instance].rxQueue.pHead == (rxQueue[instance] + CAN_QUEUE_SIZE)) {
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
 * @param      instance CAN Module number
 * @return     uint32_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint32_t CAN_GetTxQueueCount(CAN_MODULE_e instance) {
    return (Object[instance].txQueue.Count);
}

/**
 * @brief      Get rxQueue count value
 *
 * @param      instance CAN Module number
 * @return     uint32_t
 * @version    0.1
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @date       2022-08-18
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
uint32_t CAN_GetRxQueueCount(CAN_MODULE_e instance) {
    return (Object[instance].rxQueue.Count);
}

/**
 * @brief      Transfer queue data
 *
 * @param      instance CAN Module number
 * @return     true
 * @return     false
 * @version    0.1
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @date       2022-10-24
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 */
bool CAN_QueueDataXfer(CAN_MODULE_e instance) {
    bool                ret = false;
    flexcan_msgbuff_t   canTxMsg;
    flexcan_data_info_t dataInfo = {.data_length = 8, .msg_id_type = FLEXCAN_MSG_ID_EXT};

    if (FLEXCAN_DRV_GetTransferStatus(instance, TX_MAILBOX) == STATUS_SUCCESS) {
        if (CAN_GetTxQueueCount(instance)) {
            CAN_PullTxQueue(instance, &canTxMsg);
            /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
            FLEXCAN_DRV_ConfigTxMb(instance, TX_MAILBOX, &dataInfo, canTxMsg.msgId);
            /* Execute send non-blocking */
            FLEXCAN_DRV_Send(instance, TX_MAILBOX, &dataInfo, canTxMsg.msgId, canTxMsg.data);
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
void CAN_QueueDataRecv(void) {
    flexcan_msgbuff_t canRxMsg;
#if (CAN_NUMBER_OF_MODULE >= 1)
    if (CAN_GetRxQueueCount(CAN_MODULE_1)) {
        CAN_PullRxQueue(CAN_MODULE_1, &canRxMsg);
        /* USER CODE BEGING */
        PINS_DRV_TogglePins(PTC, (1 << 12));
        /* USER CODE END */
    }
#elif (CAN_NUMBER_OF_MODULE >= 2)
    if (CAN_GetRxQueueCount(CAN_MODULE_2)) {
        CAN_PullRxQueue(CAN_MODULE_2, &canRxMsg);
        /* USER CODE BEGING */
       
        /* USER CODE END */
    }
#elif (CAN_NUMBER_OF_MODULE == 3)
    if (CAN_GetRxQueueCount(CAN_MODULE_3)) {
        CAN_PullRxQueue(CAN_MODULE_3, &canRxMsg);
        /* USER CODE BEGING */
        
        /* USER CODE END */
    }
#endif
}
/* USER CODE END 0 */
/*******************************************************************************
 End of File
 */
