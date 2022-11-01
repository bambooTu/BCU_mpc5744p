/**
 * @file       can.h
 * @author     Chiou (charlie.chiou@amitatech.com)
 * @brief      CAN API
 * @version    0.1
 * @date       2022-08-18
 *
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 *
 * Abbreviation:
 * PCS             Power Control System
 * GC              Grid Control
 * BMU             Battery Management Unit
 * BMS             Battery Management System
 * MBMS            Master BMS
 * PF              PDU Formt
 */

#ifndef _CAN_H
#define _CAN_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

/* Global define -------------------------------------------------------------*/
/* USER CODE BEGIN GD */
#define SYSPARAM_CAN_DATA_LENGTH 8
// define J1939 structure size
#define J1939_MSG_LENGTH         5
#define J1939_DATA_LENGTH        8

// define J1939 queue buffer size
#define J1939_RX1_QUEUE_SIZE 50
#define J1939_TX1_QUEUE_SIZE 50
#define J1939_RX2_QUEUE_SIZE 50
#define J1939_TX2_QUEUE_SIZE 50
#define J1939_RX3_QUEUE_SIZE 50
#define J1939_TX3_QUEUE_SIZE 50
#define J1939_RX4_QUEUE_SIZE 50
#define J1939_TX4_QUEUE_SIZE 50

// J1939 Default Priorities
#define J1939_CONTROL_PRIORITY     0x03
#define J1939_INFO_PRIORITY        0x06
#define J1939_PROPRIETARY_PRIORITY 0x06
#define J1939_REQUEST_PRIORITY     0x06
#define J1939_ACK_PRIORITY         0x06
#define J1939_TP_CM_PRIORITY       0x07
#define J1939_TP_DT_PRIORITY       0x07

// J1939 Defined Addresses
#define J1939_GLOBAL_ADDRESS 255
#define J1939_NULL_ADDRESS   254

// Some J1939 PDU Formats, Control Bytes, and PGN's
#define J1939_PF_REQUEST2 201
#define J1939_PF_TRANSFER 202

#define J1939_PF_ACKNOWLEDGMENT           232
#define J1939_ACK_CONTROL_BYTE            0
#define J1939_NACK_CONTROL_BYTE           1
#define J1939_ACCESS_DENIED_CONTROL_BYTE  2
#define J1939_CANNOT_RESPOND_CONTROL_BYTE 3

#define J1939_PF_REQUEST 234

#define J1939_PF_DT 235  // Data Transfer message

#define J1939_PF_TP_CM               236  // Connection Management message
#define J1939_RTS_CONTROL_BYTE       16   // Request to Send control byte of CM message
#define J1939_CTS_CONTROL_BYTE       17   // Clear to Send control byte of CM message
#define J1939_EOMACK_CONTROL_BYTE    19   // End of Message control byte of CM message
#define J1939_BAM_CONTROL_BYTE       32   // BAM control byte of CM message
#define J1939_CONNABORT_CONTROL_BYTE 255  // Connection Abort control byte of CM message

#define J1939_PGN2_REQ_ADDRESS_CLAIM 0x00
#define J1939_PGN1_REQ_ADDRESS_CLAIM 0xEA
#define J1939_PGN0_REQ_ADDRESS_CLAIM 0x00

#define J1939_PGN2_COMMANDED_ADDRESS 0x00
#define J1939_PGN1_COMMANDED_ADDRESS 0xFE  // (-81 PDU Format)
#define J1939_PGN0_COMMANDED_ADDRESS 0xD8  // (-81 PDU Specific)

#define J1939_PF_ADDRESS_CLAIMED      238  // With global address
#define J1939_PF_CANNOT_CLAIM_ADDRESS 238  // With null address
#define J1939_PF_PROPRIETARY_A        239
#define J1939_PF_PROPRIETARY_B        255
/* USER CODE END GD */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Cpu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    CL_DEVICE_SLAVE = 0x05,  // Controlled devices such as BMS Host, BMS, MBMS,
    // parallel controller, Slave device etc.
    CL_DEVICE_MASTER,  // PCS, GC, Master device...etc.
} AMITA_CTRL_DEVICE_e;

typedef enum {
    CMD_DATA_GET             = 0xEA,  // Request for response information
    CMD_SET_COMM_ADDR        = 0xEF,  // Change mailing address
    CMD_SET_PROD_MFG_DATE    = 0x11,  // Set Product manufacturing date
    CMD_SET_PROD_SN1         = 0x12,  // Set Product serial number1
    CMD_SET_PROD_SN2         = 0x13,  // Set Product serial number2
    CMD_WR_PARAM_TO_EEPROM   = 0x15,  // Execute write EEPROM(command:0x84 ~ 0x8F)
    CMD_EX_CTRL_CMD          = 0x1A,  // Execute control commands
    CMD_ENTER_ENGR_MODE      = 0x1E,  // Request to switch to engineering mode
    CMD_EXT_TIME             = 0x1F,  // Requesting "Engineering Mode" extension time
    CMD_SET_CURR_PARAM       = 0x75,  // Set Current protection parameters
    CMD_SET_VOLT_PARAM       = 0x76,  // Set Voltage protection parameters
    CMD_SET_TEMP_PARAM       = 0x77,  // Set Temperature protection parameters
    CMD_SET_OTHERS_PARAM     = 0x7A,  // Set Other types of protection parameters
    CMD_SET_CURR_CALIB_PARAM = 0x8F,  // Current Sensor Correction Parameters
    CMD_RMT_CTRL_RLY         = 0x90,  // Remote control(Relay)
    CMD_SET_BAL_DATA_PARAM   = 0xBC,  // Set Balance Data parameter(BQ76930)
    CMD_ENTER_BOOTLOADER     = 0x1C,  // Enter Bootloader mode
} AMITA_CMD_e;

typedef enum {
    PF_GET_COMM_VER           = 0x01,  // Get communication protocol version
    PF_GET_MFG_DATE           = 0x11,  // Get product manufacturing date
    PF_GET_SEED               = 0x02,  // Get the Seed
    PF_GET_PROD_SN            = 0x12,  // Get product serial number
    PF_GET_SW_HW_VER          = 0x14,  // Get hardware and firmware version
    PF_GET_CURR_GRP_PARAM     = 0x75,  // Set current protection parameters
    PF_GET_VOLT_GRP_PARAM     = 0x76,  // Set voltage protection parameters
    PF_GET_TEMP_GRP_PARAM     = 0x77,  // Set temperature protection parameters
    PF_GET_OTHER_GRP_PARAM    = 0x7A,  // Set other types of protection parameters
    PF_GET_CURR_CALIB_PARAM   = 0x8F,  // Get current sensor parameters
    PF_GET_SYS_PARAM_G1       = 0x91,  // Get system parameters(PCS Only)
    PF_GET_SYS_PARAM_G2       = 0x92,
    PF_GET_SYS_PARAM_G3       = 0x94,
    PF_GET_SYS_PARAM_G4       = 0x95,
    PF_GET_CELL_DATA_G1       = 0xA0,  // Get the BMU temperature reading and the voltage value of each Cell
    PF_GET_CELL_DATA_G2       = 0xA8,
    PF_GET_CELL_DATA_G3       = 0xA9,
    PF_GET_BALANCE_DATA       = 0xBC,  // Get the currently executed Balance Data
    PF_GET_COULOMB_GAUGE_DATA = 0xCB,  // Read gas Gauge
    PF_GET_DTC_LOG_DATA       = 0xDC,  // Log fault code message log
    PF_GET_DTC_FLAG           = 0xD8,  // Get fault message status flag
} AMITA_PF_e;

typedef struct {
    union {
        struct {  // J1939 Protocol
            uint8_t sourceAddress : 8;
            uint8_t pduSpecific   : 8;
            uint8_t pduFormat     : 8;
            uint8_t dataPage      : 1;
            uint8_t reserved      : 1;
            uint8_t priority      : 3;
        } J1939;

        struct {  // Amita Protocol
            uint8_t     sourceAddress : 8;
            uint8_t     destAddress   : 8;
            AMITA_CMD_e command       : 8;
            uint8_t     dataPage      : 1;
            uint8_t     reserved      : 1;
            AMITA_CTRL_DEVICE_e class : 3;
        } Amita;
        uint32_t id;
    };
    uint8_t  dlc;
    uint8_t  data[J1939_DATA_LENGTH];
    uint32_t timestamp;
} CAN_MSG_t;

typedef enum {
    CAN_EVENT_ERR,
    CAN_EVENT_RX,
    CAN_EVENT_TX,
} CAN_EVENT_e;

typedef enum {
    CAN_MODULE_1,
    CAN_MODULE_2,
    CAN_MODULE_3,
} CAN_MODULE_e;

typedef union {
    struct {
        uint8_t full     : 1;
        uint8_t empty    : 1;
        uint8_t reserved : 6;
    } b;
    uint8_t byte;
} CAN_QUEUE_STATUS_t;

typedef struct {
    flexcan_msgbuff_t* pHead;
    flexcan_msgbuff_t* pTail;
    CAN_QUEUE_STATUS_t Status;
    uint32_t           Count;
} CAN_QUEUE_t;

typedef struct {
    CAN_QUEUE_t txQueue;
    CAN_QUEUE_t rxQueue;
    uint16_t    errorCount;
} can_object_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_NUMBER_OF_MODULE 1
#define CAN_QUEUE_SIZE       8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void CAN_Initialize(void);

uint8_t CAN_PushTxQueue(CAN_MODULE_e Instance, flexcan_msgbuff_t* txMessage);
uint8_t CAN_PushRxQueue(CAN_MODULE_e Instance, flexcan_msgbuff_t* txMessage);
uint8_t CAN_PullTxQueue(CAN_MODULE_e Instance, flexcan_msgbuff_t* txMessage);
uint8_t CAN_PullRxQueue(CAN_MODULE_e Instance, flexcan_msgbuff_t* txMessage);

uint32_t CAN_GetTxQueueCount(CAN_MODULE_e Instance);
uint32_t CAN_GetRxQueueCount(CAN_MODULE_e Instance);

bool CAN_QueueDataXfer(CAN_MODULE_e Instance);
void CAN_QueueDataRecv(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _CAN_H */

/*******************************************************************************
 End of File
 */
