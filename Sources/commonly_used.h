/**
 * @file       commonly_used.h
 * @author     Tu (Bamboo.Tu@amitatech.com)
 * @brief      
 * @version    0.1
 * @date       2022-10-24
 * 
 * @copyright  Copyright (c) 2022 Amita Technologies Inc.
 * 
 * Abbreviation: 
 * None
 */
#ifndef _COMMONLY_USED_H /* Guard against multiple inclusion */
#define _COMMONLY_USED_H
// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

/* Global define -------------------------------------------------------------*/
/* USER CODE BEGIN GD */
#define MOD(A, M)              (A % M)
#define DIVIDE(A, M)           (A / M)
#define MAX(A, B)              ((A < B) ? B : A)
#define MIN(A, B)              ((B < A) ? B : A)
#define ABS(A)                 (((A) >= 0) ? (A) : -(A))          /* absolute value */
#define DELTA(A, B)            (((A) >= (B)) ? (A - B) : (B - A)) /* defferrence */
#define LOW_BYTE(A)            (*((unsigned char *)(&A)))
#define HIGH_BYTE(A)           (*((unsigned char *)(&A) + 1))
#define LOW_WORD(A)            (*((unsigned short *)(&A)))
#define HIGH_WORD(A)           (*((unsigned short *)(&A) + 1))
#define BIT_SET(VAR, BIT_NUM)  ((VAR) |= 1UL << (BIT_NUM))
#define BIT_CLR(VAR, BIT_NUM)  ((VAR) &= ~(1UL << (BIT_NUM)))
#define BIT_READ(VAR, BIT_NUM) ((VAR) &= (1UL << (BIT_NUM)))
#define SATURATION(VAR, MAX, MIN)   \
    ({                              \
        double __ret = 0;           \
        do {                        \
            if (MIN > MAX) {        \
                __ret = 0;          \
            } else if (VAR > MAX) { \
                __ret = MAX;        \
            } else if (VAR < MIN) { \
                __ret = MIN;        \
            } else {                \
                __ret = VAR;        \
            }                       \
        } while (0);                \
        __ret;                      \
    })
/* USER CODE END GD */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Global variables -----------------------------------------------------------*/
/* USER CODE BEGIN GV */

/* USER CODE END GV */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
/* USER CODE BEGIN FP */
double Lookup_Table(double input, volatile const double *bp0, volatile const double *table, unsigned int tableSize);
double Filter_Deadzone(double data, double threshold);
double Filter_ArrayAverage(double *arr, double arrSize);
/* USER CODE END FP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END
#endif /* _COMMONLY_USED_H */
/*******************************************************************************
 End of File
 */
