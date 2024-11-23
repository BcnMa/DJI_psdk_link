/**
 ********************************************************************
 * @file    hal_uart.h
 * @brief   This is the header file for "hal_uart.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HAL_UART_H
#define HAL_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include "stdlib.h"
#include <stdbool.h>

//#include "dji_platform.h"

typedef void *T_DjiUartHandle;

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
//User can config dev based on there environmental conditions
#define LINUX_UART_DEV1    "/dev/ttyUSB0"
#define LINUX_UART_DEV2    "/dev/ttyUSB1"
#define LINUX_UART_DEV3    "/dev/ttyUSB2"


typedef enum {
    /**
    * All aircraft type support，users can connect via chip serial port or USB to TTL serial port.
    * Baud rate support list on M300 RTK Payload Port: 115200, 230400, 460800, 921600.
    * Baud rate support list on M300 RTK Extension Port: 115200, 230400, 460800, 921600, 1000000.
    * Baud rate support list on M30/M30T: 115200, 230400, 460800, 921600, 1000000.
    * Baud rate support list on M3E/M3T: 921600.
    * Baud rate support list on M350 RTK Payload Port: 115200, 230400, 460800, 921600.
    * Baud rate support list on M350 RTK Extension Port: 115200, 230400, 460800, 921600, 1000000.
    * */
    DJI_HAL_UART_NUM_0=0,
    /**
    * Only support on M300/M350 RTK Extension Port by USB virtual serial port, such as /dev/ttyACM0.
    * Baud rate support list on M300 RTK Extension Port: 921600.
    * Baud rate support list on M350 RTK Extension Port: 921600.
    * */
    DJI_HAL_UART_NUM_1,

    DJI_HAL_UART_NUM_2,
} E_DjiHalUartNum;

typedef struct {
    bool isConnect;
} T_DjiUartStatus;


/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
int HalUart_Init(E_DjiHalUartNum uartNum, uint32_t baudRate, T_DjiUartHandle *uartHandle);
int HalUart_DeInit(T_DjiUartHandle uartHandle);
int HalUart_WriteData(T_DjiUartHandle uartHandle, const uint8_t *buf, uint32_t len, uint32_t *realLen);
int HalUart_ReadData(T_DjiUartHandle uartHandle, uint8_t *buf, uint32_t len, uint32_t *realLen);
int HalUart_GetStatus(E_DjiHalUartNum uartNum, T_DjiUartStatus *status);

#ifdef __cplusplus
}
#endif

#endif // HAL_UART_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
