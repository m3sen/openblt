/************************************************************************************//**
* \file         xcptprs485.h
* \brief        XCP Rs495 transport layer header file.
* \ingroup      XcpTpRs485
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2017  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/
/************************************************************************************//**
* \defgroup   XcpTpUart XCP UART transport layer
* \brief      This module implements the XCP transport layer for UART.
* \ingroup    XcpLoader
****************************************************************************************/
#ifndef XCPTPRS485_H
#define XCPTPRS485_H

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Layout of structure with settings specific to the XCP transport layer module
 *         for UART.
 */
typedef struct t_xcp_tp_rs485_settings
{
  char const * portname;               /**< Interface port name, i.e. /dev/ttyUSB0.    */
  uint32_t baudrate;                   /**< Communication speed in bits/sec.           */
  uint32_t SlaveAddr;                  /**< Slave Address, scanf를 사용함으로 입력변수 처리를 위해 32bit로 선언함. */
  uint32_t FuncCode;                   /**< Set to 16    , scanf를 사용함으로 입력변수 처리를 위해 32bit로 선언함. */
  uint32_t RegAddr;                    /**< Set to 0x1f  , scanf를 사용함으로 입력변수 처리를 위해 32bit로 선언함. */
  uint32_t CRC16;                      /**< Set to CRC16 , scanf를 사용함으로 입력변수 처리를 위해 32bit로 선언함. */
} tXcpTpRs485Settings;


/***************************************************************************************
* Function prototypes
****************************************************************************************/
tXcpTransport const * XcpTpRs485GetTransport(void);

#ifdef __cplusplus
}
#endif

#endif /* XCPTPRS485_H */
/*********************************** end of xcptpuart.h ********************************/
