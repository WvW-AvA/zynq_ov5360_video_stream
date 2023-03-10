
/******************************************************************************
* Copyright (C) 2020 - 2021 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/*****************************************************************************/
/**
 * @file xipipsu_buf.h
* @addtogroup ipipsu Overview
* @{
* @details
 *
 * The xipipsu_buf.h is the header file for implementation of IPIPSU driver
 * get buffer functions.
 * Inter Processor Interrupt (IPI) is used for communication between
 * different processors.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver  Who Date     Changes
 * ---- --- -------- --------------------------------------------------
 * 2.6	sd	04/02/20	Restructured the code for more readability and modularity
 * </pre>
 *
 *****************************************************************************/
/** @cond INTERNAL */
#ifndef XIPIPSU_BUF_H_
#define XIPIPSU_BUF_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include "xil_io.h"
#include "xstatus.h"

/************************** Function Prototypes *****************************/

u32* XIpiPsu_GetBufferAddress(XIpiPsu *InstancePtr, u32 SrcCpuMask,
		u32 DestCpuMask, u32 BufferType);

#ifdef __cplusplus
}
#endif

#endif /* XIPIPSU_BUF_H_ */
/** @endcond */
/** @} */
