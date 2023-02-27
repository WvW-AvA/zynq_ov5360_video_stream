/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include "xparameters.h"
#include <stdio.h>
#include "xil_printf.h"
#include "sleep.h"
#include "xscugic.h"
#include "xuartps_intr_example.h"
#include "display_demo.h"
#define UART_DEVICE_ID XPAR_XUARTPS_0_DEVICE_ID
#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID XPAR_XUARTPS_1_INTR

/* Statement */
#define UART_TX 0
#define UART_RXCHECK 1
#define UART_WAIT 2

/* maximum receiver length */
#define MAX_LEN 2000
u8 TxString[14] =
    {
        "Hello ALINX!\r\n"};

XUartPsFormat UartFormat =
    {
        115200,
        XUARTPS_FORMAT_8_BITS,
        XUARTPS_FORMAT_NO_PARITY,
        XUARTPS_FORMAT_1_STOP_BIT};

/************************** Variable Definitions *****************************/

/* UART receiver buffer */
u8 ReceivedBuffer[MAX_LEN];
/* UART receiver buffer pointer*/
u8 *ReceivedBufferPtr;
/* UART receiver byte number */
volatile u32 ReceivedByteNum;

volatile u32 ReceivedFlag;
u8 state = UART_TX;

u32 SendByteNum;
u8 *SendBufferPtr;
/*
 * Function declaration
 */
int UartPsSend(XUartPs *InstancePtr, u8 *BufferPtr, u32 NumBytes);
int UartPsRev(XUartPs *InstancePtr, u8 *BufferPtr, u32 NumBytes);

int SetupInterruptSystem(XScuGic *IntcInstancePtr, XUartPs *UartInstancePtr, u16 UartIntrId);
void Handler(void *CallBackRef);

int uart_main(XUartPs *Uart_PS, XScuGic *IntcInstPtr)
{
    int Status;
    XUartPs_Config *Config;

    ReceivedBufferPtr = ReceivedBuffer;

    ReceivedFlag = 0;
    ReceivedByteNum = 0;

    Config = XUartPs_LookupConfig(UART_DEVICE_ID);
    if (NULL == Config)
    {
        return XST_FAILURE;
    }
    Status = XUartPs_CfgInitialize(Uart_PS, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }
    /* Use Normal mode. */
    XUartPs_SetOperMode(Uart_PS, XUARTPS_OPER_MODE_NORMAL);
    /* Set uart mode Baud Rate 115200, 8bits, no parity, 1 stop bit */
    XUartPs_SetDataFormat(Uart_PS, &UartFormat);
    /*Set receiver FIFO interrupt trigger level, here set to 1*/
    XUartPs_SetFifoThreshold(Uart_PS, 1);
    /* Enable the receive FIFO trigger level interrupt and empty interrupt for the device */
    XUartPs_SetInterruptMask(Uart_PS, XUARTPS_IXR_RXOVR | XUARTPS_IXR_RXEMPTY);

    SetupInterruptSystem(IntcInstPtr, Uart_PS, UART_INT_IRQ_ID);
}
int uart_loop(XUartPs *Uart_PS)
{
    switch (state)
    {
    case UART_TX: /* Send string : Hello Alinx! */
    {
        state = UART_RXCHECK;
        break;
    }
    case UART_RXCHECK: /* Check receiver flag, send received data */
    {
        if (ReceivedFlag)
        {
            /* Reset receiver pointer, flag, byte number */
            ReceivedBufferPtr = ReceivedBuffer;
            SendBufferPtr = ReceivedBuffer;
            SendByteNum = ReceivedByteNum;
            ReceivedFlag = 0;
            ReceivedByteNum = 0;
            UartPsSend(Uart_PS, SendBufferPtr, SendByteNum);
            float c_w = ReceivedBuffer[0] / 100.f;
            float c_h = ReceivedBuffer[1] / 100.f;
            float w = ReceivedBuffer[2] / 100.f;
            float h = ReceivedBuffer[3] / 100.f;
            set_window(c_w, c_w, w, h);
            xil_printf("set window done");
        }
        else
        {
            state = UART_WAIT;
        }
        break;
    }
    case UART_WAIT: /* Wait for 1s */
    {
        sleep(1);
        state = UART_TX;
        break;
    }
    default:
        break;
    }
}

int SetupInterruptSystem(XScuGic *IntcInstancePtr, XUartPs *UartInstancePtr, u16 UartIntrId)
{
    int Status;

    Status = XScuGic_Connect(IntcInstancePtr, UartIntrId,
                             (Xil_ExceptionHandler)Handler,
                             (void *)UartInstancePtr);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    XScuGic_Enable(IntcInstancePtr, UartIntrId);

    return Status;
}

void Handler(void *CallBackRef)
{
    XUartPs *UartInstancePtr = (XUartPs *)CallBackRef;
    u32 ReceivedCount = 0;
    u32 UartSrValue;

    UartSrValue = XUartPs_ReadReg(UartInstancePtr->Config.BaseAddress, XUARTPS_SR_OFFSET) & (XUARTPS_IXR_RXOVR | XUARTPS_IXR_RXEMPTY);
    ReceivedFlag = 0;

    if (UartSrValue & XUARTPS_IXR_RXOVR) /* check if receiver FIFO trigger */
    {
        ReceivedCount = UartPsRev(UartInstancePtr, ReceivedBufferPtr, MAX_LEN);
        ReceivedByteNum += ReceivedCount;
        ReceivedBufferPtr += ReceivedCount;
        /* clear trigger interrupt */
        XUartPs_WriteReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, XUARTPS_IXR_RXOVR);
    }
    else if (UartSrValue & XUARTPS_IXR_RXEMPTY) /*check if receiver FIFO empty */
    {
        /* clear empty interrupt */
        XUartPs_WriteReg(UartInstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET, XUARTPS_IXR_RXEMPTY);
        ReceivedFlag = 1;
    }
}

int UartPsSend(XUartPs *InstancePtr, u8 *BufferPtr, u32 NumBytes)
{

    u32 SentCount = 0U;

    /* Setup the buffer parameters */
    InstancePtr->SendBuffer.RequestedBytes = NumBytes;
    InstancePtr->SendBuffer.RemainingBytes = NumBytes;
    InstancePtr->SendBuffer.NextBytePtr = BufferPtr;

    while (InstancePtr->SendBuffer.RemainingBytes > SentCount)
    {
        /* Fill the FIFO from the buffer */
        if (!XUartPs_IsTransmitFull(InstancePtr->Config.BaseAddress))
        {
            XUartPs_WriteReg(InstancePtr->Config.BaseAddress,
                             XUARTPS_FIFO_OFFSET,
                             ((u32)InstancePtr->SendBuffer.NextBytePtr[SentCount]));

            /* Increment the send count. */
            SentCount++;
        }
    }

    /* Update the buffer to reflect the bytes that were sent from it */
    InstancePtr->SendBuffer.NextBytePtr += SentCount;
    InstancePtr->SendBuffer.RemainingBytes -= SentCount;

    return SentCount;
}

int UartPsRev(XUartPs *InstancePtr, u8 *BufferPtr, u32 NumBytes)
{
    u32 ReceivedCount = 0;
    u32 CsrRegister;

    /* Setup the buffer parameters */
    InstancePtr->ReceiveBuffer.RequestedBytes = NumBytes;
    InstancePtr->ReceiveBuffer.RemainingBytes = NumBytes;
    InstancePtr->ReceiveBuffer.NextBytePtr = BufferPtr;

    /*
     * Read the Channel Status Register to determine if there is any data in
     * the RX FIFO
     */
    CsrRegister = XUartPs_ReadReg(InstancePtr->Config.BaseAddress,
                                  XUARTPS_SR_OFFSET);

    /*
     * Loop until there is no more data in RX FIFO or the specified
     * number of bytes has been received
     */
    while ((ReceivedCount < InstancePtr->ReceiveBuffer.RemainingBytes) &&
           (((CsrRegister & XUARTPS_SR_RXEMPTY) == (u32)0)))
    {
        InstancePtr->ReceiveBuffer.NextBytePtr[ReceivedCount] =
            XUartPs_ReadReg(InstancePtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);

        ReceivedCount++;

        CsrRegister = XUartPs_ReadReg(InstancePtr->Config.BaseAddress,
                                      XUARTPS_SR_OFFSET);
    }
    InstancePtr->is_rxbs_error = 0;
    /*
     * Update the receive buffer to reflect the number of bytes just
     * received
     */
    if (InstancePtr->ReceiveBuffer.NextBytePtr != NULL)
    {
        InstancePtr->ReceiveBuffer.NextBytePtr += ReceivedCount;
    }
    InstancePtr->ReceiveBuffer.RemainingBytes -= ReceivedCount;

    return ReceivedCount;
}
