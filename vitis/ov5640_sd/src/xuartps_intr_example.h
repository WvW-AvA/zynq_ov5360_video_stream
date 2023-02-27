#ifndef __XUART_INTR_EXAMPLE_H__
#define __XUART_INTR_EXAMPLE_H__
#include "xuartps.h"
#include "xscugic.h"
int uart_main(XUartPs *Uart_PS, XScuGic *IntcInstPtr);

int uart_loop(XUartPs *Uart_PS);
#endif