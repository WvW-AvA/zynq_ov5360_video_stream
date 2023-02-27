/************************************************************************/
/*																		*/
/*	display_demo.c	--	ALINX AX7010 HDMI Display demonstration 						*/
/*																		*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include "display_demo.h"
#include <stdio.h>
#include "math.h"
#include <ctype.h>
#include <stdlib.h>
#include "xil_types.h"
#include "xil_cache.h"
#include "xparameters.h"
#include "xiicps.h"
#include "vdma.h"
#include "i2c/PS_i2c.h"
#include "xgpio.h"
#include "sleep.h"
#include "ov5640.h"
#include "dp/dp.h"
#include "xgpiops.h"
#include "xuartps_intr_example.h"
// #include "ff.h"
//  #include "bmp.h"
#include "xtime_l.h"
#include "zynq_interrupt.h"
#include "xuartps.h"
/*
 * XPAR redefines
 */
#define CAM_VDMA_ID XPAR_AXIVDMA_0_DEVICE_ID
#define DISPLAY_NUM_FRAMES 3

#define UART_INT_IRQ_ID XPAR_XUARTPS_1_INTR
#define UART_DEVICE_ID XPAR_XUARTPS_0_DEVICE_ID
#define S2MM_INTID XPAR_FABRIC_AXI_VDMA_0_S2MM_INTROUT_INTR

#define KEY_INTR_ID XPAR_XGPIOPS_0_INTR
#define MIO_0_ID XPAR_PSU_GPIO_0_DEVICE_ID
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define PS_KEY_MIO 79
#define PS_LED_MIO 78

#define HIGHT 720
#define WIDTH 1280
#define STRIDE 1280 * 4
/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/*
 * Display Driver structs
 */

XAxiVdma camera_vdma;
XIicPs ps_i2c0;
XGpio cmos_rstn;
XUartPs MyUartPs;
XScuGic XScuGicInstance;

// static FIL fil; /* File object */
// static FATFS fatfs;

static int WriteError;

int wr_index = 0;
int rd_index = 0;

XGpioPs GpioInstance;
volatile int key_flag = 0;
int KeyFlagHold = 1;
/*
 * Framebuffers for video data
 */

u8 photobuf[DEMO_MAX_FRAME] __attribute__((aligned(256)));
;

u8 frameBuf[DISPLAY_NUM_FRAMES][DEMO_MAX_FRAME] __attribute__((aligned(256)));
u8 *pFrames[DISPLAY_NUM_FRAMES]; // array of pointers to the frame buffers

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

static void WriteCallBack(void *CallbackRef, u32 Mask);
static void WriteErrorCallBack(void *CallbackRef, u32 Mask);

int GpioSetup(XScuGic *InstancePtr, u16 DeviceId, u16 IntrID, XGpioPs *GpioInstancePtr);
void GpioHandler(void *CallbackRef);

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CLAM(a, b, c) MAX(MIN((a), (b)), (c))
u32 window_top_left_x = 0;
u32 window_top_left_y = 0;
u32 window_w = STRIDE;
u32 window_h = HIGHT;
void set_window(float c_x, float c_y, float w, float h)
{
    memset(pFrames[1], 0, STRIDE * HIGHT);
    c_x = CLAM(0.0f, 1.0f, c_x);
    c_y = CLAM(0.0f, 1.0f, c_y);
    w = CLAM(0.0f, 1.0f, w);
    h = CLAM(0.0f, 1.0f, h);
    float l = MAX(c_x - w / 2, 0);
    float r = MIN(c_x + w / 2, 1);
    float t = MAX(c_x - w / 2, 0);
    float b = MIN(c_x + w / 2, 1);
    window_top_left_x = (int)(l * WIDTH) * 4;
    window_top_left_y = (int)(t * HIGHT);
    window_h = h * HIGHT;
    window_w = w * WIDTH * 4;
}
void update_window(u8 index)
{
    u8 *buffer = pFrames[index];
    memset(buffer, 0, STRIDE * (window_top_left_y));
    buffer += STRIDE * (window_top_left_y);
    for (int i = 0; i < window_h; i++)
    {
        memset(buffer, 0, window_top_left_x);
        memset(buffer + window_w + window_top_left_x, 0, STRIDE - (window_w + window_top_left_x));
        buffer += STRIDE;
    }
    memset(buffer, 0, STRIDE * (HIGHT - window_h - window_top_left_y));
}
void copy_window()
{
    u32 offset = STRIDE * (window_top_left_y);
    for (int i = 0; i < window_h; i++)
    {
        for (int j = 0; j < window_w; j += 4)
            *(u32 *)(pFrames[1] + offset + window_top_left_x + j) = *(u32 *)(pFrames[0] + offset + window_top_left_x + j);
        offset += STRIDE;
    }
}

int main(void)
{

    int Status;

    int i;
    // FRESULT rc;
    XTime TimerStart, TimerEnd;
    float elapsed_time;
    unsigned int PhotoCount = 0;

    char PhotoName[9];
    char PhotoPath[] = {'1', ':', '/', '0', '0', '0', '0', '.', 'b', 'm', 'p'};

    Xil_DCacheDisable();
    Xil_ICacheDisable();

    /*
     * Initialize an array of pointers to the 3 frame buffers
     */
    for (i = 0; i < DISPLAY_NUM_FRAMES; i++)
    {
        pFrames[i] = frameBuf[i];
        memset(pFrames[i], 0, DEMO_MAX_FRAME);
    }

    /*
     * Initial GIC
     */
    InterruptInit(XPAR_SCUGIC_0_DEVICE_ID, &XScuGicInstance);

    i2c_init(&ps_i2c0, XPAR_XIICPS_0_DEVICE_ID, 40000);
    XGpio_Initialize(&cmos_rstn, XPAR_CMOS_RST_DEVICE_ID); // initialize GPIO IP
    XGpio_SetDataDirection(&cmos_rstn, 1, 0x0);            // set GPIO as output
    XGpio_DiscreteWrite(&cmos_rstn, 1, 0x3);
    usleep(200000);
    XGpio_DiscreteWrite(&cmos_rstn, 1, 0x0); // set GPIO output value to 0

    usleep(200000);
    XGpio_DiscreteWrite(&cmos_rstn, 1, 0x3);
    usleep(200000);

    uart_main(&MyUartPs, &XScuGicInstance);
    /*
     * Initialize Sensor
     */
    sensor_init(&ps_i2c0);
    /*
     * Setup PS KEY and PS LED
     */
    GpioSetup(&XScuGicInstance, MIO_0_ID, KEY_INTR_ID, &GpioInstance);

    /*
     * DP dma demo
     */
    xil_printf("DPDMA Generic Video Example Test \r\n");
    Status = DpdmaVideoExample(&RunCfg, pFrames[1], &XScuGicInstance);
    if (Status != XST_SUCCESS)
    {
        xil_printf("DPDMA Video Example Test Failed\r\n");
        return XST_FAILURE;
    }

    sleep(1);

    Xil_DCacheEnable();
    Xil_ICacheEnable();

    /* Initial Camera Vdma */
    vdma_write_init(CAM_VDMA_ID, &camera_vdma, 1280 * 4, 720, DEMO_STRIDE, (unsigned int)pFrames[0], DISPLAY_NUM_FRAMES);
    /* Set General Callback for Sensor Vdma */
    XAxiVdma_SetCallBack(&camera_vdma, XAXIVDMA_HANDLER_GENERAL, WriteCallBack, (void *)&camera_vdma, XAXIVDMA_WRITE);
    /* Set Error Callback for Sensor Vdma */
    XAxiVdma_SetCallBack(&camera_vdma, XAXIVDMA_HANDLER_ERROR, WriteErrorCallBack, (void *)&camera_vdma, XAXIVDMA_WRITE);
    /* Connect interrupt to GIC */
    InterruptConnect(&XScuGicInstance, S2MM_INTID, XAxiVdma_WriteIntrHandler, (void *)&camera_vdma);
    /* enable vdma interrupt */
    XAxiVdma_IntrEnable(&camera_vdma, XAXIVDMA_IXR_ALL_MASK, XAXIVDMA_WRITE);

    /* Set PS LED off */
    XGpioPs_WritePin(&GpioInstance, PS_LED_MIO, 0);
    set_window(0.5, 0.5, 0.5, 0.5);
    // rc = f_mount(&fatfs, "1:/", 0);
    // if (rc != FR_OK)
    // {
    //    return 0;
    //}

    while (1)
    {
        uart_loop(&MyUartPs);
        if (key_flag == 2)
        {
            KeyFlagHold = 0;
            /*
             * increment of photo name
             */
            PhotoCount++;
            sprintf(PhotoName, "%04u.bmp", PhotoCount);
            for (i = 0; i < 8; i++)
                PhotoPath[i + 3] = PhotoName[i];
            /* Set PS LED on */
            XGpioPs_WritePin(&GpioInstance, PS_LED_MIO, 1);
            printf("Successfully Take Photo, Photo Name is %s\r\n", PhotoName);
            printf("Write to SD Card...\r\n");
            /*
             * Set timer
             */
            XTime_SetTime(0);
            XTime_GetTime(&TimerStart);

            Xil_DCacheInvalidateRange((INTPTR)pFrames[(wr_index + 1) % 3], (INTPTR)DEMO_MAX_FRAME);
            /*
             * Copy frame data to photo buffer
             */
            memcpy(&photobuf, pFrames[(wr_index + 1) % 3], DEMO_MAX_FRAME);

            /*
             * Clear key flag
             */
            key_flag = 0;
            /*
             * Write to SD Card
             */
            // bmp_write(PhotoPath, (char *)&BMODE_1280x720, (char *)&photobuf, DEMO_STRIDE, &fil);
            /*
             * Print Elapsed time
             */
            XTime_GetTime(&TimerEnd);
            elapsed_time = ((float)(TimerEnd - TimerStart)) / ((float)COUNTS_PER_SECOND);
            printf("INFO:Elapsed time = %.2f sec\r\n", elapsed_time);
        }
        /* Set PS LED off */
        XGpioPs_WritePin(&GpioInstance, PS_LED_MIO, 0);
        KeyFlagHold = 1;
    }

    return 0;
}

/*****************************************************************************/
/*
 * Call back function for write channel
 *
 * This callback only clears the interrupts and updates the transfer status.
 *
 * @param	CallbackRef is the call back reference pointer
 * @param	Mask is the interrupt mask passed in from the driver
 *
 * @return	None
 *
 ******************************************************************************/
static void WriteCallBack(void *CallbackRef, u32 Mask)
{
    if (Mask & XAXIVDMA_IXR_FRMCNT_MASK)
    {

        if (key_flag == 1)
        {
            key_flag = 2;
            return;
        }
        else if (key_flag == 2)
        {
            return;
        }
        //  XDpDma_SetChannelState(&DpDma, GraphicsChan, XDPDMA_DISABLE);

        //  if (wr_index == 2)
        //  {
        //      FrameBuffer.Address = pFrames[1];

        //      update_window(2);
        //      wr_index = 0;
        //  }
        //  else if (wr_index == 1)
        //  {
        //      FrameBuffer.Address = pFrames[0];
        //      update_window(1);
        //      wr_index = 2;
        //  }
        //  else if (wr_index == 0)
        //  {
        //      FrameBuffer.Address = pFrames[2];
        //      update_window(0);
        //      wr_index = 1;
        //  }
        // xil_printf("%d", wr_index);
        copy_window();
        // XDpDma_SetChannelState(&DpDma, GraphicsChan, XDPDMA_ENABLE);
        /* Set park pointer */
        XAxiVdma_StartParking((XAxiVdma *)CallbackRef, 0, XAXIVDMA_WRITE);
    }
}

/*****************************************************************************/
/*
 * Call back function for write channel error interrupt
 *
 * @param	CallbackRef is the call back reference pointer
 * @param	Mask is the interrupt mask passed in from the driver
 *
 * @return	None
 *
 ******************************************************************************/
static void WriteErrorCallBack(void *CallbackRef, u32 Mask)
{

    if (Mask & XAXIVDMA_IXR_ERROR_MASK)
    {
        WriteError += 1;
    }
}

/*
 * Set up GPIO and enable GPIO interrupt
 */
int GpioSetup(XScuGic *InstancePtr, u16 DeviceId, u16 IntrID, XGpioPs *GpioInstancePtr)
{
    XGpioPs_Config *GpioCfg;
    int Status;

    GpioCfg = XGpioPs_LookupConfig(DeviceId);
    Status = XGpioPs_CfgInitialize(GpioInstancePtr, GpioCfg, GpioCfg->BaseAddr);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }
    /* set MIO as input */
    XGpioPs_SetDirectionPin(GpioInstancePtr, PS_KEY_MIO, GPIO_INPUT);
    /* set interrupt type */
    XGpioPs_SetIntrTypePin(GpioInstancePtr, PS_KEY_MIO, XGPIOPS_IRQ_TYPE_EDGE_RISING);

    /* set MIO  as output */
    XGpioPs_SetDirectionPin(&GpioInstance, PS_LED_MIO, GPIO_OUTPUT);
    /* enable MIO  output */
    XGpioPs_SetOutputEnablePin(&GpioInstance, PS_LED_MIO, GPIO_OUTPUT);
    /* set priority and trigger type */
    XScuGic_SetPriorityTriggerType(InstancePtr, IntrID,
                                   0xA0, 0x3);
    Status = XScuGic_Connect(InstancePtr, IntrID,
                             (Xil_ExceptionHandler)GpioHandler,
                             (void *)GpioInstancePtr);

    XScuGic_Enable(InstancePtr, IntrID);

    XGpioPs_IntrEnablePin(GpioInstancePtr, PS_KEY_MIO);

    return XST_SUCCESS;
}
/*
 * GPIO interrupt handler
 */
void GpioHandler(void *CallbackRef)
{
    XGpioPs *GpioInstancePtr = (XGpioPs *)CallbackRef;
    int IntVal;

    IntVal = XGpioPs_IntrGetStatusPin(GpioInstancePtr, PS_KEY_MIO);
    /* clear key interrupt */
    XGpioPs_IntrClearPin(GpioInstancePtr, PS_KEY_MIO);
    if (IntVal & KeyFlagHold)
        key_flag = 1;
}
