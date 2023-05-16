/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "device.h"
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "VescCANSaw.h"
#include "HMIComms.h"
#include <stdio.h>
#include "device.h"
#include <ctype.h>  		/* required for the isalnum function */
#include <string.h>
#include <conio.h>
#include <math.h>

//#include "VescCANFeedBelt.h"


#undef TESTING
#define TESTHMI

//#ifdef TESTHMI

static char __attribute__ ((aligned (16))) uartRxBuffer[RXBUFFERSIZE] = {0};
//static char __attribute__ ((aligned (16))) uartTxBuffer[TXBUFFERSIZE] = {0};

volatile bool TransferComplete = false;
volatile bool xTimerDone = false;
volatile int TimeCount = 0;
int count = 1;
int time = 1000;

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

static void UART2RxDmaChannelHandler(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    if (event == DMAC_TRANSFER_EVENT_COMPLETE)
    {
        HmiStatus.Ctrl.bits.NewRxData = true;
        TransferComplete = true;
        
    }
}

static void UART2TxDmaChannelHandler(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    if (event == DMAC_TRANSFER_EVENT_COMPLETE)
    {
        HmiStatus.Ctrl.bits.TxDataDone = true;
        IFS1bits.U2TXIF = false;
        LD4_Toggle();
    }
}

static void SetupUartRxDMA()
{
    DMAC_ChannelCallbackRegister(DMAC_CHANNEL_0, UART2RxDmaChannelHandler, 0); //Assign callback
    DMAC_ChannelPatternMatchSetup(DMAC_CHANNEL_0, '^'); //Setup DMA transfer to stop on linefeed
    DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)&U2RXREG, 1, (const void *)uartRxBuffer, strlen((const char*)uartRxBuffer), 1);
}

static void SetupUartTxDMA()
{
    DMAC_ChannelCallbackRegister(DMAC_CHANNEL_1, UART2TxDmaChannelHandler, 0); //Assign callback
}

static void TMR4Handler (uint32_t intCause, uintptr_t context)
{
    LD4_Toggle();
    
    if(TimeCount < 5)
    {
        TimeCount++;
    }
}
     
int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    SetupHmiMachine();
    SetupUartRxDMA();
    SetupUartTxDMA();
    
    TMR4_CallbackRegister( TMR4Handler, 0 );
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
       
      //  VescCanSaw();
        
          
        RunHmiStateMachine(&uartRxBuffer);
        
        
        
        
        
        
        
        
        
        
        if(HmiStatus.Ctrl.bits.RstRxDMA)
        {
            HmiStatus.Ctrl.bits.RstRxDMA = false;
            LD4_Toggle();
            memset(uartRxBuffer,0,RXBUFFERSIZE);
            SetupUartRxDMA();
        }
        
        if( !GPIO_RD6_Get() && (TimeCount == 0) )
        {
//            LD5_Set();
            TimeCount++;
            TMR4_Start();
        }
        
        if( GPIO_RD6_Get() && (TimeCount >= 5) )
        {
//            LD5_Clear();
            xTimerDone = false;
            xTimerDone = true;
            TMR4_Stop();
            AutoPage.TxData.CurCount.INT = count;
            AutoPage.TxData.CutTime.INT = time;
            AutoPage.TxData.AutoRunning.INT = 1;
            count++;
            if(count >= 20) count = 1;
            TimeCount = 0;
        }
        
//        if( xTimerDone && HmiStatus.Ctrl.bits.InitDone)
//        {
//            xTimerDone = false;
//            AutoPage.TxData.CurCount.INT = count;
//            AutoPage.TxData.CutTime.INT = time;
//            AutoPage.TxData.AutoRunning.INT = !AutoPage.TxData.AutoRunning.INT;
//            TimeCount = 0;
//            TMR4_Start();
//        }
     
   
 
        
        
        
        
      //  VescCanFeedBelt();
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}
