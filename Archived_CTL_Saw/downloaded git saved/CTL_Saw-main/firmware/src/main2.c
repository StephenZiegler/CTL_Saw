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
//#include "VescCANSaw.h"
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

static char __attribute__ ((aligned (16))) uart2RxBuffer[RXBUFFERSIZE] = {0};
//static char __attribute__ ((aligned (16))) uart2TxBuffer[TXBUFFERSIZE] = {0};
static char __attribute__ ((aligned (16))) Test_msg[RXBUFFERSIZE] = {0};
//char Test_msg[100] = {0};
volatile bool isUART1TxComplete = false;
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

 void UART1TxDmaChannelHandler(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    if (event == DMAC_TRANSFER_EVENT_COMPLETE)
   {
        isUART1TxComplete = true;
   }
}

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
        //LD4_Toggle();
        U2TXREG = 0x0;
    }
}

static void SetupUart2RxDMA()
{
    DMAC_ChannelCallbackRegister(DMAC_CHANNEL_0, UART2RxDmaChannelHandler, 0); //Assign callback
    DMAC_ChannelPatternMatchSetup(DMAC_CHANNEL_0, '^'); //Setup DMA transfer to stop on linefeed
    DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)&U2RXREG, 1, (const void *)uart2RxBuffer, strlen((const char*)uart2RxBuffer), 1);
}

static void SetupUart2TxDMA()
{
    DMAC_ChannelCallbackRegister(DMAC_CHANNEL_1, UART2TxDmaChannelHandler, 0); //Assign callback
}
static void SetupUart1TxDMA()
{
    DMAC_ChannelCallbackRegister(DMAC_CHANNEL_2, UART1TxDmaChannelHandler, 0); //Assign callback
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
    
    SetupUart2RxDMA();
    SetupUart2TxDMA();
    SetupUart1TxDMA();
//    LD4_Set();
    TMR4_CallbackRegister( TMR4Handler, 0 );
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
       
      //  VescCanSaw();
        //TMR4_Start(); 
          
        RunHmiStateMachine(&uart2RxBuffer);
        
        
        
        
        
        
        
        
        
        
        if(HmiStatus.Ctrl.bits.RstRxDMA)
        {
            HmiStatus.Ctrl.bits.RstRxDMA = false;
         //   LD4_Toggle();
            memset(uart2RxBuffer,0,RXBUFFERSIZE);
            SetupUart2RxDMA();
        }
        
        if( !GPIO_RE9_Get() && (TimeCount == 0) )
        {
        //     LD4_Set();
       //     if (!UART1_WriteIsBusy()){ 
        //    sprintf((char*)Test_msg,"Stroke = %f count = %d \r\n",AutoPage.RxData.Stroke.FLT,count);
        //    DMAC_ChannelTransfer(DMAC_CHANNEL_2, (const void *)Test_msg, strlen((const char*)Test_msg), (const void *)&U1TXREG, 1, 1);
        //} 
          if (!UART1_WriteIsBusy()){ 
                sprintf(Test_msg,"Stroke = %d\r\n",(int)AutoPage.RxData.Stroke.FLT); 
                //UART1_Write(&Test_msg, sizeof(Test_msg));
                DMAC_ChannelTransfer(DMAC_CHANNEL_2, (const void *)Test_msg, strlen((const char*)Test_msg), (const void *)&U1TXREG, 1, 1); 
             // DMAC_ChannelTransfer(DMAC_CHANNEL_2, (const void *)Test_msg, strlen((const char*)Test_msg), (const void *)&U1TXREG, 1, 1);
                U1TXREG = 0x0;
             } 
          
   //         memset(TxBuffer,0,TXBUFFERSIZE);
  //          sprintf(TxBuffer,"$~%s~%d~%d~%d~%d~%d~%d~@",  TxHmiCmds[0],
   //                                                 data->TxData.AutoRunning.INT,
   //                                                 (int)(data->RxData.Stroke.FLT*1000),
   //                                                 (int)(data->RxData.HomeOffset.FLT*1000),
   //                                                 (int)(data->RxData.CutSpeedRPM.FLT*1000),
  //                                                  data->RxData.SetCount.INT,
   //                                                 data->TxData.CurCount.INT);
          
          
          
          
          
            TimeCount++;
            TMR4_Start();
        }
        
        if( GPIO_RE9_Get() && (TimeCount >= 5) )
//      if( GPIO_RE9_Get())
        {
         //   LD4_Clear();
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
