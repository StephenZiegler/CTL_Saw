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

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "VescCAN.h"
#include <stdio.h>
#include <ctype.h>  		/* required for the isalnum function */
#include <stdlib.h>
#include <string.h>
#include <conio.h>
#include <math.h>

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
#define VESC_CMD_ID     1
#define VESCSTATID4     (CMD_ID_STATUS_4 << 8) + VESC_CMD_ID
#define VESCRXMSGSIZE   8


//VARIABLES
uint16_t iVescCase = 0;
int unit_id = 1;
int Vesc_cmd_id;
uint32_t VescID;
uint8_t VescLength = 32;
volatile bool xTxCanData;
int32_t Ramp = 40;  //25

char VescRx_msg[100] = {0};

CAN_RX_MSG RxMsg;

uint8_t rxbuf[8];

int32_t lHomeOffset = 150;  //63  319;

int32_t Pos = 0;
int32_t Rpm = 0;
int AtSpeed = 0;
int AtPos   = 0;
int AtDuty  = 0;
int32_t Delay = 0;
int32_t BtnCount = 0;
bool BtnPressed = 0;
int32_t CutPos = 0;

//FUNCTIONS
void VescCan(void);
bool Button1Trigger();

static void tmr1EventHandler (uint32_t intCause, uintptr_t context)
{
    LD4_Toggle();
    xTxCanData = 1;
}

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    TMR2_CallbackRegister(tmr1EventHandler, 0);  
    TMR2_Start();
       

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
                
        VescCan();
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

bool Button1Trigger()
{
    int32_t limit = 30000;
    static int state = 0;
    
    state = state*(BTN1_Get());
    
    switch(state)
    {
        case 0:
            //Reset Count
            BtnCount = 0;
            BtnPressed = 0;
            state = 5;
            break;
            
        case 5:
            //Increment Count
            BtnCount= BtnCount + 1*(BTN1_Get() && (BtnCount < limit));
            state = state + 5*( (BtnCount >= limit) && BTN1_Get());
            break;
            
        case 10:
            BtnPressed = 1;
            state = 15;
            break;
            
        case 15:
            BtnPressed = 0;
            break;
            
        default:
            break;
    }
    
    return BtnPressed; 
}

void VescCan()
{
    //Variable
    char VescError_msg[60] = {0};
//    static int32_t Pos = 0;
//    static int32_t Rpm = 0;
//    static int AtSpeed = 0;
//    static int AtPos   = 0;
//    static int AtDuty  = 0;
//    static int32_t Delay = 0;
    
    switch(CAN1_ErrorGet())
    {
        case CAN_ERROR_NONE:
            break;
        case CAN_ERROR_TX_RX_WARNING_STATE:
            sprintf(VescError_msg,"VESC_TxRx Warning\r\n");          
            UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
        case CAN_ERROR_RX_WARNING_STATE:
            sprintf(VescError_msg,"VESC_Rx Warning\r\n");          
            UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
        case CAN_ERROR_TX_WARNING_STATE:
            sprintf(VescError_msg,"VESC_Tx Warning\r\n");          
            UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
        case CAN_ERROR_RX_BUS_PASSIVE_STATE:
            sprintf(VescError_msg,"VESC_Rx Bus Passive\r\n");          
            UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
        case CAN_ERROR_TX_BUS_PASSIVE_STATE:
            sprintf(VescError_msg,"VESC_Tx Bus Passive\r\n");          
            UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
        case CAN_ERROR_TX_BUS_OFF_STATE:
            sprintf(VescError_msg,"VESC_Tx Bus Off\r\n");          
            UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
        default:
            //sprintf(VescError_msg,"VESC_DFLT\r\n");          
            //UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
    }
    
    switch(iVescCase)
    {
        case 0:
//            CAN1_Initialize();
            iVescCase = 50;
            memset(VescRx_msg,NULL,100);
            sprintf(VescRx_msg,"Wait for Comms\r\n");
            UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
        //Make sure UART is done
        while(UART1_WriteIsBusy());
            break;
            
        case 10:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetDuty(unit_id,0.1);
                }
                
            }
            break;
        
        case 15:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetPos2(unit_id,Pos);
                    Pos = Pos + 100000;
                    if(Pos >= 360000000)
                    {
                        Pos = 0;
                    }
                }
                
            }
            break;    
            
        case 20:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetPos(unit_id,Pos);
                    Pos = Pos + 20;
                    if(Pos >= 360)
                    {
                        Pos = 0;
                        iVescCase = 21;
                    }
                }
                
            }
            break;
            
        case 21:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetPos(unit_id,Pos);
                    AtPos++;
                    if(AtPos >= 100)
                    {
                        AtPos = 0;
                        iVescCase = 20;
                    }
                }
                
            }
            break;
            
        case 30:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetRPM(unit_id,Rpm);
                    Rpm = Rpm + Ramp;
                }
                
            }
            if(Rpm >= 10000)
            {
                iVescCase = 31;
            }
            break;
            
        case 31:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetRPM(unit_id,Rpm);
                    AtSpeed++;
                }
                
            }
            if(AtSpeed >= 100)
            {
                iVescCase = 32;
                AtSpeed = 0;
            }
            break;    
            
        case 32:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetRPM(unit_id,Rpm);
                    Rpm = Rpm - Ramp;
                }
                
            }
            if(Rpm <= 1)
            {
                iVescCase = 30;
            }
            break; 
            
        case 40:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetPos(unit_id,20);
                    Delay++;
                    if(Delay >= 40)
                    {
                        Delay = 0;
                        iVescCase = 41;
                    }
                }
                
            }
            break;
            
        case 41:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetPos(unit_id,200);
                    Delay++;
                    if(Delay >= 40)
                    {
                        Delay = 0;
                        iVescCase = 40;
                    }
                }
                
            }
            break;
            
        case 50:
            //Test if there's data available
            if(VescFlags.bit.ValidMsgs)
            {
                //Goto next state
                iVescCase = 55;
                AtDuty = 0;
                memset(VescRx_msg,NULL,100);
                sprintf(VescRx_msg,"Set Duty Cycle\r\n");
                UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
            }
            break;
            
        case 55:
            //Check that CAN FIFO is clear
            if (!CAN1_TxFIFOIsFull(0) )
            {
                //Xmit data at interval
                if(xTxCanData)
                {
                    //Clear interval bit
                    xTxCanData = 0;
                    //Send Duty Cycle
                    SetDuty(unit_id,0.1);
                }
                //Increment Duty Count
                AtDuty++; 
            }
            
            //Test for how long to run at duty
            if(AtDuty >= 60000)
            {
                //Turn off duty
                SetDuty(unit_id,0.0);
                //Goto next state
                iVescCase = 57;
                Delay = 0;
                memset(VescRx_msg,NULL,100);
                sprintf(VescRx_msg,"Done with Duty Cycle\r\n");
                UART1_Write(&VescRx_msg, sizeof(VescRx_msg));
            }
            break;
            
        case 56:
            break;
            
        case 57:
            Delay++;
            if(Delay >= 100000)
            {
                Delay = 0;
                //Goto next state
                iVescCase = 58;
            }
            break;
            
        case 58:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetPos(unit_id,lHomeOffset);
                    iVescCase = 60;
                }
                
            }
            break;
            
        case 60:
            //When Button is pressed start moving the motor
            iVescCase = iVescCase + 5*(Button1Trigger());
            CutPos = 0;
            LD1_Toggle();
            break;
            
        case 65:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    Pos = lHomeOffset + CutPos;
                    //Check for rollover of 360 degree
                    Pos = Pos - 360*(Pos > 360);
                    
                    SetPos(unit_id,Pos);
                    //Increment and Check if Cut pos is > then 360
                    CutPos = CutPos + 60;
//                    iVescCase = 68;  //70;
                    if(CutPos >= 360)
                    {
                        CutPos = 0;
                        iVescCase = 70;
                    }
                }
            }
            
            break;
            
        case 68:    
            
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    Pos = lHomeOffset + CutPos;
                    //Check for rollover of 360 degree
                    Pos = Pos - 360*(Pos > 360);
                    
                    SetPos(unit_id,Pos);
                    //Increment and Check if Cut pos is > then 360
                    CutPos = CutPos + 10;
                    if(CutPos >= 360)
                    {
                        CutPos = 0;
                        iVescCase = 70;
                    }
                }
            }
            
        case 70:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    Pos = lHomeOffset + CutPos;
                    SetPos(unit_id,Pos);
                    AtPos++;
                    if(AtPos >= 1)
                    {
                        AtPos = 0;
                        iVescCase = 60;
                    }
                }
                
            }
            break;
            
        default:
        break;
    }
    
    //Test if there's data available
    if(CAN1_InterruptGet(1, CAN_FIFO_INTERRUPT_RXFULLIF_MASK))
    {
        //Read Data
        receiveVescMessage();
    }
}

void sendVescMessage(int unit_id,int Vesc_cmd_id, int length , uint8_t *data,uint8_t fifoNum,bool remote_transmission_request) 
{
    VescID =  (Vesc_cmd_id << 8) + unit_id;
    VescLength = length;
    uint8_t Mydata[8] = {0};
    uint16_t Mytimestamp = 0;
    uint8_t TxfifoNum = 0;
    CAN_MSG_TX_ATTRIBUTE TXmsgAttr0 =  CAN_MSG_TX_DATA_FRAME;
    CAN_MSG_TX_ATTRIBUTE TXmsgAttr1 =  CAN_MSG_TX_REMOTE_FRAME;
    CAN_MSG_RX_ATTRIBUTE MymsgAttr =  CAN_MSG_RX_REMOTE_FRAME;  //CAN_MSG_RX_DATA_FRAME;  //CAN_MSG_RX_REMOTE_FRAME;
    bool xRxValid = 0;
    if(!CAN1_TxFIFOIsFull(TxfifoNum)){
        if (!remote_transmission_request) 
        {
            CAN1_MessageTransmit(VescID, VescLength, data,TxfifoNum,TXmsgAttr0);
            return;
        }
    CAN1_MessageTransmit(VescID, VescLength, data,TxfifoNum,TXmsgAttr1);
    }
        xRxValid = CAN1_MessageReceive( (uint32_t*)&VescID, (uint8_t*)&VescLength, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, fifoNum, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr );
 
        if(xRxValid)
         {
            memcpy(data, &Mydata, sizeof(Mydata));
            return;
        }
        return;
}


void receiveVescMessage()
{
    uint32_t ID =  (CMD_ID_STATUS << 8) + 1;
    uint8_t Length = 8;
    uint8_t Mydata[8] = {0};
    uint16_t Mytimestamp = 0;
    CAN_MSG_RX_ATTRIBUTE MymsgAttr =  CAN_MSG_RX_DATA_FRAME;   //CAN_MSG_RX_DATA_FRAME;  //CAN_MSG_RX_REMOTE_FRAME;
    static bool xMsg1Valid = 0;
    static bool xMsg2Valid = 0;
    static bool xMsg3Valid = 0;
    static bool xMsg4Valid = 0;
    static bool xMsg5Valid = 0;
   
            
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, 1, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
    {
        
//        //Write data to UART port    
//        sprintf(VescRx_msg,"Data1: %02X / %02X / %02X / %02X / %02X / %02X / %02X / %02X / \r\n",Mydata[0],Mydata[1],Mydata[2],Mydata[3],Mydata[4],Mydata[5],Mydata[6],Mydata[7]);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
        xMsg1Valid = 1;
        
        VescData.msg.DutyCycle.str[0] = Mydata[7];
        VescData.msg.DutyCycle.str[1] = Mydata[6];
        VescData.msg.TotCurr.str[0]   = Mydata[5];
        VescData.msg.TotCurr.str[1]   = Mydata[4];
        VescData.msg.RPM.str[0]       = Mydata[3];
        VescData.msg.RPM.str[1]       = Mydata[2];
        VescData.msg.RPM.str[2]       = Mydata[1];
        VescData.msg.RPM.str[3]       = Mydata[0];
        
        VescData.DutyCycle = (float)VescData.msg.DutyCycle.i16/1000.0;
        VescData.TotCurr   = (float)VescData.msg.TotCurr.i16/10.0;
        VescData.RPM       = VescData.msg.RPM.i32;
        
//        memset(VescRx_msg,NULL,100);
//        sprintf(VescRx_msg,"Duty: %1.3f\r\nTotCurr: %2.2f\r\nRPM: %d\r\n",VescData.DutyCycle,VescData.TotCurr,VescData.RPM);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
                
    }
    else
    {
       xMsg1Valid = 0; 
    }
    
    ID =  (CMD_ID_STATUS_2 << 8) + 1;
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, 2, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
    {
        
//        //Write data to UART port    
//        sprintf(VescRx_msg,"Data2: %02X / %02X / %02X / %02X / %02X / %02X / %02X / %02X / \r\n",Mydata[0],Mydata[1],Mydata[2],Mydata[3],Mydata[4],Mydata[5],Mydata[6],Mydata[7]);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
        xMsg2Valid = 1;
        
        VescData.msg.AmpHrsCharged.str[0] = Mydata[7];
        VescData.msg.AmpHrsCharged.str[1] = Mydata[6];
        VescData.msg.AmpHrsCharged.str[2] = Mydata[5];
        VescData.msg.AmpHrsCharged.str[3] = Mydata[4];
        VescData.msg.AmpHrs.str[0]        = Mydata[3];
        VescData.msg.AmpHrs.str[1]        = Mydata[2];
        VescData.msg.AmpHrs.str[2]        = Mydata[1];
        VescData.msg.AmpHrs.str[3]        = Mydata[0];
        
        VescData.AmpHrsCharged = (float)VescData.msg.AmpHrsCharged.i32/1e4;
        VescData.AmpHrs        = (float)VescData.msg.AmpHrs.i32/1e4;
        
//        memset(VescRx_msg,NULL,100);
//        sprintf(VescRx_msg,"AmpChrg: %1.3f\r\nTotCurr: %2.2f\r\n",VescData.AmpHrsCharged,VescData.AmpHrs);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
    }
    else
    {
       xMsg2Valid = 0; 
    }
    
    ID =  (CMD_ID_STATUS_3 << 8) + 1;
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, 3, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
    {
        
//        //Write data to UART port     
//        sprintf(VescRx_msg,"Data3: %02X / %02X / %02X / %02X / %02X / %02X / %02X / %02X / \r\n",Mydata[0],Mydata[1],Mydata[2],Mydata[3],Mydata[4],Mydata[5],Mydata[6],Mydata[7]);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
        xMsg3Valid = 1;
        
        VescData.msg.WattHrsCharged.str[0] = Mydata[7];
        VescData.msg.WattHrsCharged.str[1] = Mydata[6];
        VescData.msg.WattHrsCharged.str[2] = Mydata[5];
        VescData.msg.WattHrsCharged.str[3] = Mydata[4];
        VescData.msg.WattHrs.str[0]        = Mydata[3];
        VescData.msg.WattHrs.str[1]        = Mydata[2];
        VescData.msg.WattHrs.str[2]        = Mydata[1];
        VescData.msg.WattHrs.str[3]        = Mydata[0];
        
        VescData.WattHrsCharged = (float)VescData.msg.WattHrsCharged.i32/1e4;
        VescData.WattHrs        = (float)VescData.msg.WattHrs.i32/1e4;
        
//        memset(VescRx_msg,NULL,100);
//        sprintf(VescRx_msg,"WattChrg: %1.3f\r\nWattHrs: %2.2f\r\n",VescData.WattHrsCharged,VescData.WattHrs);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
    }
    else
    {
       xMsg3Valid = 0; 
    }
   
    ID =  (CMD_ID_STATUS_4 << 8) + 1;
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, 4, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
    {
        
//        //Write data to UART port    
//        sprintf(VescRx_msg,"Data4: %02X / %02X / %02X / %02X / %02X / %02X / %02X / %02X / \r\n",Mydata[0],Mydata[1],Mydata[2],Mydata[3],Mydata[4],Mydata[5],Mydata[6],Mydata[7]);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
        xMsg4Valid = 1;
        
        VescData.msg.PidPos.str[0]       = Mydata[7];
        VescData.msg.PidPos.str[1]       = Mydata[6];
        VescData.msg.TotInputCurr.str[0] = Mydata[5];
        VescData.msg.TotInputCurr.str[1] = Mydata[4];
        VescData.msg.MotorTemp.str[0]    = Mydata[3];
        VescData.msg.MotorTemp.str[1]    = Mydata[2];
        VescData.msg.FetTemp.str[0]      = Mydata[1];
        VescData.msg.FetTemp.str[1]      = Mydata[0];
        
        VescData.PidPos        = (float)VescData.msg.PidPos.i16/50.0;
        VescData.TotInputCurr  = (float)VescData.msg.TotInputCurr.i16/10.0;
        VescData.MotorTemp     = (float)VescData.msg.MotorTemp.i16/10.0;
        VescData.FetTemp       = (float)VescData.msg.FetTemp.i16/10.0;      
         
//        memset(VescRx_msg,NULL,100);
//        sprintf(VescRx_msg,"PidPos: %2.3f\r\nTotInputCurr: %2.3f\r\nMotorTemp: %2.3f\r\nFetTemp: %2.3f\r\n",VescData.PidPos,VescData.TotInputCurr,VescData.MotorTemp,VescData.FetTemp);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
    }
    else
    {
       xMsg4Valid = 0; 
    }
    
    ID =  (CMD_ID_STATUS_5 << 8) + 1;
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, 5, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
    {
        
//        //Write data to UART port    
//        sprintf(VescRx_msg,"Data5: %02X / %02X / %02X / %02X / %02X / %02X / %02X / %02X / \r\n",Mydata[0],Mydata[1],Mydata[2],Mydata[3],Mydata[4],Mydata[5],Mydata[6],Mydata[7]);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
        xMsg5Valid = 1;
        
        VescData.msg.InputVolts.str[0] = Mydata[5];
        VescData.msg.InputVolts.str[1] = Mydata[4];
        VescData.msg.Tach.str[0]       = Mydata[3];
        VescData.msg.Tach.str[1]       = Mydata[2];
        VescData.msg.Tach.str[2]       = Mydata[1];
        VescData.msg.Tach.str[3]       = Mydata[0];
        
        VescData.InputVolts = (float)VescData.msg.InputVolts.i16/1e1;
        VescData.Tach       = VescData.msg.Tach.i32;      
              
//        memset(VescRx_msg,NULL,100);
//        sprintf(VescRx_msg,"InputVolts: %2.3f\r\nTach: %d\r\nValid: %d\r\n",VescData.InputVolts,VescData.Tach,VescFlags.bit.ValidMsgs);
//        UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
//        //Make sure UART is done
//        while(UART1_WriteIsBusy());
    }
    else
    {
       xMsg5Valid = 0; 
    }
    
    VescFlags.bit.ValidMsgs = xMsg1Valid & xMsg2Valid & xMsg3Valid & xMsg4Valid & xMsg5Valid;  
}


void SetDuty(int unit_id,float Duty) 
{
    
//    uint8_t D_data[4] = {0,0,0,0};
    uint8_t D_data[8] = {0,0,0,0,0,0,0,0};
    uint8_t Dlength = 8;  //4;
    uint8_t DfifoNum = 1;  //2;
    int32_t lDuty;
    //uint8_t* duty_b = (uint8_t*) &Duty;
    lDuty = (int32_t)(Duty * 100000.0);
    D_data[0] = *((uint8_t *)(&lDuty) + 3);
    D_data[1] = *((uint8_t *)(&lDuty) + 2);
    D_data[2] = *((uint8_t *)(&lDuty) + 1);
    D_data[3] = *((uint8_t *)(&lDuty) + 0);
    
  

   sendVescMessage(unit_id,CMD_ID_SET_DUTY,Dlength,D_data,DfifoNum,false);
}


void SetPos(int unit_id, int32_t Pos)
{
    uint8_t data[8] = {0,0,0,0,0,0,0,0};
    uint8_t length = 8;  //4;
    uint8_t fifoNum = 1;  //2;
    int32_t SendPos = Pos * 1000000;
    
    data[0] = *((uint8_t *)(&SendPos) + 3);
    data[1] = *((uint8_t *)(&SendPos) + 2);
    data[2] = *((uint8_t *)(&SendPos) + 1);
    data[3] = *((uint8_t *)(&SendPos) + 0);
    
    sendVescMessage(unit_id,CMD_ID_SET_POS,length,data,fifoNum,false);
}


void SetPos2(int unit_id, int32_t Pos)
{
    uint8_t data[8] = {0,0,0,0,0,0,0,0};
    uint8_t length = 8;  //4;
    uint8_t fifoNum = 1;  //2;
    
    data[0] = *((uint8_t *)(&Pos) + 3);
    data[1] = *((uint8_t *)(&Pos) + 2);
    data[2] = *((uint8_t *)(&Pos) + 1);
    data[3] = *((uint8_t *)(&Pos) + 0);
    
    sendVescMessage(unit_id,CMD_ID_SET_POS,length,data,fifoNum,false);
}


void SetRPM(int unit_id, int32_t RPM)
{
    uint8_t data[8] = {0,0,0,0,0,0,0,0};
    uint8_t length = 8;  //4;
    uint8_t fifoNum = 1;  //2;
    
    data[0] = *((uint8_t *)(&RPM) + 3);
    data[1] = *((uint8_t *)(&RPM) + 2);
    data[2] = *((uint8_t *)(&RPM) + 1);
    data[3] = *((uint8_t *)(&RPM) + 0);
    
    sendVescMessage(unit_id,CMD_ID_SET_RPM,length,data,fifoNum,false);
}


/*******************************************************************************
 End of File
*/

