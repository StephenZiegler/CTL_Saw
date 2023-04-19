/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "device.h"
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "VescCAN.h"
#includ
#include "device.h"
#ie <stdio.h>
#include <ctype.h>  		/* required for the isalnum function */
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
int traverse = 1;
int saw = 2;
int Vesc_cmd_id;
uint32_t VescID;
uint8_t VescLength = 32;
volatile bool xTxCanData;
int32_t Ramp = 40;  //25

char VescRx_msg[100] = {0};

CAN_RX_MSG RxMsg;

uint8_t rxbuf[8];


int32_t   Test1;
float   lHomeMM = 30;  //offset from negative over travel limit up to 69mm for now
int32_t lHomeOffset = 0;  //degrees
float   lMoveMM = 150;  //;distance back and forth to cut
int32_t lMoveDistance = 0;  //;degrees
int32_t lMoveIncrement = 30;  //;degrees


int32_t iMoveQuotient = 0;
int32_t iMoveRemainder = 0;
int32_t Pos = 0;
int32_t Rpm = 0;
int AtSpeed = 0;
int AtPos   = 0;
int AtDuty  = 0;

int32_t Delay = 0;
int32_t BtnCount = 0;
bool BtnPressed = 0;
int32_t CutPos = 0;

int32_t i = 0;
bool direction = true;
static volatile bool pos_overtravel = false;
static volatile bool neg_overtravel = false;
#define SWITCH_ON_STATE                     1   // Active HI switch
static volatile bool isUART1TxComplete = false;
static uint8_t __attribute__ ((aligned(32))) uart1TxBuffer[100] = {0};
char Buf[30] = {0};
//FUNCTIONS
void VescCan(void);
bool Button1Trigger();

static void tmr1EventHandler (uint32_t intCause, uintptr_t context)
{
    LD4_Toggle();
    xTxCanData = 1;
}
static void POS_LIM_User_Handler(CN_PIN POS_LIM, uintptr_t context)
{          
       if(POS_LIM_Get() == SWITCH_ON_STATE){
            pos_overtravel = true;  
            LD3_Set();
        }
       else{
           pos_overtravel = false;
           LD3_Clear();
       }
}
static void NEG_LIM_User_Handler(CN_PIN NEG_LIM, uintptr_t context)
{          
       if(NEG_LIM_Get() == SWITCH_ON_STATE){
            neg_overtravel = true;     
            LD2_Set();
        }
       else{
           neg_overtravel = false;  
           LD2_Clear();
       }
}
static void UART1DmaChannelHandler(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    if (event == DMAC_TRANSFER_EVENT_COMPLETE)
    {
        isUART1TxComplete = true;
        // write to port J2 usb
    }
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
           
            
           
             while(!UART1_TransmitComplete());
                    //sprintf((char*)uart1TxBuffer, "Temp = %02dF    SEC = %d   DegLat = %d\r\n", temperatureVal,GPSMod.ucSeconds,GPSMod.latDegrees);
                sprintf((char*)uart1TxBuffer, "Quotient = %d Remainder = %d lMoveDistance = %d deg  home = %d MoveMM = %f \r\n",iMoveQuotient,iMoveRemainder,lMoveDistance,lHomeOffset,lMoveMM);
              
                DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
            while(!UART1_TransmitComplete());
            iVescCase = 10;
            
            break;
            
        case 10:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    //SetCurrentLimits(int axis_id, float AmpLimitPlus, float AmpLimitMinus)
                    SetCurrentLimits(traverse, 20,-20);
                    SetCurrentLimits(saw, 20,-20);
                    iVescCase = 50;           
                }
                
            }
            break;
        
        case 15:
        
            break;    
            
        case 20:
          
            break;
            
        case 21:
           
            break;
            
        case 30:
           
            break;
            
        case 31:
          
            
            
            break;    
            
        case 32:
          
            
        case 40:
            break;
            
        case 41:
          
            break;
            
        case 50:
            //Test if there's data available
            if(VescFlags.bit.ValidMsgs)
            {
               if (!CAN1_TxFIFOIsFull(0) )
              {
                //Xmit data at interval
                if(xTxCanData)
                {
                    //Clear interval bit
                    xTxCanData = 0;
                    SetRPM(traverse,-800);
                    SetRPM(saw,-800);
                }
                //Increment Duty Count 
               }
                Delay++;
            if(Delay >= 100000)  //100,000  looks like one second
            {
                Delay = 0;
                CutPos = 0;
                //Goto next state
                iVescCase = 55;
            
                AtDuty = 0;
                if (UART1_TransmitterIsReady())
                {
                   memset(VescRx_msg,NULL,100);
                   sprintf(VescRx_msg,"Set home speed \r\n");
                   UART1_Write(&VescRx_msg, sizeof(VescRx_msg)); 
                }
            }   
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
                    //SetDuty(traverse,-0.1);
                    SetRPM(traverse,-400);
                    //SetDuty(traverse,0.01);
                }
                //Increment Duty Count
                AtDuty++; 
            }
            
            //Test for how long to run at duty
            //if(AtDuty >= 60000
           // if(neg_overtravel == true)
            //{
                //Turn off duty
                //SetDuty(traverse,0.0);
                  //SetRPM(traverse,-400);
                //SET_CURRENT(traverse,1.0);
                //Goto next state
                iVescCase = iVescCase + 1*((neg_overtravel)||(NEG_LIM_Get())) ;
               // Delay = 0;
               // memset(VescRx_msg,NULL,100);
               // sprintf(VescRx_msg,"Done with Duty Cycle\r\n");
               // UART1_Write(&VescRx_msg, sizeof(VescRx_msg));
           // }
            break;
            
        case 56:
              if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetCurrentLimits(traverse, 60,-60);
                     SetRPM(traverse,400);
                    //SetRPM(unit_id,4000);
                    //SetDuty(unit_id,0.0);
                     //SetCurrentLimits(unit_id, 20,20);
                    //SetPos(unit_id,lHomeOffset);
                    iVescCase = 57;
                }
                
            }
            break;
            
            
        case 57:
            Delay++;
            if(Delay >= 200000)  //100,000  looks like one second
            {
                Delay = 0;
                CutPos = 0;
                if (!CAN1_TxFIFOIsFull(0) )
                {
                if(xTxCanData)
                    {
                    xTxCanData = 0;
                    SetRPM(traverse,0);
                    //SetDuty(traverse,0.0);
                     //SetCurrentLimits(unit_id, 20,20);
                    //SetPos(traverse,lHomeOffset);
                    iVescCase = 57;
                    }
                
                }  
                               
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
                    //SetRPM(unit_id,1);
                    SetPos(traverse,lHomeOffset);
                    iVescCase = 59;
                }
                
            }
            break;
        case 59:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    Pos = lHomeOffset + CutPos;
                    SetPos(traverse,Pos);
                    iVescCase = 60;
                }
            }
          
            break;
            
        case 60:
            
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetPos(unit_id,Pos);
                }
            }
            //When Button is pressed start moving the motor
            iVescCase = iVescCase + 2*(Button1Trigger());
         
            break;
            
        case 62:
             if (direction == true) {
                direction = false;
                } else {
                direction = true;
                 }
            
            iVescCase = 65;
         
            break;
            
        case 65:
             
           if (direction == false) {
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    CutPos = CutPos + lMoveIncrement;
                    Pos = Pos + lMoveIncrement;
                    Pos = Pos - 360*(Pos > 360);
                    SetPos(traverse,Pos);   
                }   
                if(CutPos >= iMoveQuotient * lMoveIncrement)
                     {
                        CutPos = 0;
                        iVescCase = 69;
                     }                                  
            }
           }
           if (direction == true) { 
                if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    CutPos = CutPos - lMoveIncrement;
                    Pos = Pos - lMoveIncrement;
                    Pos = Pos + 360*(Pos < 0);
                    SetPos(traverse,Pos);   
                }   
                if(CutPos <= -(iMoveQuotient * lMoveIncrement))
                     {
                        CutPos = 0;
                        iVescCase = 69;
                     }                                  
            }
           }

            break;
        
        case 69:
           if (direction == false) {
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    Pos = Pos + iMoveRemainder;
                    Pos = Pos - 360*(Pos > 0);
                    SetPos(traverse,Pos);
                    CutPos = 0;
                    iVescCase = 70;
                }
            }
           }
           if (direction == true) {
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    Pos = Pos - iMoveRemainder;
                    Pos = Pos + 360*(Pos < 0);
                    SetPos(traverse,Pos);
                    CutPos = 0;
                    iVescCase = 70;
                }
            }
           }
            break;
        
        case 70:
            if (!CAN1_TxFIFOIsFull(0) )
            {
                if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetPos(traverse,Pos);
                    AtPos++;
                    if(AtPos >= 1)
                    {
                        AtPos = 0;
                        iVescCase = 60;
                        LD1_Toggle();
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


void SET_CURRENT(int unit_id,float current)
{   
//    uint8_t D_data[4] = {0,0,0,0};
    uint8_t D_data[8] = {0,0,0,0,0,0,0,0};
    uint8_t Dlength = 8;  //4;
    uint8_t DfifoNum = 1;  //2;
    int32_t lCurrent;
    //uint8_t* duty_b = (uint8_t*) &Duty;
    lCurrent = (int32_t)(current * 1000.0);
    D_data[0] = *((uint8_t *)(&lCurrent) + 3);
    D_data[1] = *((uint8_t *)(&lCurrent) + 2);
    D_data[2] = *((uint8_t *)(&lCurrent) + 1);
    D_data[3] = *((uint8_t *)(&lCurrent) + 0);
    
   sendVescMessage(unit_id,CMD_ID_SET_CURRENT,Dlength,D_data,DfifoNum,false);
}


void SET_CURRENT_BRAKE(int unit_id,float brake_current)
{   
//    uint8_t D_data[4] = {0,0,0,0};
    uint8_t D_data[8] = {0,0,0,0,0,0,0,0};
    uint8_t Dlength = 8;  //4;
    uint8_t DfifoNum = 1;  //2;
    int32_t lBrakeCurrent;
    //uint8_t* duty_b = (uint8_t*) &Duty;
    lBrakeCurrent = (int32_t)(brake_current * 1000.0);
    D_data[0] = *((uint8_t *)(&lBrakeCurrent) + 3);
    D_data[1] = *((uint8_t *)(&lBrakeCurrent) + 2);
    D_data[2] = *((uint8_t *)(&lBrakeCurrent) + 1);
    D_data[3] = *((uint8_t *)(&lBrakeCurrent) + 0);
    
   sendVescMessage(unit_id,CMD_ID_SET_CURRENT_BRAKE,Dlength,D_data,DfifoNum,false);
}

void SetCurrentLimits(int axis_id, float AmpLimitPlus, float AmpLimitMinus) {
    
    uint8_t  fifoNum = 0;
    uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int32_t lAmplimit;
    lAmplimit = (int32_t)(AmpLimitPlus * 1000.0);
    msg_data[4] = *((uint8_t *)(&lAmplimit) + 3);
    msg_data[5] = *((uint8_t *)(&lAmplimit) + 2);
    msg_data[6] = *((uint8_t *)(&lAmplimit) + 1);
    msg_data[7] = *((uint8_t *)(&lAmplimit) + 0);
    lAmplimit = (int32_t)(AmpLimitMinus * 1000.0);
    msg_data[0] = *((uint8_t *)(&lAmplimit) + 3);
    msg_data[1] = *((uint8_t *)(&lAmplimit) + 2);
    msg_data[2] = *((uint8_t *)(&lAmplimit) + 1);
    msg_data[3] = *((uint8_t *)(&lAmplimit) + 0);
  
    
    sendVescMessage(axis_id, CMD_ID_CONF_CURRENT_LIMITS,8,msg_data,fifoNum,false);
  
}
/*******************************************************************************
 End of File
*/
