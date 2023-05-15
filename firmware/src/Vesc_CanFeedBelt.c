/* ************************************************************************** */
/** Descriptive File Name

  @Company
 Zigtronics Inc.

  @File Name
    Vesc_Can.c

  @Summary
 This file sends commands over a can bus to BLDC motors to home them and run cycles of feed and cut
 * along with control of all the inputs and outputs

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
#include "VescCANFeedBelt.h"
#include <stdio.h>
#include "device.h"
#include <ctype.h>  		/* required for the isalnum function */
#include <string.h>
#include <conio.h>
#include <math.h>
#include "Trajectory.h"
//#include "VescCAN.h"
// *****************************************************************************
// *****************************************************************************

#define STEADYVEL       50000 //20000 //50  //5000  //360000000
#define ACCEL           1000  //2   //2000  //250000000
#define DECEL           1000  //2 //2000  //250000000
#define HOMEDUTYCYCLE   0.04
#define ATDUTYLIMIT     95000

//VARIABLES
int32_t ENDTARGETPOS = 0;
uint16_t iVescCase = 0;
int unit_id = 1;
int Vesc_cmd_id;
uint32_t VescID;
uint8_t VescLength = 32;
volatile bool xTxCanData;
//int32_t Ramp = 40;  //25
char VescRx_msg[100] = {0};
CAN_RX_MSG RxMsg;
uint8_t rxbuf[8];
tTRAJECTORY uTrajectory;
uint32_t nPos; 
uint8_t Posnow = 0;
float   lDiameter= 1.5; //inches
float   lCircumference ; //inches
float   Pi = 3.1416;

//  HMI VARIABLES ***************************************
float   lHomeMM = 1;  // received   //offset from negative over travel limit up to 69mm for now 70mm per rev
float   lHomeinch = 1.0;  //-1;  // received   //offset from negative over travel limit up to 69mm for now 70mm per rev
float   lMoveMM = 200; //  received  distance back and forth to cut
float   lMoveinch = 4.7124;  // -1.0;  //-4.7124; //  received  distance back and forth to cut 4.7124 inches per rev of motor
//int32_t iCutSpeed = 1500;  // received  speed is in RPM
float TotalTime = 0;   //sent
int32_t PartCount = 0 ; //sent
int32_t SetPartCount = 1000 ; // received
int32_t PartsLeft ;    //sent
int32_t PartsTotal ;  //sent = 
int32_t i_CurrentMax = 60;
//  ****************************************************


int32_t feedroller = 1;
int32_t   Test1;
int32_t lHomeOffset = 0;  //degrees
int32_t lMoveDistance = 0;  //;degrees
int32_t lMoveIncrement = 20;  //;degrees
 uint32_t nPos;
 uint32_t quotiant = 0;
 uint32_t sendPos = 0;
int32_t iMoveQuotient = 0;
int32_t iMoveRemainder = 0;
int32_t iMoveHomeQuotient = 0;
int32_t iMoveHomeRemainder = 0;
int32_t Pos = 0;
float fStartPos = 0;  
int32_t iCut_eRpm = 0;  //speed in eRpm

// DELAYS
//******************************************
int32_t EstopDelay = 0 ;
int AtPos   = 0;
int AtDuty  = 0;
int32_t Delay = 0;
int32_t Delay2 = 0;
int32_t Delay3 = 0;
int32_t Delay4 = 0;
int32_t CutPos = 0;

int32_t RestartDelay = 0;
int32_t PowerupDelay = 0;





bool b_directionBelt = false;
bool b_first = true;
bool b_first1 = true;
bool b_HomeStatusBelts = false;
bool Stop = false;
bool b_StartBelts = false;
bool b_ControlledStopBelt = false;

static volatile bool Estop = false;
//static volatile bool pos_overtravel = false;
//static volatile bool neg_overtravel = false;
#define SWITCH_ON_STATE                     1   // Active HI switch
#define SWITCH_OFF_STATE                    0   // Active HI switch
static volatile bool isUART1TxComplete = false;
static uint8_t __attribute__ ((aligned(32))) uart1TxBuffer[100] = {0};
//char Buf[30] = {0};


//FUNCTIONS
static void tmr4EventHandler (uint32_t intCause, uintptr_t context)
{
 //   LD4_Toggle();
    xTxCanData = 1;
}






void VescCanFeedBelt()
{
    //Variable
    char VescError_msg[60] = {0};
    //switch(CAN1_ErrorGet())
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
            sprintf(VescError_msg,"VESC_Tx Bus Off\r\n error = %d\r\n",CAN1_ErrorGet());          
            UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
        default:
            //sprintf(VescError_msg,"VESC_DFLT\r\n");          
            //UART1_Write(&VescError_msg, sizeof(VescError_msg)); 
            break;
    }
   //switch(iVescCase)   
    switch(iVescCase)
    {
        case 0:
           
            //Case to Initialize all settings
            CAN1_Initialize();
            
            TMR4_CallbackRegister(tmr4EventHandler, 0);  
            TMR4_Start();
            // Initialize Trajectory:
            TrajectoryInit(&uTrajectory,
                STEADYVEL,              // Steady velocity, in counts per second
                ACCEL,                  // Acceleration, in 1/65536 counts per ms^2
                DECEL                   // Deceleration, in 1/65536 counts per ms^2
            );
           // lMoveDistance = (lMoveMM/70)*360;   //1.5 inches diameter 4.7124 inches per rev
            
         
            lMoveinch = 4.7124; // Length #4
            
            VescFlags.bit.ValidMsgs = 0;
            lCircumference = Pi*lDiameter;
            lMoveDistance =  (lMoveinch/lCircumference)*360;  //4.7124
            lHomeOffset = (lHomeinch/4.7124)*360; 
            PartCount = 0;
            PartsLeft = 0;
            sendPos = 0;
            Delay = 0; 
            b_directionBelt = false;
            //O_KnifeSol_Clear();
            b_HomeStatusBelts = false;
            Delay = 0;  
            while(!UART1_TransmitComplete());
                sprintf((char*)uart1TxBuffer, "\r\nlMoveDistance = %d lMoveinch = %f \r\n",lMoveDistance,lMoveinch);
                DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
            while(!UART1_TransmitComplete());
                 //iVescCase = iVescCase +10*(I_Estop_Get() == 1);Estop = false
            iVescCase = iVescCase +10*(!(Estop));
               
            break;
            
        case 10:
            PowerupDelay++;
            RestartDelay++;
            if ((PowerupDelay >= 1000)&&(RestartDelay >= 1000)) {
            //PowerupDelay = 0;
                RestartDelay = 0;
                if(VescFlags.bit.ValidMsgs == 1) {
                    fStartPos = VescData.PidPos;
                    Pos = fStartPos;
                    if (!CAN1_TxFIFOIsFull(0) )
                    {
                        if(xTxCanData)
                        {
                            xTxCanData = 0;
                            SetCurrentLimits(feedroller, 60,-60);
                            Delay = 0;  
                            Delay2 = 0;
                            while(!UART1_TransmitComplete());
                            sprintf((char*)uart1TxBuffer, "iVescCase = %d VescFlags.bit.ValidMsgs = %d  Pos = %d PIDPos#1 = %f fStartPos = %f \r\n",iVescCase,VescFlags.bit.ValidMsgs,Pos,VescData.PidPos,fStartPos);
                            DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                            while(!UART1_TransmitComplete());
                            iVescCase = 12; 
                        }  
                    }
              }
            }
            break;
  
        case 12:
          //if(VescFlags.bit.ValidMsgs == 1){
           Delay2++;
             if(Delay2 >= 1000) {
               Delay2 = 0;
               if (!CAN1_TxFIFOIsFull(0) )
              {
                //Xmit data at interval
                if(xTxCanData)
                {
                    //Clear interval bit
                    xTxCanData = 0;
                    SetDuty(feedroller,0.05);  // rotates the taverse int he positive direction in duty cycle to find the encoder positon mark to find commutator angles
                }
               }
             }
                Delay++;
                
                //rotate for a time but if it hits the positive overtravel stop
           if ((abs(fStartPos - VescData.PidPos) >= 180)&& (VescData.RPM >= 1000)) {    
            if(Delay >= 50000)  //100,000  looks like one second
                {
                    Delay = 0;
                    AtDuty = 0;
                    while(!UART1_TransmitComplete());
                        sprintf((char*)uart1TxBuffer, "iVescCase = %d VescData.RPM = %d Pos = %d fStartPos = %f PIDPos#2 = %f \r\n",iVescCase,VescData.RPM,Pos,fStartPos,VescData.PidPos);
                        DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                     while(!UART1_TransmitComplete());
                      iVescCase = 20;
                }   
           }
            break;
            
   
        case 20:
           Delay2++;
             if(Delay2 >= 1000) {
               Delay2 = 0;
               if (!CAN1_TxFIFOIsFull(0) )
              {
                //Xmit data at interval
                if(xTxCanData)
                {
                    //Clear interval bit
                    xTxCanData = 0;
                    SetDuty(feedroller,0.01);  // rotates the taverse int he positive direction in duty cycle to find the encoder positon mark to find commutator angles
                }
               }
             }
                Delay++;
                
                //rotate for a time but if it hits the positive overtravel stop
            if(Delay >= 10000)  //100,000  looks like one second
                {
                    Delay = 0;
                    while(!UART1_TransmitComplete());
                        sprintf((char*)uart1TxBuffer, "iVescCase = %d Pos1 = %d PIDPos#3 = %f \r\n",iVescCase,Pos,VescData.PidPos);
                        DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                     while(!UART1_TransmitComplete());
                      iVescCase = 22;
                }   
          
            break;
            
           case 22:
               
              Delay++;
                //rotate for a time but if it hits the positive overtravel stop
            if(Delay >= 1000)  //100,000  looks like one second
                {
                 Delay = 0;   
                if (!CAN1_TxFIFOIsFull(0) ){
                    if(xTxCanData)
                     {
                    //Clear interval bit
                      xTxCanData = 0;
                      SetDuty(feedroller,0);
                      iVescCase = 23;  
                 }
               }
            }     
                //Goto next state
              //iVescCase = iVescCase + 1*((neg_overtravel)||(I_Neg_Lim_Get())) ;
            
            break;
            
             case 23:
             Delay2++;
             if(Delay2 >= 50000) {
               Delay2 = 0;
                Pos = VescData.PidPos;
                 if (!CAN1_TxFIFOIsFull(0) ){
                    if(xTxCanData)
                     {
                    //Clear interval bit
                    xTxCanData = 0;
                    SetCurrentLimits(feedroller, 60,-60);
                     
                    while(!UART1_TransmitComplete());
                        sprintf((char*)uart1TxBuffer, "iVescCase = %d Pos1 = %d  PIDPos#4 = %f \r\n",iVescCase,Pos,VescData.PidPos);
                        DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                    while(!UART1_TransmitComplete());
                    iVescCase = 24; 
                     }
                }
             }
                //Goto next state
              //iVescCase = iVescCase + 1*((neg_overtravel)||(I_Neg_Lim_Get())) ;
            
            break;
            
            case 24:
                
            Pos = VescData.PidPos;
            if (!CAN1_TxFIFOIsFull(0) ){
                if(xTxCanData)
                {
                    //Clear interval bit
                    xTxCanData = 0;
                       SetPos(feedroller,Pos); //hold in position
                       while(!UART1_TransmitComplete());
                        sprintf((char*)uart1TxBuffer, "iVescCase = %d lHomeOffset = %d Pos1 = %d  PIDPos#5 = %f \r\n",iVescCase,lHomeOffset,Pos,VescData.PidPos);
                        DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                       while(!UART1_TransmitComplete());
                      iVescCase = 44;  
                     }
                }
            
            break;
            
        case 44:
            AtPos++;
            if (AtPos >= 10000) {   
              AtPos = 0;  
              TrajectoryInit(&uTrajectory,
                STEADYVEL,              // Steady velocity, in counts per second
                ACCEL,                  // Acceleration, in 1/65536 counts per ms^2
                DECEL                   // Deceleration, in 1/65536 counts per ms^2
                    );
              
            lHomeOffset = (360 + lHomeOffset) - Pos ;  
            if (lHomeOffset >= 0)
                {
                TrajectorySeek(&uTrajectory, lHomeOffset, FORWARD);
                }
            else{
                 TrajectorySeek(&uTrajectory, lHomeOffset, REVERSE);
                 }
              
            while(!UART1_TransmitComplete());
                sprintf((char*)uart1TxBuffer, "iVescCase = %d lHomeOffset = %d Pos = %d  PIDPos#6 = %f \r\n",iVescCase,lHomeOffset,Pos,VescData.PidPos);
                DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
            while(!UART1_TransmitComplete());  
            iVescCase = 45;     
            }
            break;
            
        case 45: 
      if (lHomeOffset >= 0)
                { 
                AtPos++;
                if (AtPos >= 1000) {   
                    AtPos = 0;  
                    if(xTxCanData)
                    {
                        xTxCanData = 0;
                         if (!CAN1_TxFIFOIsFull(0) )
                        {
                        //Obtain the trajectory real-time position:
                        nPos = TrajectoryUpdate(&uTrajectory);
                        
                        sendPos = nPos + Pos ;
                        quotiant = abs(sendPos)/360;
                        sendPos = sendPos - (360*quotiant);  //if reverse direction
                        
                        //sendPos = nPos ; 
                       // quotiant = abs(sendPos)/360;  
                       // sendPos = (sendPos + Pos) - (360*quotiant);  //if pos direction
                        SetPos(feedroller,sendPos); // send out to can bus a SetPos command
                      
                        
                        }
                    }
                }  
                 }
           else{
                    
                AtPos++;
                if (AtPos >= 1000) {   
                    AtPos = 0;     
                if(xTxCanData)
                  {
                    xTxCanData = 0;
                    if (!CAN1_TxFIFOIsFull(0) )
                    {
                        // Obtain the trajectory real-time position:
                        nPos = TrajectoryUpdate(&uTrajectory);
                        
                        sendPos = 360 + nPos + Pos ;
                        quotiant = abs(sendPos)/360;
                        sendPos = sendPos - (360*quotiant);  //if reverse direction
                        
                        //sendPos = 360 + nPos ;     /// + (360*quotiant);  //if reverse direction
                        //quotiant = abs(sendPos)/360;
                        //sendPos = sendPos - (360*quotiant);  //if reverse direction
                        
                        
                        SetPos(feedroller,sendPos);
                    }
                  } 
                }   
               }  
            iVescCase = iVescCase + 1*(TrajectoryStatus(&uTrajectory) == TRAJECTORY_DONE);
            break; 
          
       
        case 46:
            while(!UART1_TransmitComplete());
                sprintf((char*)uart1TxBuffer, "iVescCase = %d Pos1 = %d sendPos = %d PIDPos#7 = %f \r\n",iVescCase,Pos,sendPos,VescData.PidPos);
                DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
            while(!UART1_TransmitComplete());
            Pos = sendPos;
            //Pos = VescData.PidPos;
            b_HomeStatusBelts = true;
            if (!CAN1_TxFIFOIsFull(0) )
            {
              if(xTxCanData)
                {
                    xTxCanData = 0;
                    SetCurrentLimits(feedroller, 60,-60);  // set the new current limit of the drive
                   
                    while(!UART1_TransmitComplete());
                        sprintf((char*)uart1TxBuffer, "iVescCase = %d Pos = %d sendPos = %d PIDPos#8 = %f \r\n",iVescCase,Pos,sendPos,VescData.PidPos);
                        DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                    while(!UART1_TransmitComplete());
                    iVescCase = 59;
                }              
            }
            break;
           
        case 59: 
            Delay = 0;
            AtPos++;
            b_ControlledStopBelt = false;
            if(PartCount >= SetPartCount){
               PartCount = 0; 
            }
             if (AtPos >= 10000) {   
              AtPos = 0;
              if(xTxCanData)
                 {
                  xTxCanData = 0;
                  if (!CAN1_TxFIFOIsFull(0) )
                     {
                      SetPos(feedroller,Pos);
                     }
                }
            }
            iVescCase = iVescCase + 1*((Button1Trigger()) || (b_StartBelts));
  //          iVescCase = 60;
        break;
            
        case 60:
            Delay++;
            AtPos++;
            if(PartCount >= SetPartCount){
               iVescCase = 59; 
            }
            if (AtPos >= 10000) {   
              AtPos = 0;
              if(xTxCanData)
                 {
                  xTxCanData = 0;
                  if (!CAN1_TxFIFOIsFull(0) )
                     {
                      SetPos(feedroller,Pos);
                     }
                }
            }
            //When Button is pressed start moving the motor or a delay then start
             iVescCase = iVescCase + 2*(((Button1Trigger())||(b_StartBelts)||(Delay >= 100000)));
        //     iVescCase = iVescCase + 2*(((Button1Trigger())||(b_StartBelts)));
        break;
            
        case 62:
            //O_KnifeSol_Clear();
            PartCount++;
            PartsTotal++;
            Delay = 0;
           while(!UART1_TransmitComplete());
                sprintf((char*)uart1TxBuffer, "iVescCase = %d Pos = %d  sendPos = %d PIDPos#9 = %f \r\n",iVescCase,Pos,sendPos,VescData.PidPos);
                DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
           while(!UART1_TransmitComplete());
             iVescCase = 64;
             
            break;
        case 64:
            Delay++;
            if (Delay >= 10000) {
                if(xTxCanData)
                {
                    Delay = 0;
                    xTxCanData = 0; 
                    if (!CAN1_TxFIFOIsFull(0) )
                       {
                       SetPos(feedroller,Pos); //hold in position
                      }
                     iVescCase = 66;
                    }
                }
            break;
                    
        case 66:
            LD4_Toggle();
            TrajectoryInit(&uTrajectory,
                STEADYVEL,              // Steady velocity, in counts per second
                ACCEL,                  // Acceleration, in 1/65536 counts per ms^2
                DECEL                   // Deceleration, in 1/65536 counts per ms^2
                    );
            
                TrajectorySeek(&uTrajectory, lMoveDistance, FORWARD);
                //TrajectorySeek(&uTrajectory, lMoveDistance, REVERSE);
                iVescCase = iVescCase + 2*(TrajectoryStatus(&uTrajectory) != TRAJECTORY_DONE);
          
        break;
        
        case 68: 
            Delay3++;
            if (Delay3 >= 1000) {
               Delay3 = 0;
               if(xTxCanData)
                    {
                    xTxCanData = 0;
                    //Delay3++;
                   // if (Delay3 >= 1) {
                     //   Delay3 = 0;
                        // Obtain the trajectory real-time position:
                        nPos = TrajectoryUpdate(&uTrajectory);
                        //sendPos = 360 + (nPos + Pos);     /// + (360*quotiant);  //if reverse direction
                        
                        //sendPos = nPos ;
                        //quotiant = abs(sendPos)/360;
                        //sendPos = (sendPos + Pos) - (360*quotiant);  //if reverse direction
                        
                        sendPos = nPos + Pos ;
                        quotiant = abs(sendPos)/360;
                        sendPos = sendPos - (360*quotiant);  //if reverse direction
                        if (!CAN1_TxFIFOIsFull(0) )
                            {
                            SetPos(feedroller,sendPos);
                            }
                      }
                    }
                    iVescCase = iVescCase + 2*(TrajectoryStatus(&uTrajectory) == TRAJECTORY_DONE);
               
        break;
             
        case 70:
            Pos =sendPos;
             AtPos = 0;
            while(!UART1_TransmitComplete());
            sprintf((char*)uart1TxBuffer, "iVescCase = %d Pos = %d  sendPos = %d PIDPos#10 = %f \r\n",iVescCase,Pos,sendPos,VescData.PidPos);
            DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
           while(!UART1_TransmitComplete());
            iVescCase = 80;
        break;
    
        case 80:
            //O_KnifeSol_Set();
            if(xTxCanData)
                {
                xTxCanData = 0;
                if (!CAN1_TxFIFOIsFull(0) )
                    {
                    SetPos(feedroller,Pos);
                    }
                AtPos++;
                if(AtPos >= 100)
                    {
                    AtPos = 0;
                   while(!UART1_TransmitComplete());
                    sprintf((char*)uart1TxBuffer, "iVescCase = %d Pos = %d  sendPos = %d PIDPos#11 = %f \r\n",iVescCase,Pos,sendPos,VescData.PidPos);
                    DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                   while(!UART1_TransmitComplete());
                    if((b_ControlledStopBelt)||(PartCount >= SetPartCount)){
                        iVescCase = 59; 
                         LD1_Toggle();
                        }
                    else {
                        iVescCase = 60;
                        LD1_Toggle();
                        }
                    }
                }
            
        break;
                  
        case 100:
            //O_KnifeSol_Clear();
            b_HomeStatusBelts = false;
            Pos =sendPos;
            sprintf((char*)uart1TxBuffer, "Pos overtravel Pos = %d \r\n",Pos);
            while(!UART1_TransmitComplete());
            DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
            while(!UART1_TransmitComplete());
            iVescCase = 105;
        break;
         case 101:
            //O_KnifeSol_Clear();
            b_HomeStatusBelts = false;
            Pos =sendPos;
            sprintf((char*)uart1TxBuffer, "Neg overtravel Pos = %d \r\n",Pos);
            while(!UART1_TransmitComplete());
            DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
            while(!UART1_TransmitComplete());
            iVescCase = 105;
        break;
         case 105:
            iVescCase = iVescCase + 5*(Estop);
        break;
       case 110:
            b_first1 = true;
            EstopDelay++; 
            //O_KnifeSol_Clear();
            b_HomeStatusBelts = false;
            Pos =sendPos;
            if(xTxCanData)
                {
                xTxCanData = 0;
                if (!CAN1_TxFIFOIsFull(0) )
                   {
                    SetDuty(feedroller,0);
                    }
                }
            if (b_first == true) {
                b_first = false;
                sprintf((char*)uart1TxBuffer, "ESTOP RESET Pos = %d Estop = %d\r\n",Pos,Estop);
                while(!UART1_TransmitComplete());
                DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                while(!UART1_TransmitComplete());
            }
            iVescCase = iVescCase + 1*(!(Estop)&&(EstopDelay >= 300000 ));
            
        break;
        
        case 111:
             b_first = true;
             EstopDelay = 0;
            
            if (b_first1 == true) {
                b_first1 = false;
                sprintf((char*)uart1TxBuffer, "ESTOP Done Pos = %d \r\n",Pos);
                while(!UART1_TransmitComplete());
                DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
                while(!UART1_TransmitComplete());
            while(!UART1_TransmitComplete());
                sprintf((char*)uart1TxBuffer, "Pos1 = %d iVescCase = %d sendPos = %d PIDPos#Estop = %f \r\n",Pos,iVescCase,sendPos,VescData.PidPos);
                DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
           while(!UART1_TransmitComplete());
            }
             iVescCase = 0;
        break; 
        
        default:
        break;
    }
    
    //Test if there's data available
    
    if(CAN1_InterruptGet(1, CAN_FIFO_INTERRUPT_RXFULLIF_MASK))
    {
      //Read Data from Vesc Drive on CANBUS Node 1
      receiveVescMessage(feedroller);
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
                CAN1_MessageTransmit(VescID, VescLength,data,TxfifoNum,TXmsgAttr0);
                return;
            }
        else{
            CAN1_MessageTransmit(VescID, VescLength, data,TxfifoNum,TXmsgAttr1);
            xRxValid = CAN1_MessageReceive( (uint32_t*)&VescID, (uint8_t*)&VescLength, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, fifoNum, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr );
            if(xRxValid)
                {
                    memcpy(data, &Mydata, sizeof(Mydata));
                    return;
                }
            }
            return;
        }
}

void receiveVescMessage(int unit_id)
{
    uint32_t ID =  (CMD_ID_STATUS << 8) + unit_id;  //unit_id;  //unit_id; // 2 for saw 1 for traverse
    int8_t SfifoNum = unit_id;  //6;
    uint8_t Length = 8;
    uint8_t Mydata[8] = {0};
    uint16_t Mytimestamp = 0;
    CAN_MSG_RX_ATTRIBUTE MymsgAttr =  CAN_MSG_RX_DATA_FRAME;   //CAN_MSG_RX_DATA_FRAME;  //CAN_MSG_RX_REMOTE_FRAME;
    static bool xMsg1Valid = 0;
    static bool xMsg2Valid = 0;
    static bool xMsg3Valid = 0;
    static bool xMsg4Valid = 0;
    static bool xMsg5Valid = 0;
   
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, SfifoNum, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
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
       
       //      while(!UART1_TransmitComplete());
                    //sprintf((char*)uart1TxBuffer, "Temp = %02dF    SEC = %d   DegLat = %d\r\n", temperatureVal,GPSMod.ucSeconds,GPSMod.latDegrees);
      //          sprintf((char*)uart1TxBuffer, "Duty: %1.3f\r\nTotCurr: %2.2f\r\nRPM: %d\r\nID: %d\r\n Valid = %d\r\n\n",VescData.DutyCycle,VescData.TotCurr,VescData.RPM,ID,xMsg1Valid);
              
       //         DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
      //      while(!UART1_TransmitComplete()); 
    }
    else
    {
       xMsg1Valid = 0; 
    }
    
    ID =  (CMD_ID_STATUS_2 << 8) + unit_id;
    SfifoNum = unit_id +2;  //2;
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, SfifoNum, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
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
 
             // while(!UART1_TransmitComplete());
            //  sprintf((char*)uart1TxBuffer,"AmpChrg: %1.3f\r\nTotCurr: %2.2f\r\nID %d\r\n",VescData.AmpHrsCharged,VescData.AmpHrs,ID);
             // DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
           //   while(!UART1_TransmitComplete()); 
    }
    else
    {
       xMsg2Valid = 0; 
    }
    
    ID =  (CMD_ID_STATUS_3 << 8) + unit_id;
    SfifoNum = unit_id +4; 
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, SfifoNum, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
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
   
    ID =  (CMD_ID_STATUS_4 << 8) + unit_id;
    SfifoNum = unit_id +6;
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, SfifoNum, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
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
    
    ID =  (CMD_ID_STATUS_5 << 8) + unit_id;
    SfifoNum = unit_id +8;
    if( CAN1_MessageReceive( (uint32_t*)&ID, (uint8_t*)&Length, (uint8_t*)&Mydata, (uint16_t*)&Mytimestamp, SfifoNum, (CAN_MSG_RX_ATTRIBUTE*)&MymsgAttr ) )
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
    //VescFlags.bit.ValidMsgs = xMsg1Valid;  // & xMsg2Valid & xMsg3Valid & xMsg4Valid & xMsg5Valid;  
    //VescFlags.bit.ValidMsgs = 1;
     //VescFlags.bit.ValidMsgs = xMsg2Valid & xMsg3Valid & xMsg4Valid & xMsg5Valid;
    
     if (unit_id == 1) {
        Traverse = VescData;
        }
          if (unit_id == 2) {
        Saw = VescData;
        }
        //   while(!UART1_TransmitComplete());
        //        sprintf((char*)uart1TxBuffer, "Valid = %d\r\n ID = %d\r\n vescRPM = %d\r\n SawRPM = %d\r\n",VescData.RPM,ID,xMsg1Valid,Saw.RPM);
        //        DMAC_ChannelTransfer(DMAC_CHANNEL_0, (const void *)uart1TxBuffer,strlen((const char*)uart1TxBuffer),(const void *)&U1TXREG, 1, 1); 
        //   while(!UART1_TransmitComplete()); 
    
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
