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

/************************INCLUDES********************************/
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "definitions.h"                // SYS function prototypes
#include "HMIComms.h"
                   

/**********************VARIABLES********************************/
enum ReceiveCmd
{
    CMD_INIT,
    CMD_INPUTS,
    CMD_OUTPUTS,
    CMD_ERRORS,
    CMD_BLADE,
    CMD_CLAMP,
    CMD_TOP,
    CMD_BOTTOM,
    CMD_SHEAR,
    CMD_AUTO,
    CMD_PAGE
};
enum ReceiveCmd RxHmiCmd;
dt_Hmi_Status HmiStatus;
dt_enum_Hmi_State HmiState = HMI_RX;
dt_HMI_Auto_Page AutoPage;
dt_HMI_Auto_Page PrevAutoPage;



char TxHmiCmds[11][10] = {
                            "INIT",
                            "INPUTS",
                            "OUTPUTS",
                            "ERRORS",
                            "BLADE",
                            "CLAMP",
                            "TOP",
                            "BOTTOM",
                            "SHEAR",
                            "AUTO",
                            "PAGE"
                         };

//static char __attribute__ ((aligned (16))) RxBuffer[RXBUFFERSIZE] = {0};
static char __attribute__ ((aligned (16))) TxBuffer[TXBUFFERSIZE] = {0};

/*********************FUNCTIONS**********************************/

static void HmiRxData(dt_enum_Hmi_State *,dt_HMI_Auto_Page *,void *);
static void HmiInitData(dt_enum_Hmi_State *,dt_HMI_Auto_Page *,void *);
static void HmiAutoPage(dt_enum_Hmi_State *,dt_HMI_Auto_Page *,void *);
static void HmiTxData(dt_enum_Hmi_State *,dt_HMI_Auto_Page *,void *);
static int  TestForCmd(void *);
static void EnableTxData();
static void SetupInitTxBuffer(dt_HMI_Auto_Page *);
//static void ClearRxBuffer(char (*RxBuff)[RXBUFFERSIZE]);
static long IntDataBetween(void*, int, int);
static bool NewTxDataFound(dt_HMI_Auto_Page *);
static void SetupTxAutoPgBuffer(dt_HMI_Auto_Page *);


void (* pHmiStateMachine[])(dt_enum_Hmi_State *,dt_HMI_Auto_Page *,void *) = 
{
    &HmiRxData,
    &HmiInitData,
    &HmiAutoPage,
    &HmiTxData
};


void SetupHmiMachine()
{
    //Setup Variables
    HmiStatus.Ctrl.BYTE            = 0;
    HmiStatus.Ctrl.bits.TxDataDone = true;
    HmiStatus.State                = HMI_RX;
    
    //Setup UART DMA Channels and call backs
//    SetupUartRxDMA();
//    SetupUartTxDMA();
}

void RunHmiStateMachine(char (*RxBuff)[RXBUFFERSIZE])
{
    while(HmiState != HMI_MAX_STATES)
    {
       ( *pHmiStateMachine[HmiState] ) (&HmiState, &AutoPage, RxBuff);
    }
    
    //Clear RxBuffer
//    ClearRxBuffer(RxBuff);    
    
    //Reset State when done
    HmiState = HmiStatus.State = HMI_RX;
}


static void HmiRxData(dt_enum_Hmi_State * State, dt_HMI_Auto_Page * AutoPg, void *RxData)
{
    int RxCmd = 0;  //Received Command from HMI
    
    //Check if Data is received
    if(HmiStatus.Ctrl.bits.NewRxData)
    {
        HmiStatus.Ctrl.bits.NewRxData = false;
        HmiStatus.Ctrl.bits.RstRxDMA = true;
        
        //Test for Cmd Received
        RxCmd = TestForCmd(RxData);
        
        //Determine which State to go to next
        switch(RxCmd)
        {
            case CMD_INIT:
                (*State) = HmiStatus.State = HMI_INIT;
                break;
                
            case CMD_AUTO:
                (*State) = HmiStatus.State = HMI_AUTO_PAGE;
                break;
                
            default:
                (*State) = HmiStatus.State = HMI_TX;
                break;
        }
    }
    else
    {
        //If there is no received data then check to transmit data
        (*State) = HmiStatus.State = HMI_TX;
    }
    
}


static void HmiInitData(dt_enum_Hmi_State * State, dt_HMI_Auto_Page * AutoPg, void *RxData)
{
    //Setup all data values by reading them from memory
    AutoPg->RxData.CutSpeedRPM.FLT = 10.3;
    AutoPg->RxData.HomeOffset.FLT  = 2.0;
    AutoPg->RxData.SetCount.INT    = 200;
    AutoPg->RxData.Stroke.FLT      = 1.3;
    AutoPg->TxData.CurCount.INT    = 0;
    AutoPg->TxData.CutTime.INT     = 0;
    AutoPg->TxData.Fault.INT       = 0;
    AutoPg->TxData.AutoRunning.INT = 0;
    
    PrevAutoPage.RxData = AutoPg->RxData;
    PrevAutoPage.TxData = AutoPg->TxData;
    
    //Setup Transmit Buffer
    SetupInitTxBuffer(AutoPg);
    
    //Enable Tx of Data
    EnableTxData();
    
    HmiStatus.Ctrl.bits.InitDone = true;
    
    //Go to Transmit Data
    (*State) = HMI_TX;
}


static void HmiAutoPage(dt_enum_Hmi_State * State, dt_HMI_Auto_Page * AutoPg, void *RxData)
{
    //Pass received data to Auto Page struct
    AutoPg->RxData.CutSpeedRPM.FLT = IntDataBetween(RxData, 2, 3)/1000.0;
    AutoPg->RxData.HomeOffset.FLT  = IntDataBetween(RxData, 3, 4)/1000.0;
    AutoPg->RxData.SetCount.INT    = IntDataBetween(RxData, 4, 5);
    AutoPg->RxData.Stroke.FLT      = IntDataBetween(RxData, 5, 6)/1000.0;
    
    //Signal that Rx Data is transfered
    HmiStatus.Ctrl.bits.RxDataRdy = true;
    
    
    
    (*State) = HMI_TX;
}


static void HmiTxData(dt_enum_Hmi_State * State, dt_HMI_Auto_Page * AutoPg, void *RxData)
{
    //Test for new data to transmit
    if(NewTxDataFound(AutoPg) && HmiStatus.Ctrl.bits.InitDone)
    {
        HmiStatus.Ctrl.bits.NewTxData  = true;
        SetupTxAutoPgBuffer(AutoPg);
    }
    
    //Transmit Data
    if(HmiStatus.Ctrl.bits.NewTxData && HmiStatus.Ctrl.bits.TxDataDone)
    {
        HmiStatus.Ctrl.bits.NewTxData  = false;
        HmiStatus.Ctrl.bits.TxDataDone = false;
        DMAC_ChannelTransfer(DMAC_CHANNEL_1, (const void *)TxBuffer, strlen((const char*)TxBuffer), (const void *)&U2TXREG, 1, 1);
 
        
  //      U2TXREG = 0x0;
 
    }
    
    //Increment State
    (*State)++; 
}


static int  TestForCmd(void *RxData)
{
    int i;
    char *ret = NULL;
    
    //Look at for command in Uart Rx Buffer
    for(i = 0; i < 11; i++)
    {
        ret = strstr(RxData, TxHmiCmds[i]); //Search for needle in haystack
        if(ret != NULL) break; //Break when match is found
    }
    
    return i;
}


static void SetupInitTxBuffer(dt_HMI_Auto_Page *data)
{
    //Clear the Tx Buffer
    memset(TxBuffer,0,TXBUFFERSIZE);
    sprintf(TxBuffer,"$~%s~%d~%d~%d~%d~%d~%d~@",  TxHmiCmds[0],
                                                    data->TxData.AutoRunning.INT,
                                                    (int)(data->RxData.Stroke.FLT*1000),
                                                    (int)(data->RxData.HomeOffset.FLT*1000),
                                                    (int)(data->RxData.CutSpeedRPM.FLT*1000),
                                                    data->RxData.SetCount.INT,
                                                    data->TxData.CurCount.INT);
    
}



static void EnableTxData()
{
    HmiStatus.Ctrl.bits.NewTxData = true;
}


//static void ClearRxBuffer(char (*RxData)[RXBUFFERSIZE])
//{
//    int i;
//    for(i = 0; i<RXBUFFERSIZE; i++)
//    {
//        (*RxData)[i] = 0;
//    }
//   // memset(RxData, 0, sizeof(RxData));
//}

static long IntDataBetween(void* data, int First, int Second)
{
    int CharCnt = 0;
    int cnt = 0;
    int DataSize = strlen(data);
    int i, FirstPos, SecPos;
    long value = 0;
    int power = 0;
    int digit = 0;
    
    FirstPos = SecPos = cnt = 0;
    
    //Search for First '~' in Array
    for(i = 0; i < DataSize; i++)
    {
        if( *((char*)data+i) == '~') 
        {
            FirstPos = i;
            cnt++;
            if(cnt == First) break;
        }
    }
    
    //Search for Second '~' in Array
    for(i = FirstPos+1; i < DataSize; i++)
    {
        if( *((char*)data+i) == '~') 
        {
            SecPos = i;
            CharCnt = SecPos - FirstPos - 1;
            break;
        }
    }
    
    //Calculate Value
    for(i = FirstPos+1; i < SecPos; i++)
    {
        CharCnt -= 1;
        power = (pow(10,CharCnt));
        digit = (*((char*)data+i)-'0');
        value = value + (long)(digit * power);
    }
    
    return value;
}


static bool NewTxDataFound(dt_HMI_Auto_Page *AutoPg)
{
    bool logic = false;
    
    if(PrevAutoPage.TxData.CurCount.INT != AutoPg->TxData.CurCount.INT)
    {
        logic = true;
        PrevAutoPage.TxData.CurCount.INT = AutoPg->TxData.CurCount.INT;
    }
    
    if(PrevAutoPage.TxData.CutTime.INT != AutoPg->TxData.CutTime.INT)
    {
        logic = true;
        PrevAutoPage.TxData.CutTime.INT = AutoPg->TxData.CutTime.INT;
    }
    
    if(PrevAutoPage.TxData.Fault.INT !=AutoPg->TxData.Fault.INT)
    {
        logic = true;
        PrevAutoPage.TxData.Fault.INT = AutoPg->TxData.Fault.INT;
    }
            
    if(PrevAutoPage.TxData.AutoRunning.INT != AutoPg->TxData.AutoRunning.INT)
    {
        logic = true;
        PrevAutoPage.TxData.AutoRunning.INT = AutoPg->TxData.AutoRunning.INT;
    }
    
    return logic;
}


static void SetupTxAutoPgBuffer(dt_HMI_Auto_Page *data)
{
    //Clear the Tx Buffer
    memset(TxBuffer,0,strlen(TxBuffer));
    sprintf(TxBuffer,"$~%s~%d~%d~%d~@",TxHmiCmds[9],
                                     data->TxData.CutTime.INT,
                                     data->TxData.CurCount.INT,
                                    // data->TxData.Fault.INT,
                                     data->TxData.AutoRunning.INT);
}

/* *****************************************************************************
 End of File
 */
