/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _HMI_COMMS    /* Guard against multiple inclusion */
#define _HMI_COMMS

/************************INCLUDES********************************/



/***************************DEFINES*******************************/
#define RXBUFFERSIZE  100
#define TXBUFFERSIZE  256

typedef enum
{ 
    HMI_RX, 
    HMI_INIT,
    HMI_AUTO_PAGE,
    HMI_TX,
    HMI_MAX_STATES
}dt_enum_Hmi_State;

typedef union
{
    unsigned char BYTE[4];
    unsigned int INT;
    float FLT;
}dt_Hmi_TxRx_Data;

typedef union
{
    struct
    {
        unsigned NewTxData  : 1;
        unsigned TxDataDone : 1;
        unsigned NewRxData  : 1;
        unsigned InitDone   : 1;
        unsigned RxDataRdy  : 1;
        unsigned RstRxDMA   : 1;
        unsigned bit6 : 1;
        unsigned bit7 : 1;
    }bits;
    unsigned char BYTE;
}dt_Hmi_Data_Ctrl;

typedef struct
{
    dt_Hmi_Data_Ctrl Ctrl;
    dt_enum_Hmi_State State;
}dt_Hmi_Status;


/*AUTO PAGE*/
typedef struct
{
    dt_Hmi_TxRx_Data CurCount;
    dt_Hmi_TxRx_Data CutTime;
    dt_Hmi_TxRx_Data Fault;
    dt_Hmi_TxRx_Data AutoRunning;
}dt_HMI_Tx_Auto;

typedef struct
{
    dt_Hmi_TxRx_Data SetCount;
    dt_Hmi_TxRx_Data HomeOffset;
    dt_Hmi_TxRx_Data Stroke;
    dt_Hmi_TxRx_Data CutSpeedRPM;
}dt_HMI_Rx_Auto;

typedef struct
{
    dt_HMI_Tx_Auto TxData;
    dt_HMI_Rx_Auto RxData;
}dt_HMI_Auto_Page;

typedef struct
{
    
}dt_HMI_Tx_Init;

/***************************VARIABLES*****************************/
extern dt_Hmi_Status HmiStatus;
extern dt_enum_Hmi_State HmiState;
extern dt_HMI_Auto_Page AutoPage;

volatile static char __attribute__ ((aligned (16))) RxBuffer[RXBUFFERSIZE] = {0};
//volatile static char __attribute__ ((aligned (16))) TxBuffer[TXBUFFERSIZE] = {0};
/**************************FUNCTIONS******************************/

void SetupHmiMachine();
void RunHmiStateMachine(char (*RxBuff)[RXBUFFERSIZE]);

#endif

/* *****************************************************************************
 End of File
 * 
 * HMI_SETUP,
    HMI_RX, 
    HMI_TX,
    HMI_MAX_STATES
 */
