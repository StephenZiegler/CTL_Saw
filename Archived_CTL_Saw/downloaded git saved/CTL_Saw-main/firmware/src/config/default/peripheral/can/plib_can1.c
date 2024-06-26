/*******************************************************************************
  Controller Area Network (CAN) Peripheral Library Source File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_can1.c

  Summary:
    CAN peripheral library interface.

  Description:
    This file defines the interface to the CAN peripheral library. This
    library provides access to and control of the associated peripheral
    instance.

  Remarks:
    None.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
//DOM-IGNORE-END
// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include <sys/kmem.h>
#include "plib_can1.h"

// *****************************************************************************
// *****************************************************************************
// Global Data
// *****************************************************************************
// *****************************************************************************
/* Number of configured FIFO */
#define CAN_NUM_OF_FIFO             12
/* Maximum number of CAN Message buffers in each FIFO */
#define CAN_FIFO_MESSAGE_BUFFER_MAX 32

#define CAN_CONFIGURATION_MODE      0x4
#define CAN_OPERATION_MODE          0x0
#define CAN_NUM_OF_FILTER           10
/* FIFO Offset in word (4 bytes) */
#define CAN_FIFO_OFFSET             0x10
/* Filter Offset in word (4 bytes) */
#define CAN_FILTER_OFFSET           0x4
/* Acceptance Mask Offset in word (4 bytes) */
#define CAN_ACCEPTANCE_MASK_OFFSET  0x4
#define CAN_MESSAGE_RAM_CONFIG_SIZE 19
#define CAN_MSG_IDE_MASK            0x10000000
#define CAN_MSG_SID_MASK            0x000
#define CAN_MSG_TIMESTAMP_MASK      0xFFFF0000
#define CAN_MSG_EID_MASK            0x1FFFFFFF
#define CAN_MSG_DLC_MASK            0xF
#define CAN_MSG_RTR_MASK            0x200
#define CAN_MSG_SRR_MASK            0x20000000

static CAN_TX_RX_MSG_BUFFER __attribute__((coherent, aligned(32))) can_message_buffer[CAN_MESSAGE_RAM_CONFIG_SIZE];

// *****************************************************************************
// *****************************************************************************
// CAN1 PLib Interface Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    void CAN1_Initialize(void)

   Summary:
    Initializes given instance of the CAN peripheral.

   Precondition:
    None.

   Parameters:
    None.

   Returns:
    None
*/
void CAN1_Initialize(void)
{
    /* Switch the CAN module ON */
    C1CONSET = _C1CON_ON_MASK;

    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    C1CON = (C1CON & ~_C1CON_REQOP_MASK) | ((CAN_CONFIGURATION_MODE << _C1CON_REQOP_POSITION) & _C1CON_REQOP_MASK);
    while(((C1CON & _C1CON_OPMOD_MASK) >> _C1CON_OPMOD_POSITION) != CAN_CONFIGURATION_MODE);

    /* Set the Bitrate to 250 Kbps */
    C1CFG = ((7 << _C1CFG_BRP_POSITION) & _C1CFG_BRP_MASK)
                            | ((2 << _C1CFG_SJW_POSITION) & _C1CFG_SJW_MASK)
                            | ((2 << _C1CFG_SEG2PH_POSITION) & _C1CFG_SEG2PH_MASK)
                            | ((7 << _C1CFG_SEG1PH_POSITION) & _C1CFG_SEG1PH_MASK)
                            | ((7 << _C1CFG_PRSEG_POSITION) & _C1CFG_PRSEG_MASK)
                            | _C1CFG_SEG2PHTS_MASK | _C1CFG_SAM_MASK;

    /* Set FIFO base address for all message buffers */
    C1FIFOBA = (uint32_t)KVA_TO_PA(can_message_buffer);

    /* Configure CAN FIFOs */
    C1FIFOCON0 = (((8 - 1) << _C1FIFOCON0_FSIZE_POSITION) & _C1FIFOCON0_FSIZE_MASK) | _C1FIFOCON0_TXEN_MASK | ((0x3 << _C1FIFOCON0_TXPRI_POSITION) & _C1FIFOCON0_TXPRI_MASK) | ((0x1 << _C1FIFOCON0_RTREN_POSITION) & _C1FIFOCON0_RTREN_MASK);
    C1FIFOCON1 = (((1 - 1) << _C1FIFOCON1_FSIZE_POSITION) & _C1FIFOCON1_FSIZE_MASK);
    C1FIFOCON2 = (((1 - 1) << _C1FIFOCON2_FSIZE_POSITION) & _C1FIFOCON2_FSIZE_MASK);
    C1FIFOCON3 = (((1 - 1) << _C1FIFOCON3_FSIZE_POSITION) & _C1FIFOCON3_FSIZE_MASK);
    C1FIFOCON4 = (((1 - 1) << _C1FIFOCON4_FSIZE_POSITION) & _C1FIFOCON4_FSIZE_MASK);
    C1FIFOCON5 = (((1 - 1) << _C1FIFOCON5_FSIZE_POSITION) & _C1FIFOCON5_FSIZE_MASK);
    C1FIFOCON6 = (((1 - 1) << _C1FIFOCON6_FSIZE_POSITION) & _C1FIFOCON6_FSIZE_MASK);
    C1FIFOCON7 = (((1 - 1) << _C1FIFOCON7_FSIZE_POSITION) & _C1FIFOCON7_FSIZE_MASK);
    C1FIFOCON8 = (((1 - 1) << _C1FIFOCON8_FSIZE_POSITION) & _C1FIFOCON8_FSIZE_MASK);
    C1FIFOCON9 = (((1 - 1) << _C1FIFOCON9_FSIZE_POSITION) & _C1FIFOCON9_FSIZE_MASK);
    C1FIFOCON10 = (((1 - 1) << _C1FIFOCON10_FSIZE_POSITION) & _C1FIFOCON10_FSIZE_MASK);
    C1FIFOCON11 = (((1 - 1) << _C1FIFOCON11_FSIZE_POSITION) & _C1FIFOCON11_FSIZE_MASK);

    /* Configure CAN Filters */
    C1RXF0 = (2305 & _C1RXF0_EID_MASK) | (((2305 & 0x1FFC0000u) >> 18u) << _C1RXF0_SID_POSITION) | _C1RXF0_EXID_MASK;
    C1FLTCON0SET = ((0x1 << _C1FLTCON0_FSEL0_POSITION) & _C1FLTCON0_FSEL0_MASK)
                                                         | ((0x0 << _C1FLTCON0_MSEL0_POSITION) & _C1FLTCON0_MSEL0_MASK)| _C1FLTCON0_FLTEN0_MASK;
    C1RXF1 = (2306 & _C1RXF1_EID_MASK) | (((2306 & 0x1FFC0000u) >> 18u) << _C1RXF1_SID_POSITION) | _C1RXF1_EXID_MASK;
    C1FLTCON0SET = ((0x2 << _C1FLTCON0_FSEL1_POSITION) & _C1FLTCON0_FSEL1_MASK)
                                                         | ((0x0 << _C1FLTCON0_MSEL1_POSITION) & _C1FLTCON0_MSEL1_MASK)| _C1FLTCON0_FLTEN1_MASK;
    C1RXF2 = (3585 & _C1RXF2_EID_MASK) | (((3585 & 0x1FFC0000u) >> 18u) << _C1RXF2_SID_POSITION) | _C1RXF2_EXID_MASK;
    C1FLTCON0SET = ((0x3 << _C1FLTCON0_FSEL2_POSITION) & _C1FLTCON0_FSEL2_MASK)
                                                         | ((0x0 << _C1FLTCON0_MSEL2_POSITION) & _C1FLTCON0_MSEL2_MASK)| _C1FLTCON0_FLTEN2_MASK;
    C1RXF3 = (3586 & _C1RXF3_EID_MASK) | (((3586 & 0x1FFC0000u) >> 18u) << _C1RXF3_SID_POSITION) | _C1RXF3_EXID_MASK;
    C1FLTCON0SET = ((0x4 << _C1FLTCON0_FSEL3_POSITION) & _C1FLTCON0_FSEL3_MASK)
                                                         | ((0x0 << _C1FLTCON0_MSEL3_POSITION) & _C1FLTCON0_MSEL3_MASK)| _C1FLTCON0_FLTEN3_MASK;
    C1RXF4 = (3841 & _C1RXF4_EID_MASK) | (((3841 & 0x1FFC0000u) >> 18u) << _C1RXF4_SID_POSITION) | _C1RXF4_EXID_MASK;
    C1FLTCON1SET = ((0x5 << _C1FLTCON1_FSEL4_POSITION) & _C1FLTCON1_FSEL4_MASK)
                                                         | ((0x0 << _C1FLTCON1_MSEL4_POSITION) & _C1FLTCON1_MSEL4_MASK)| _C1FLTCON1_FLTEN4_MASK;
    C1RXF5 = (3842 & _C1RXF5_EID_MASK) | (((3842 & 0x1FFC0000u) >> 18u) << _C1RXF5_SID_POSITION) | _C1RXF5_EXID_MASK;
    C1FLTCON1SET = ((0x6 << _C1FLTCON1_FSEL5_POSITION) & _C1FLTCON1_FSEL5_MASK)
                                                         | ((0x0 << _C1FLTCON1_MSEL5_POSITION) & _C1FLTCON1_MSEL5_MASK)| _C1FLTCON1_FLTEN5_MASK;
    C1RXF6 = (4097 & _C1RXF6_EID_MASK) | (((4097 & 0x1FFC0000u) >> 18u) << _C1RXF6_SID_POSITION) | _C1RXF6_EXID_MASK;
    C1FLTCON1SET = ((0x7 << _C1FLTCON1_FSEL6_POSITION) & _C1FLTCON1_FSEL6_MASK)
                                                         | ((0x0 << _C1FLTCON1_MSEL6_POSITION) & _C1FLTCON1_MSEL6_MASK)| _C1FLTCON1_FLTEN6_MASK;
    C1RXF7 = (3842 & _C1RXF7_EID_MASK) | (((3842 & 0x1FFC0000u) >> 18u) << _C1RXF7_SID_POSITION) | _C1RXF7_EXID_MASK;
    C1FLTCON1SET = ((0x8 << _C1FLTCON1_FSEL7_POSITION) & _C1FLTCON1_FSEL7_MASK)
                                                         | ((0x0 << _C1FLTCON1_MSEL7_POSITION) & _C1FLTCON1_MSEL7_MASK)| _C1FLTCON1_FLTEN7_MASK;
    C1RXF8 = (6913 & _C1RXF8_EID_MASK) | (((6913 & 0x1FFC0000u) >> 18u) << _C1RXF8_SID_POSITION) | _C1RXF8_EXID_MASK;
    C1FLTCON2SET = ((0x9 << _C1FLTCON2_FSEL8_POSITION) & _C1FLTCON2_FSEL8_MASK)
                                                         | ((0x0 << _C1FLTCON2_MSEL8_POSITION) & _C1FLTCON2_MSEL8_MASK)| _C1FLTCON2_FLTEN8_MASK;
    C1RXF9 = (6914 & _C1RXF9_EID_MASK) | (((6914 & 0x1FFC0000u) >> 18u) << _C1RXF9_SID_POSITION) | _C1RXF9_EXID_MASK;
    C1FLTCON2SET = ((0xa << _C1FLTCON2_FSEL9_POSITION) & _C1FLTCON2_FSEL9_MASK)
                                                         | ((0x0 << _C1FLTCON2_MSEL9_POSITION) & _C1FLTCON2_MSEL9_MASK)| _C1FLTCON2_FLTEN9_MASK;

    /* Configure CAN Acceptance Filter Masks */
    C1RXM0 = (262143 & _C1RXM0_EID_MASK) | (((262143 & 0x1FFC0000u) >> 18u) << _C1RXM0_SID_POSITION) | _C1RXM0_MIDE_MASK;

    /* Switch the CAN module to CAN_OPERATION_MODE. Wait until the switch is complete */
    C1CON = (C1CON & ~_C1CON_REQOP_MASK) | ((CAN_OPERATION_MODE << _C1CON_REQOP_POSITION) & _C1CON_REQOP_MASK);
    while(((C1CON & _C1CON_OPMOD_MASK) >> _C1CON_OPMOD_POSITION) != CAN_OPERATION_MODE);
}

// *****************************************************************************
/* Function:
    bool CAN1_MessageTransmit(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum, CAN_MSG_TX_ATTRIBUTE msgAttr)

   Summary:
    Transmits a message into CAN bus.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    id          - 11-bit / 29-bit identifier (ID).
    length      - length of data buffer in number of bytes.
    data        - pointer to source data buffer
    fifoNum     - FIFO number
    msgAttr     - Data frame or Remote frame to be transmitted

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN1_MessageTransmit(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum, CAN_MSG_TX_ATTRIBUTE msgAttr)
{
    CAN_TX_RX_MSG_BUFFER *txMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if ((fifoNum > (CAN_NUM_OF_FIFO - 1)) || (data == NULL))
    {
        return status;
    }

    if ((*(volatile uint32_t *)(&C1FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & _C1FIFOINT0_TXNFULLIF_MASK) == _C1FIFOINT0_TXNFULLIF_MASK)
    {
        txMessage = (CAN_TX_RX_MSG_BUFFER *)PA_TO_KVA1(*(volatile uint32_t *)(&C1FIFOUA0 + (fifoNum * CAN_FIFO_OFFSET)));

        /* Check the id whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if (id > CAN_MSG_SID_MASK)
        {
            txMessage->msgSID = (id & CAN_MSG_EID_MASK) >> 18;
            txMessage->msgEID = ((id & 0x3FFFF) << 10) | CAN_MSG_IDE_MASK;
        }
        else
        {
            txMessage->msgSID = id;
            txMessage->msgEID = 0;
        }

        if (msgAttr == CAN_MSG_TX_REMOTE_FRAME)
        {
            txMessage->msgEID |= CAN_MSG_RTR_MASK;
        }
        else
        {
            if (length > 8)
            {
                length = 8;
            }
            txMessage->msgEID |= length;

            while(count < length)
            {
                txMessage->msgData[count++] = *data++;
            }
        }

        /* Request the transmit */
        *(volatile uint32_t *)(&C1FIFOCON0SET + (fifoNum * CAN_FIFO_OFFSET)) = _C1FIFOCON0_UINC_MASK;
        *(volatile uint32_t *)(&C1FIFOCON0SET + (fifoNum * CAN_FIFO_OFFSET)) = _C1FIFOCON0_TXREQ_MASK;

        status = true;
    }
    return status;
}

// *****************************************************************************
/* Function:
    bool CAN1_MessageReceive(uint32_t *id, uint8_t *length, uint8_t *data, uint16_t *timestamp, uint8_t fifoNum, CAN_MSG_RX_ATTRIBUTE *msgAttr)

   Summary:
    Receives a message from CAN bus.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    id          - Pointer to 11-bit / 29-bit identifier (ID) to be received.
    length      - Pointer to data length in number of bytes to be received.
    data        - Pointer to destination data buffer
    timestamp   - Pointer to Rx message timestamp, timestamp value is 0 if Timestamp is disabled in C1CON
    fifoNum     - FIFO number
    msgAttr     - Data frame or Remote frame to be received

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN1_MessageReceive(uint32_t *id, uint8_t *length, uint8_t *data, uint16_t *timestamp, uint8_t fifoNum, CAN_MSG_RX_ATTRIBUTE *msgAttr)
{
    CAN_TX_RX_MSG_BUFFER *rxMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if ((fifoNum > (CAN_NUM_OF_FIFO - 1)) || (data == NULL) || (length == NULL) || (id == NULL))
    {
        return status;
    }

    /* Check if there is a message available in FIFO */
    if ((*(volatile uint32_t *)(&C1FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & _C1FIFOINT0_RXNEMPTYIF_MASK) == _C1FIFOINT0_RXNEMPTYIF_MASK)
    {
        /* Get a pointer to RX message buffer */
        rxMessage = (CAN_TX_RX_MSG_BUFFER *)PA_TO_KVA1(*(volatile uint32_t *)(&C1FIFOUA0 + (fifoNum * CAN_FIFO_OFFSET)));

        /* Check if it's a extended message type */
        if (rxMessage->msgEID & CAN_MSG_IDE_MASK)
        {
            *id = ((rxMessage->msgSID & CAN_MSG_SID_MASK) << 18) | ((rxMessage->msgEID >> 10) & _C1RXM0_EID_MASK);
            if (rxMessage->msgEID & CAN_MSG_RTR_MASK)
            {
                *msgAttr = CAN_MSG_RX_REMOTE_FRAME;
            }
            else
            {
                *msgAttr = CAN_MSG_RX_DATA_FRAME;
            }
        }
        else
        {
            *id = rxMessage->msgSID & CAN_MSG_SID_MASK;
            if (rxMessage->msgEID & CAN_MSG_SRR_MASK)
            {
                *msgAttr = CAN_MSG_RX_REMOTE_FRAME;
            }
            else
            {
                *msgAttr = CAN_MSG_RX_DATA_FRAME;
            }
        }

        *length = rxMessage->msgEID & CAN_MSG_DLC_MASK;

        /* Copy the data into the payload */
        while (count < *length)
        {
           *data++ = rxMessage->msgData[count++];
        }

        if (timestamp != NULL)
        {
            *timestamp = (rxMessage->msgSID & CAN_MSG_TIMESTAMP_MASK) >> 16;
        }

        /* Message processing is done, update the message buffer pointer. */
        *(volatile uint32_t *)(&C1FIFOCON0SET + (fifoNum * CAN_FIFO_OFFSET)) = _C1FIFOCON0_UINC_MASK;

        /* Message is processed successfully, so return true */
        status = true;
    }

    return status;
}

// *****************************************************************************
/* Function:
    void CAN1_MessageAbort(uint8_t fifoNum)

   Summary:
    Abort request for a FIFO.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoNum - FIFO number

   Returns:
    None.
*/
void CAN1_MessageAbort(uint8_t fifoNum)
{
    if (fifoNum > (CAN_NUM_OF_FIFO - 1))
    {
        return;
    }
    *(volatile uint32_t *)(&C1FIFOCON0CLR + (fifoNum * CAN_FIFO_OFFSET)) = _C1FIFOCON0_TXREQ_MASK;
}

// *****************************************************************************
/* Function:
    void CAN1_MessageAcceptanceFilterSet(uint8_t filterNum, uint32_t id)

   Summary:
    Set Message acceptance filter configuration.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    filterNum - Filter number
    id        - 11-bit or 29-bit identifier

   Returns:
    None.
*/
void CAN1_MessageAcceptanceFilterSet(uint8_t filterNum, uint32_t id)
{
    uint32_t filterEnableBit = 0;
    uint8_t filterRegIndex = 0;

    if (filterNum < CAN_NUM_OF_FILTER)
    {
        filterRegIndex = filterNum >> 2;
        filterEnableBit = (filterNum % 4 == 0)? _C1FLTCON0_FLTEN0_MASK : 1 << ((((filterNum % 4) + 1) * 8) - 1);

        *(volatile uint32_t *)(&C1FLTCON0CLR + (filterRegIndex * CAN_FILTER_OFFSET)) = filterEnableBit;

        if (id > CAN_MSG_SID_MASK)
        {
            *(volatile uint32_t *)(&C1RXF0 + (filterNum * CAN_FILTER_OFFSET)) = (id & _C1RXF0_EID_MASK)
                                                                           | (((id & 0x1FFC0000u) >> 18) << _C1RXF0_SID_POSITION)
                                                                           | _C1RXF0_EXID_MASK;
        }
        else
        {
            *(volatile uint32_t *)(&C1RXF0 + (filterNum * CAN_FILTER_OFFSET)) = (id & CAN_MSG_SID_MASK) << _C1RXF0_SID_POSITION;
        }
        *(volatile uint32_t *)(&C1FLTCON0SET + (filterRegIndex * CAN_FILTER_OFFSET)) = filterEnableBit;
    }
}

// *****************************************************************************
/* Function:
    uint32_t CAN1_MessageAcceptanceFilterGet(uint8_t filterNum)

   Summary:
    Get Message acceptance filter configuration.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    filterNum - Filter number

   Returns:
    Returns Message acceptance filter identifier
*/
uint32_t CAN1_MessageAcceptanceFilterGet(uint8_t filterNum)
{
    uint32_t id = 0;

    if (filterNum < CAN_NUM_OF_FILTER)
    {
        if (*(volatile uint32_t *)(&C1RXF0 + (filterNum * CAN_FILTER_OFFSET)) & _C1RXF0_EXID_MASK)
        {
            id = (*(volatile uint32_t *)(&C1RXF0 + (filterNum * CAN_FILTER_OFFSET)) & _C1RXF0_EID_MASK)
               | ((*(volatile uint32_t *)(&C1RXF0 + (filterNum * CAN_FILTER_OFFSET)) & _C1RXF0_SID_MASK) >> 3);
        }
        else
        {
            id = (*(volatile uint32_t *)(&C1RXF0 + (filterNum * CAN_FILTER_OFFSET)) & _C1RXF0_SID_MASK) >> _C1RXF0_SID_POSITION;
        }
    }
    return id;
}

// *****************************************************************************
/* Function:
    void CAN1_MessageAcceptanceFilterMaskSet(uint8_t acceptanceFilterMaskNum, uint32_t id)

   Summary:
    Set Message acceptance filter mask configuration.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    acceptanceFilterMaskNum - Acceptance Filter Mask number (0 to 3)
    id                      - 11-bit or 29-bit identifier

   Returns:
    None.
*/
void CAN1_MessageAcceptanceFilterMaskSet(uint8_t acceptanceFilterMaskNum, uint32_t id)
{
    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    C1CON = (C1CON & ~_C1CON_REQOP_MASK) | ((CAN_CONFIGURATION_MODE << _C1CON_REQOP_POSITION) & _C1CON_REQOP_MASK);
    while(((C1CON & _C1CON_OPMOD_MASK) >> _C1CON_OPMOD_POSITION) != CAN_CONFIGURATION_MODE);

    if (id > CAN_MSG_SID_MASK)
    {
        *(volatile uint32_t *)(&C1RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) = (id & _C1RXM0_EID_MASK)
                                                                       | (((id & 0x1FFC0000u) >> 18) << _C1RXM0_SID_POSITION) | _C1RXM0_MIDE_MASK;
    }
    else
    {
        *(volatile uint32_t *)(&C1RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) = (id & CAN_MSG_SID_MASK) << _C1RXM0_SID_POSITION;
    }

    /* Switch the CAN module to CAN_OPERATION_MODE. Wait until the switch is complete */
    C1CON = (C1CON & ~_C1CON_REQOP_MASK) | ((CAN_OPERATION_MODE << _C1CON_REQOP_POSITION) & _C1CON_REQOP_MASK);
    while(((C1CON & _C1CON_OPMOD_MASK) >> _C1CON_OPMOD_POSITION) != CAN_OPERATION_MODE);
}

// *****************************************************************************
/* Function:
    uint32_t CAN1_MessageAcceptanceFilterMaskGet(uint8_t acceptanceFilterMaskNum)

   Summary:
    Get Message acceptance filter mask configuration.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    acceptanceFilterMaskNum - Acceptance Filter Mask number (0 to 3)

   Returns:
    Returns Message acceptance filter mask.
*/
uint32_t CAN1_MessageAcceptanceFilterMaskGet(uint8_t acceptanceFilterMaskNum)
{
    uint32_t id = 0;

    if (*(volatile uint32_t *)(&C1RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) & _C1RXM0_MIDE_MASK)
    {
        id = (*(volatile uint32_t *)(&C1RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) & _C1RXM0_EID_MASK)
           | ((*(volatile uint32_t *)(&C1RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) & _C1RXM0_SID_MASK) >> 3);
    }
    else
    {
        id = (*(volatile uint32_t *)(&C1RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) & _C1RXM0_SID_MASK) >> _C1RXM0_SID_POSITION;
    }
    return id;
}

// *****************************************************************************
/* Function:
    CAN_ERROR CAN1_ErrorGet(void)

   Summary:
    Returns the error during transfer.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    None.

   Returns:
    Error during transfer.
*/
CAN_ERROR CAN1_ErrorGet(void)
{
    CAN_ERROR error = CAN_ERROR_NONE;
    uint32_t errorStatus = C1TREC;

    /* Check if error occurred */
    error = (CAN_ERROR)((errorStatus & _C1TREC_EWARN_MASK) |
                        (errorStatus & _C1TREC_RXWARN_MASK) |
                        (errorStatus & _C1TREC_TXWARN_MASK) |
                        (errorStatus & _C1TREC_RXBP_MASK) |
                        (errorStatus & _C1TREC_TXBP_MASK) |
                        (errorStatus & _C1TREC_TXBO_MASK));

    return error;
}

// *****************************************************************************
/* Function:
    void CAN1_ErrorCountGet(uint8_t *txErrorCount, uint8_t *rxErrorCount)

   Summary:
    Returns the transmit and receive error count during transfer.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    txErrorCount - Transmit Error Count to be received
    rxErrorCount - Receive Error Count to be received

   Returns:
    None.
*/
void CAN1_ErrorCountGet(uint8_t *txErrorCount, uint8_t *rxErrorCount)
{
    *txErrorCount = (uint8_t)((C1TREC & _C1TREC_TERRCNT_MASK) >> _C1TREC_TERRCNT_POSITION);
    *rxErrorCount = (uint8_t)(C1TREC & _C1TREC_RERRCNT_MASK);
}

// *****************************************************************************
/* Function:
    bool CAN1_InterruptGet(uint8_t fifoNum, CAN_FIFO_INTERRUPT_FLAG_MASK fifoInterruptFlagMask)

   Summary:
    Returns the FIFO Interrupt status.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoNum               - FIFO number
    fifoInterruptFlagMask - FIFO interrupt flag mask

   Returns:
    true - Requested fifo interrupt is occurred.
    false - Requested fifo interrupt is not occurred.
*/
bool CAN1_InterruptGet(uint8_t fifoNum, CAN_FIFO_INTERRUPT_FLAG_MASK fifoInterruptFlagMask)
{
    if (fifoNum > (CAN_NUM_OF_FIFO - 1))
    {
        return false;
    }
    return ((*(volatile uint32_t *)(&C1FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & fifoInterruptFlagMask) != 0x0);
}

// *****************************************************************************
/* Function:
    bool CAN1_TxFIFOIsFull(uint8_t fifoNum)

   Summary:
    Returns true if Tx FIFO is full otherwise false.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoNum - FIFO number

   Returns:
    true  - Tx FIFO is full.
    false - Tx FIFO is not full.
*/
bool CAN1_TxFIFOIsFull(uint8_t fifoNum)
{
    return ((*(volatile uint32_t *)(&C1FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & _C1FIFOINT0_TXNFULLIF_MASK) != _C1FIFOINT0_TXNFULLIF_MASK);
}

// *****************************************************************************
/* Function:
    bool CAN1_AutoRTRResponseSet(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum)

   Summary:
    Set the Auto RTR response for remote transmit request.

   Precondition:
    CAN1_Initialize must have been called for the associated CAN instance.
    Auto RTR Enable must be set to 0x1 for the requested Transmit FIFO in MHC configuration.

   Parameters:
    id          - 11-bit / 29-bit identifier (ID).
    length      - length of data buffer in number of bytes.
    data        - pointer to source data buffer
    fifoNum     - FIFO number

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN1_AutoRTRResponseSet(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum)
{
    CAN_TX_RX_MSG_BUFFER *txMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if ((*(volatile uint32_t *)(&C1FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & _C1FIFOINT0_TXNFULLIF_MASK) == _C1FIFOINT0_TXNFULLIF_MASK)
    {
        txMessage = (CAN_TX_RX_MSG_BUFFER *)PA_TO_KVA1(*(volatile uint32_t *)(&C1FIFOUA0 + (fifoNum * CAN_FIFO_OFFSET)));

        /* Check the id whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if (id > CAN_MSG_SID_MASK)
        {
            txMessage->msgSID = (id & CAN_MSG_EID_MASK) >> 18;
            txMessage->msgEID = ((id & 0x3FFFF) << 10) | CAN_MSG_IDE_MASK;
        }
        else
        {
            txMessage->msgSID = id;
            txMessage->msgEID = 0;
        }

        if (length > 8)
        {
            length = 8;
        }
        txMessage->msgEID |= length;

        while(count < length)
        {
            txMessage->msgData[count++] = *data++;
        }

        /* Set UINC to respond to RTR */
        *(volatile uint32_t *)(&C1FIFOCON0SET + (fifoNum * CAN_FIFO_OFFSET)) = _C1FIFOCON0_UINC_MASK;

        status = true;
    }
    return status;
}

