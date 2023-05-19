/*******************************************************************************
  UART2 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart2.c

  Summary:
    UART2 PLIB Implementation File

  Description:
    None

*******************************************************************************/

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

#include "device.h"
#include "plib_uart2.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART2 Implementation
// *****************************************************************************
// *****************************************************************************

UART_OBJECT uart2Obj;

void static UART2_ErrorClear( void )
{
    UART_ERROR errors = UART_ERROR_NONE;
    uint8_t dummyData = 0u;

    errors = (UART_ERROR)(U2STA & (_U2STA_OERR_MASK | _U2STA_FERR_MASK | _U2STA_PERR_MASK));

    if(errors != UART_ERROR_NONE)
    {
        /* If it's a overrun error then clear it to flush FIFO */
        if(U2STA & _U2STA_OERR_MASK)
        {
            U2STACLR = _U2STA_OERR_MASK;
        }

        /* Read existing error bytes from FIFO to clear parity and framing error flags */
        while(U2STA & _U2STA_URXDA_MASK)
        {
            dummyData = U2RXREG;
        }

        /* Clear error interrupt flag */
        IFS1CLR = _IFS1_U2EIF_MASK;

        /* Clear up the receive interrupt flag so that RX interrupt is not
         * triggered for error bytes */
        IFS1CLR = _IFS1_U2RXIF_MASK;
    }

    // Ignore the warning
    (void)dummyData;
}

void UART2_Initialize( void )
{
    /* Set up UxMODE bits */
    /* STSEL  = 0 */
    /* PDSEL = 0 */
    /* UEN = 0 */

    U2MODE = 0x8;

    /* Enable UART2 Receiver and Transmitter */
    U2STASET = (_U2STA_UTXEN_MASK | _U2STA_URXEN_MASK | _U2STA_UTXISEL1_MASK );

    /* BAUD Rate register Setup */
    U2BRG = 21;

    /* Disable Interrupts */
    IEC1CLR = _IEC1_U2EIE_MASK;

    IEC1CLR = _IEC1_U2RXIE_MASK;

    IEC1CLR = _IEC1_U2TXIE_MASK;

    /* Initialize instance object */
    uart2Obj.rxBuffer = NULL;
    uart2Obj.rxSize = 0;
    uart2Obj.rxProcessedSize = 0;
    uart2Obj.rxBusyStatus = false;
    uart2Obj.rxCallback = NULL;
    uart2Obj.txBuffer = NULL;
    uart2Obj.txSize = 0;
    uart2Obj.txProcessedSize = 0;
    uart2Obj.txBusyStatus = false;
    uart2Obj.txCallback = NULL;
    uart2Obj.errors = UART_ERROR_NONE;

    /* Turn ON UART2 */
    U2MODESET = _U2MODE_ON_MASK;
}

bool UART2_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    uint32_t status_ctrl;
    bool brgh = 1;
    int32_t uxbrg = 0;

    if((uart2Obj.rxBusyStatus == true) || (uart2Obj.txBusyStatus == true))
    {
        /* Transaction is in progress, so return without updating settings */
        return status;
    }

    if (setup != NULL)
    {
        baud = setup->baudRate;

        if ((baud == 0) || ((setup->dataWidth == UART_DATA_9_BIT) && (setup->parity != UART_PARITY_NONE)))
        {
            return status;
        }

        if(srcClkFreq == 0)
        {
            srcClkFreq = UART2_FrequencyGet();
        }

        /* Calculate BRG value */
        if (brgh == 0)
        {
            uxbrg = (((srcClkFreq >> 4) + (baud >> 1)) / baud ) - 1;
        }
        else
        {
            uxbrg = (((srcClkFreq >> 2) + (baud >> 1)) / baud ) - 1;
        }

        /* Check if the baud value can be set with low baud settings */
        if((uxbrg < 0) || (uxbrg > UINT16_MAX))
        {
            return status;
        }

        /* Turn OFF UART2. Save UTXEN, URXEN and UTXBRK bits as these are cleared upon disabling UART */

        status_ctrl = U2STA & (_U2STA_UTXEN_MASK | _U2STA_URXEN_MASK | _U2STA_UTXBRK_MASK);

        U2MODECLR = _U2MODE_ON_MASK;

        if(setup->dataWidth == UART_DATA_9_BIT)
        {
            /* Configure UART2 mode */
            U2MODE = (U2MODE & (~_U2MODE_PDSEL_MASK)) | setup->dataWidth;
        }
        else
        {
            /* Configure UART2 mode */
            U2MODE = (U2MODE & (~_U2MODE_PDSEL_MASK)) | setup->parity;
        }

        /* Configure UART2 mode */
        U2MODE = (U2MODE & (~_U2MODE_STSEL_MASK)) | setup->stopBits;

        /* Configure UART2 Baud Rate */
        U2BRG = uxbrg;

        U2MODESET = _U2MODE_ON_MASK;

        /* Re-enable UTXEN, URXEN and UTXBRK. */
        U2STASET = status_ctrl;

        status = true;
    }

    return status;
}

bool UART2_AutoBaudQuery( void )
{
    if(U2MODE & _U2MODE_ABAUD_MASK)
        return true;
    else
        return false;
}

void UART2_AutoBaudSet( bool enable )
{
    if( enable == true )
    {
        U2MODESET = _U2MODE_ABAUD_MASK;
    }

    /* Turning off ABAUD if it was on can lead to unpredictable behavior, so that
       direction of control is not allowed in this function.                      */
}

bool UART2_Read(void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* lBuffer = (uint8_t* )buffer;

    if(lBuffer != NULL)
    {
        /* Check if receive request is in progress */
        if(uart2Obj.rxBusyStatus == false)
        {
            /* Clear error flags and flush out error data that may have been received when no active request was pending */
            UART2_ErrorClear();

            uart2Obj.rxBuffer = lBuffer;
            uart2Obj.rxSize = size;
            uart2Obj.rxProcessedSize = 0;
            uart2Obj.rxBusyStatus = true;
            uart2Obj.errors = UART_ERROR_NONE;

            status = true;

            /* Enable UART2_FAULT Interrupt */
            IEC1SET = _IEC1_U2EIE_MASK;

            /* Enable UART2_RX Interrupt */
            IEC1SET = _IEC1_U2RXIE_MASK;
        }
    }

    return status;
}

bool UART2_Write( void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* lBuffer = (uint8_t*)buffer;

    if(lBuffer != NULL)
    {
        /* Check if transmit request is in progress */
        if(uart2Obj.txBusyStatus == false)
        {
            uart2Obj.txBuffer = lBuffer;
            uart2Obj.txSize = size;
            uart2Obj.txProcessedSize = 0;
            uart2Obj.txBusyStatus = true;
            status = true;

            /* Initiate the transfer by writing as many bytes as we can */
            while((!(U2STA & _U2STA_UTXBF_MASK)) && (uart2Obj.txSize > uart2Obj.txProcessedSize) )
            {
                if (( U2MODE & (_U2MODE_PDSEL0_MASK | _U2MODE_PDSEL1_MASK)) == (_U2MODE_PDSEL0_MASK | _U2MODE_PDSEL1_MASK))
                {
                    /* 9-bit mode */
                    U2TXREG = ((uint16_t*)uart2Obj.txBuffer)[uart2Obj.txProcessedSize++];
                }
                else
                {
                    /* 8-bit mode */
                    U2TXREG = uart2Obj.txBuffer[uart2Obj.txProcessedSize++];
                }
            }

            IEC1SET = _IEC1_U2TXIE_MASK;
        }
    }

    return status;
}

UART_ERROR UART2_ErrorGet( void )
{
    UART_ERROR errors = uart2Obj.errors;

    uart2Obj.errors = UART_ERROR_NONE;

    /* All errors are cleared, but send the previous error state */
    return errors;
}

void UART2_ReadCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart2Obj.rxCallback = callback;

    uart2Obj.rxContext = context;
}

bool UART2_ReadIsBusy( void )
{
    return uart2Obj.rxBusyStatus;
}

size_t UART2_ReadCountGet( void )
{
    return uart2Obj.rxProcessedSize;
}

bool UART2_ReadAbort(void)
{
    if (uart2Obj.rxBusyStatus == true)
    {
        /* Disable the fault interrupt */
        IEC1CLR = _IEC1_U2EIE_MASK;

        /* Disable the receive interrupt */
        IEC1CLR = _IEC1_U2RXIE_MASK;

        uart2Obj.rxBusyStatus = false;

        /* If required application should read the num bytes processed prior to calling the read abort API */
        uart2Obj.rxSize = uart2Obj.rxProcessedSize = 0;
    }

    return true;
}

void UART2_WriteCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart2Obj.txCallback = callback;

    uart2Obj.txContext = context;
}

bool UART2_WriteIsBusy( void )
{
    return uart2Obj.txBusyStatus;
}

size_t UART2_WriteCountGet( void )
{
    return uart2Obj.txProcessedSize;
}

static void UART2_FAULT_InterruptHandler (void)
{
    /* Save the error to be reported later */
    uart2Obj.errors = (UART_ERROR)(U2STA & (_U2STA_OERR_MASK | _U2STA_FERR_MASK | _U2STA_PERR_MASK));

    /* Disable the fault interrupt */
    IEC1CLR = _IEC1_U2EIE_MASK;

    /* Disable the receive interrupt */
    IEC1CLR = _IEC1_U2RXIE_MASK;

    /* Clear rx status */
    uart2Obj.rxBusyStatus = false;

    UART2_ErrorClear();

    /* Client must call UARTx_ErrorGet() function to get the errors */
    if( uart2Obj.rxCallback != NULL )
    {
        uart2Obj.rxCallback(uart2Obj.rxContext);
    }
}

static void UART2_RX_InterruptHandler (void)
{
    if(uart2Obj.rxBusyStatus == true)
    {
        while((_U2STA_URXDA_MASK == (U2STA & _U2STA_URXDA_MASK)) && (uart2Obj.rxSize > uart2Obj.rxProcessedSize) )
        {
            if (( U2MODE & (_U2MODE_PDSEL0_MASK | _U2MODE_PDSEL1_MASK)) == (_U2MODE_PDSEL0_MASK | _U2MODE_PDSEL1_MASK))
            {
                /* 9-bit mode */
                ((uint16_t*)uart2Obj.rxBuffer)[uart2Obj.rxProcessedSize++] = (uint16_t )(U2RXREG);
            }
            else
            {
                /* 8-bit mode */
                uart2Obj.rxBuffer[uart2Obj.rxProcessedSize++] = (uint8_t )(U2RXREG);
            }
        }

        /* Clear UART2 RX Interrupt flag */
        IFS1CLR = _IFS1_U2RXIF_MASK;

        /* Check if the buffer is done */
        if(uart2Obj.rxProcessedSize >= uart2Obj.rxSize)
        {
            uart2Obj.rxBusyStatus = false;

            /* Disable the fault interrupt */
            IEC1CLR = _IEC1_U2EIE_MASK;

            /* Disable the receive interrupt */
            IEC1CLR = _IEC1_U2RXIE_MASK;


            if(uart2Obj.rxCallback != NULL)
            {
                uart2Obj.rxCallback(uart2Obj.rxContext);
            }
        }
    }
    else
    {
        // Nothing to process
        ;
    }
}

static void UART2_TX_InterruptHandler (void)
{
    if(uart2Obj.txBusyStatus == true)
    {
        while((!(U2STA & _U2STA_UTXBF_MASK)) && (uart2Obj.txSize > uart2Obj.txProcessedSize) )
        {
            if (( U2MODE & (_U2MODE_PDSEL0_MASK | _U2MODE_PDSEL1_MASK)) == (_U2MODE_PDSEL0_MASK | _U2MODE_PDSEL1_MASK))
            {
                /* 9-bit mode */
                U2TXREG = ((uint16_t*)uart2Obj.txBuffer)[uart2Obj.txProcessedSize++];
            }
            else
            {
                /* 8-bit mode */
                U2TXREG = uart2Obj.txBuffer[uart2Obj.txProcessedSize++];
            }
        }

        /* Clear UART2TX Interrupt flag */
        IFS1CLR = _IFS1_U2TXIF_MASK;

        /* Check if the buffer is done */
        if(uart2Obj.txProcessedSize >= uart2Obj.txSize)
        {
            uart2Obj.txBusyStatus = false;

            /* Disable the transmit interrupt, to avoid calling ISR continuously */
            IEC1CLR = _IEC1_U2TXIE_MASK;

            if(uart2Obj.txCallback != NULL)
            {
                uart2Obj.txCallback(uart2Obj.txContext);
            }
        }
    }
    else
    {
        // Nothing to process
        ;
    }
}

void UART_2_InterruptHandler (void)
{
    /* Call Error handler if Error interrupt flag is set */
    if ((IFS1 & _IFS1_U2EIF_MASK) && (IEC1 & _IEC1_U2EIE_MASK))
    {
        UART2_FAULT_InterruptHandler();
    }
    /* Call RX handler if RX interrupt flag is set */
    if ((IFS1 & _IFS1_U2RXIF_MASK) && (IEC1 & _IEC1_U2RXIE_MASK))
    {
        UART2_RX_InterruptHandler();
    }
    /* Call TX handler if TX interrupt flag is set */
    if ((IFS1 & _IFS1_U2TXIF_MASK) && (IEC1 & _IEC1_U2TXIE_MASK))
    {
        UART2_TX_InterruptHandler();
    }
}


bool UART2_TransmitComplete( void )
{
    bool transmitComplete = false;

    if((U2STA & _U2STA_TRMT_MASK))
    {
        transmitComplete = true;
    }

    return transmitComplete;
}