/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

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

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for LD4 pin ***/
#define LD4_Set()               (LATGSET = (1<<15))
#define LD4_Clear()             (LATGCLR = (1<<15))
#define LD4_Toggle()            (LATGINV= (1<<15))
#define LD4_OutputEnable()      (TRISGCLR = (1<<15))
#define LD4_InputEnable()       (TRISGSET = (1<<15))
#define LD4_Get()               ((PORTG >> 15) & 0x1)
#define LD4_PIN                  GPIO_PIN_RG15

/*** Macros for O_BeltClampSol pin ***/
#define O_BeltClampSol_Set()               (LATESET = (1<<5))
#define O_BeltClampSol_Clear()             (LATECLR = (1<<5))
#define O_BeltClampSol_Toggle()            (LATEINV= (1<<5))
#define O_BeltClampSol_OutputEnable()      (TRISECLR = (1<<5))
#define O_BeltClampSol_InputEnable()       (TRISESET = (1<<5))
#define O_BeltClampSol_Get()               ((PORTE >> 5) & 0x1)
#define O_BeltClampSol_PIN                  GPIO_PIN_RE5

/*** Macros for O_StartLed pin ***/
#define O_StartLed_Set()               (LATESET = (1<<6))
#define O_StartLed_Clear()             (LATECLR = (1<<6))
#define O_StartLed_Toggle()            (LATEINV= (1<<6))
#define O_StartLed_OutputEnable()      (TRISECLR = (1<<6))
#define O_StartLed_InputEnable()       (TRISESET = (1<<6))
#define O_StartLed_Get()               ((PORTE >> 6) & 0x1)
#define O_StartLed_PIN                  GPIO_PIN_RE6

/*** Macros for O_Sharpen pin ***/
#define O_Sharpen_Set()               (LATESET = (1<<7))
#define O_Sharpen_Clear()             (LATECLR = (1<<7))
#define O_Sharpen_Toggle()            (LATEINV= (1<<7))
#define O_Sharpen_OutputEnable()      (TRISECLR = (1<<7))
#define O_Sharpen_InputEnable()       (TRISESET = (1<<7))
#define O_Sharpen_Get()               ((PORTE >> 7) & 0x1)
#define O_Sharpen_PIN                  GPIO_PIN_RE7

/*** Macros for I_Infeed pin ***/
#define I_Infeed_Set()               (LATCSET = (1<<4))
#define I_Infeed_Clear()             (LATCCLR = (1<<4))
#define I_Infeed_Toggle()            (LATCINV= (1<<4))
#define I_Infeed_OutputEnable()      (TRISCCLR = (1<<4))
#define I_Infeed_InputEnable()       (TRISCSET = (1<<4))
#define I_Infeed_Get()               ((PORTC >> 4) & 0x1)
#define I_Infeed_PIN                  GPIO_PIN_RC4

/*** Macros for BTN1 pin ***/
#define BTN1_Set()               (LATGSET = (1<<6))
#define BTN1_Clear()             (LATGCLR = (1<<6))
#define BTN1_Toggle()            (LATGINV= (1<<6))
#define BTN1_OutputEnable()      (TRISGCLR = (1<<6))
#define BTN1_InputEnable()       (TRISGSET = (1<<6))
#define BTN1_Get()               ((PORTG >> 6) & 0x1)
#define BTN1_PIN                  GPIO_PIN_RG6

/*** Macros for BTN2 pin ***/
#define BTN2_Set()               (LATGSET = (1<<7))
#define BTN2_Clear()             (LATGCLR = (1<<7))
#define BTN2_Toggle()            (LATGINV= (1<<7))
#define BTN2_OutputEnable()      (TRISGCLR = (1<<7))
#define BTN2_InputEnable()       (TRISGSET = (1<<7))
#define BTN2_Get()               ((PORTG >> 7) & 0x1)
#define BTN2_PIN                  GPIO_PIN_RG7

/*** Macros for I_Start pin ***/
#define I_Start_Set()               (LATDSET = (1<<9))
#define I_Start_Clear()             (LATDCLR = (1<<9))
#define I_Start_Toggle()            (LATDINV= (1<<9))
#define I_Start_OutputEnable()      (TRISDCLR = (1<<9))
#define I_Start_InputEnable()       (TRISDSET = (1<<9))
#define I_Start_Get()               ((PORTD >> 9) & 0x1)
#define I_Start_PIN                  GPIO_PIN_RD9

/*** Macros for I_BeltDwn pin ***/
#define I_BeltDwn_Set()               (LATDSET = (1<<10))
#define I_BeltDwn_Clear()             (LATDCLR = (1<<10))
#define I_BeltDwn_Toggle()            (LATDINV= (1<<10))
#define I_BeltDwn_OutputEnable()      (TRISDCLR = (1<<10))
#define I_BeltDwn_InputEnable()       (TRISDSET = (1<<10))
#define I_BeltDwn_Get()               ((PORTD >> 10) & 0x1)
#define I_BeltDwn_PIN                  GPIO_PIN_RD10

/*** Macros for I_BladeUp pin ***/
#define I_BladeUp_Set()               (LATDSET = (1<<0))
#define I_BladeUp_Clear()             (LATDCLR = (1<<0))
#define I_BladeUp_Toggle()            (LATDINV= (1<<0))
#define I_BladeUp_OutputEnable()      (TRISDCLR = (1<<0))
#define I_BladeUp_InputEnable()       (TRISDSET = (1<<0))
#define I_BladeUp_Get()               ((PORTD >> 0) & 0x1)
#define I_BladeUp_PIN                  GPIO_PIN_RD0

/*** Macros for I_Stop pin ***/
#define I_Stop_Set()               (LATDSET = (1<<1))
#define I_Stop_Clear()             (LATDCLR = (1<<1))
#define I_Stop_Toggle()            (LATDINV= (1<<1))
#define I_Stop_OutputEnable()      (TRISDCLR = (1<<1))
#define I_Stop_InputEnable()       (TRISDSET = (1<<1))
#define I_Stop_Get()               ((PORTD >> 1) & 0x1)
#define I_Stop_PIN                  GPIO_PIN_RD1

/*** Macros for I_BladeDwn pin ***/
#define I_BladeDwn_Set()               (LATDSET = (1<<2))
#define I_BladeDwn_Clear()             (LATDCLR = (1<<2))
#define I_BladeDwn_Toggle()            (LATDINV= (1<<2))
#define I_BladeDwn_OutputEnable()      (TRISDCLR = (1<<2))
#define I_BladeDwn_InputEnable()       (TRISDSET = (1<<2))
#define I_BladeDwn_Get()               ((PORTD >> 2) & 0x1)
#define I_BladeDwn_PIN                  GPIO_PIN_RD2

/*** Macros for I_BeltUp pin ***/
#define I_BeltUp_Set()               (LATDSET = (1<<3))
#define I_BeltUp_Clear()             (LATDCLR = (1<<3))
#define I_BeltUp_Toggle()            (LATDINV= (1<<3))
#define I_BeltUp_OutputEnable()      (TRISDCLR = (1<<3))
#define I_BeltUp_InputEnable()       (TRISDSET = (1<<3))
#define I_BeltUp_Get()               ((PORTD >> 3) & 0x1)
#define I_BeltUp_PIN                  GPIO_PIN_RD3

/*** Macros for I_Sharpen pin ***/
#define I_Sharpen_Set()               (LATDSET = (1<<12))
#define I_Sharpen_Clear()             (LATDCLR = (1<<12))
#define I_Sharpen_Toggle()            (LATDINV= (1<<12))
#define I_Sharpen_OutputEnable()      (TRISDCLR = (1<<12))
#define I_Sharpen_InputEnable()       (TRISDSET = (1<<12))
#define I_Sharpen_Get()               ((PORTD >> 12) & 0x1)
#define I_Sharpen_PIN                  GPIO_PIN_RD12

/*** Macros for I_Pos_Lim pin ***/
#define I_Pos_Lim_Get()               ((PORTD >> 4) & 0x1)
#define I_Pos_Lim_PIN                  GPIO_PIN_RD4

/*** Macros for I_Neg_Lim pin ***/
#define I_Neg_Lim_Get()               ((PORTD >> 5) & 0x1)
#define I_Neg_Lim_PIN                  GPIO_PIN_RD5

/*** Macros for I_Estop pin ***/
#define I_Estop_Get()               ((PORTD >> 7) & 0x1)
#define I_Estop_PIN                  GPIO_PIN_RD7

/*** Macros for O_ClampSol pin ***/
#define O_ClampSol_Set()               (LATESET = (1<<0))
#define O_ClampSol_Clear()             (LATECLR = (1<<0))
#define O_ClampSol_Toggle()            (LATEINV= (1<<0))
#define O_ClampSol_OutputEnable()      (TRISECLR = (1<<0))
#define O_ClampSol_InputEnable()       (TRISESET = (1<<0))
#define O_ClampSol_Get()               ((PORTE >> 0) & 0x1)
#define O_ClampSol_PIN                  GPIO_PIN_RE0

/*** Macros for O_AirHockeySol pin ***/
#define O_AirHockeySol_Set()               (LATESET = (1<<1))
#define O_AirHockeySol_Clear()             (LATECLR = (1<<1))
#define O_AirHockeySol_Toggle()            (LATEINV= (1<<1))
#define O_AirHockeySol_OutputEnable()      (TRISECLR = (1<<1))
#define O_AirHockeySol_InputEnable()       (TRISESET = (1<<1))
#define O_AirHockeySol_Get()               ((PORTE >> 1) & 0x1)
#define O_AirHockeySol_PIN                  GPIO_PIN_RE1

/*** Macros for LD3 pin ***/
#define LD3_Set()               (LATGSET = (1<<14))
#define LD3_Clear()             (LATGCLR = (1<<14))
#define LD3_Toggle()            (LATGINV= (1<<14))
#define LD3_OutputEnable()      (TRISGCLR = (1<<14))
#define LD3_InputEnable()       (TRISGSET = (1<<14))
#define LD3_Get()               ((PORTG >> 14) & 0x1)
#define LD3_PIN                  GPIO_PIN_RG14

/*** Macros for LD1 pin ***/
#define LD1_Set()               (LATGSET = (1<<12))
#define LD1_Clear()             (LATGCLR = (1<<12))
#define LD1_Toggle()            (LATGINV= (1<<12))
#define LD1_OutputEnable()      (TRISGCLR = (1<<12))
#define LD1_InputEnable()       (TRISGSET = (1<<12))
#define LD1_Get()               ((PORTG >> 12) & 0x1)
#define LD1_PIN                  GPIO_PIN_RG12

/*** Macros for LD2 pin ***/
#define LD2_Set()               (LATGSET = (1<<13))
#define LD2_Clear()             (LATGCLR = (1<<13))
#define LD2_Toggle()            (LATGINV= (1<<13))
#define LD2_OutputEnable()      (TRISGCLR = (1<<13))
#define LD2_InputEnable()       (TRISGSET = (1<<13))
#define LD2_Get()               ((PORTG >> 13) & 0x1)
#define LD2_PIN                  GPIO_PIN_RG13

/*** Macros for O_Estop_Relay pin ***/
#define O_Estop_Relay_Set()               (LATESET = (1<<2))
#define O_Estop_Relay_Clear()             (LATECLR = (1<<2))
#define O_Estop_Relay_Toggle()            (LATEINV= (1<<2))
#define O_Estop_Relay_OutputEnable()      (TRISECLR = (1<<2))
#define O_Estop_Relay_InputEnable()       (TRISESET = (1<<2))
#define O_Estop_Relay_Get()               ((PORTE >> 2) & 0x1)
#define O_Estop_Relay_PIN                  GPIO_PIN_RE2

/*** Macros for O_StopLed pin ***/
#define O_StopLed_Set()               (LATESET = (1<<3))
#define O_StopLed_Clear()             (LATECLR = (1<<3))
#define O_StopLed_Toggle()            (LATEINV= (1<<3))
#define O_StopLed_OutputEnable()      (TRISECLR = (1<<3))
#define O_StopLed_InputEnable()       (TRISESET = (1<<3))
#define O_StopLed_Get()               ((PORTE >> 3) & 0x1)
#define O_StopLed_PIN                  GPIO_PIN_RE3

/*** Macros for O_AirBlastSol pin ***/
#define O_AirBlastSol_Set()               (LATESET = (1<<4))
#define O_AirBlastSol_Clear()             (LATECLR = (1<<4))
#define O_AirBlastSol_Toggle()            (LATEINV= (1<<4))
#define O_AirBlastSol_OutputEnable()      (TRISECLR = (1<<4))
#define O_AirBlastSol_InputEnable()       (TRISESET = (1<<4))
#define O_AirBlastSol_Get()               ((PORTE >> 4) & 0x1)
#define O_AirBlastSol_PIN                  GPIO_PIN_RE4


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/

typedef enum
{
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_F = 5,
    GPIO_PORT_G = 6,
} GPIO_PORT;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/

typedef enum
{
    GPIO_PIN_RA0 = 0,
    GPIO_PIN_RA1 = 1,
    GPIO_PIN_RA2 = 2,
    GPIO_PIN_RA3 = 3,
    GPIO_PIN_RA4 = 4,
    GPIO_PIN_RA5 = 5,
    GPIO_PIN_RA6 = 6,
    GPIO_PIN_RA7 = 7,
    GPIO_PIN_RA9 = 9,
    GPIO_PIN_RA10 = 10,
    GPIO_PIN_RA14 = 14,
    GPIO_PIN_RA15 = 15,
    GPIO_PIN_RB0 = 16,
    GPIO_PIN_RB1 = 17,
    GPIO_PIN_RB2 = 18,
    GPIO_PIN_RB3 = 19,
    GPIO_PIN_RB4 = 20,
    GPIO_PIN_RB5 = 21,
    GPIO_PIN_RB6 = 22,
    GPIO_PIN_RB7 = 23,
    GPIO_PIN_RB8 = 24,
    GPIO_PIN_RB9 = 25,
    GPIO_PIN_RB10 = 26,
    GPIO_PIN_RB11 = 27,
    GPIO_PIN_RB12 = 28,
    GPIO_PIN_RB13 = 29,
    GPIO_PIN_RB14 = 30,
    GPIO_PIN_RB15 = 31,
    GPIO_PIN_RC1 = 33,
    GPIO_PIN_RC2 = 34,
    GPIO_PIN_RC3 = 35,
    GPIO_PIN_RC4 = 36,
    GPIO_PIN_RC12 = 44,
    GPIO_PIN_RC13 = 45,
    GPIO_PIN_RC14 = 46,
    GPIO_PIN_RC15 = 47,
    GPIO_PIN_RD0 = 48,
    GPIO_PIN_RD1 = 49,
    GPIO_PIN_RD2 = 50,
    GPIO_PIN_RD3 = 51,
    GPIO_PIN_RD4 = 52,
    GPIO_PIN_RD5 = 53,
    GPIO_PIN_RD6 = 54,
    GPIO_PIN_RD7 = 55,
    GPIO_PIN_RD8 = 56,
    GPIO_PIN_RD9 = 57,
    GPIO_PIN_RD10 = 58,
    GPIO_PIN_RD11 = 59,
    GPIO_PIN_RD12 = 60,
    GPIO_PIN_RD13 = 61,
    GPIO_PIN_RD14 = 62,
    GPIO_PIN_RD15 = 63,
    GPIO_PIN_RE0 = 64,
    GPIO_PIN_RE1 = 65,
    GPIO_PIN_RE2 = 66,
    GPIO_PIN_RE3 = 67,
    GPIO_PIN_RE4 = 68,
    GPIO_PIN_RE5 = 69,
    GPIO_PIN_RE6 = 70,
    GPIO_PIN_RE7 = 71,
    GPIO_PIN_RE8 = 72,
    GPIO_PIN_RE9 = 73,
    GPIO_PIN_RF0 = 80,
    GPIO_PIN_RF1 = 81,
    GPIO_PIN_RF2 = 82,
    GPIO_PIN_RF3 = 83,
    GPIO_PIN_RF4 = 84,
    GPIO_PIN_RF5 = 85,
    GPIO_PIN_RF8 = 88,
    GPIO_PIN_RF12 = 92,
    GPIO_PIN_RF13 = 93,
    GPIO_PIN_RG0 = 96,
    GPIO_PIN_RG1 = 97,
    GPIO_PIN_RG2 = 98,
    GPIO_PIN_RG3 = 99,
    GPIO_PIN_RG6 = 102,
    GPIO_PIN_RG7 = 103,
    GPIO_PIN_RG8 = 104,
    GPIO_PIN_RG9 = 105,
    GPIO_PIN_RG12 = 108,
    GPIO_PIN_RG13 = 109,
    GPIO_PIN_RG14 = 110,
    GPIO_PIN_RG15 = 111,

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
    GPIO_PIN_NONE = -1

} GPIO_PIN;

typedef enum
{
  CN0_PIN = 1 << 0,
  CN1_PIN = 1 << 1,
  CN2_PIN = 1 << 2,
  CN3_PIN = 1 << 3,
  CN4_PIN = 1 << 4,
  CN5_PIN = 1 << 5,
  CN6_PIN = 1 << 6,
  CN7_PIN = 1 << 7,
  CN8_PIN = 1 << 8,
  CN9_PIN = 1 << 9,
  CN10_PIN = 1 << 10,
  CN11_PIN = 1 << 11,
  CN12_PIN = 1 << 12,
  CN13_PIN = 1 << 13,
  CN14_PIN = 1 << 14,
  CN15_PIN = 1 << 15,
  CN16_PIN = 1 << 16,
  CN17_PIN = 1 << 17,
  CN18_PIN = 1 << 18,
  CN19_PIN = 1 << 19,
  CN20_PIN = 1 << 20,
  CN21_PIN = 1 << 21,
}CN_PIN;

typedef  void (*GPIO_PIN_CALLBACK) ( CN_PIN cnPin, uintptr_t context);

void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PinInterruptEnable(CN_PIN cnPin);

void GPIO_PinInterruptDisable(CN_PIN cnPin);

// *****************************************************************************
// *****************************************************************************
// Section: Local Data types and Prototypes
// *****************************************************************************
// *****************************************************************************

typedef struct {

    /* CN Pin number */
    CN_PIN                  cnPin;

    /* Corresponding GPIO pin name */
    GPIO_PIN                gpioPin;

    /* previous port pin value, need to be stored to check if it has changed later */
    bool                    prevPinValue;

    /* Callback for event on target pin*/
    GPIO_PIN_CALLBACK       callback;

    /* Callback Context */
    uintptr_t               context;

} GPIO_PIN_CALLBACK_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
    GPIO_PortWrite((GPIO_PORT)(pin>>4), (uint32_t)(0x1) << (pin & 0xF), (uint32_t)(value) << (pin & 0xF));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return (bool)(((GPIO_PortRead((GPIO_PORT)(pin>>4))) >> (pin & 0xF)) & 0x1);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (bool)((GPIO_PortLatchRead((GPIO_PORT)(pin>>4)) >> (pin & 0xF)) & 0x1);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

bool GPIO_PinInterruptCallbackRegister(
    CN_PIN cnPin,
    const   GPIO_PIN_CALLBACK callBack,
    uintptr_t context
);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
