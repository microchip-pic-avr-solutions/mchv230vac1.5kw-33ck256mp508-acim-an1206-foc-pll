// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file port_config.c
 *
 * @brief This module initializes the GPIO pins as analog/digital,input or 
 * output etc. 
 * 
 * Definitions in the file are for dsPIC33CK256MP508 MC DIM plugged onto 
 * Motor Control Development board from Microchip
 *
 * Component: PORTS
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include "port_config.h"
#include "userparms.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/* Function:
    SetupGPIOPorts()

  Summary:
    Routine to set-up GPIO ports

  Description:
    Function initializes GPIO pins for input or output ports,analog/digital pins,
    remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void SetupGPIOPorts(void)
{
    // Reset all PORTx register (all inputs)
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    MapGPIOHWFunction();

    return;
}
// *****************************************************************************
/* Function:
    Map_GPIO_HW_Function()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initializes GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFunction(void)
{
    
    /* ANALOG SIGNALS */

    // Configure Port pins for Motor Current Sensing
    
#ifdef INTERNAL_OPAMP_CONFIG
    // Ib Out
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 0;   // Pin 23: OA3OUT/AN4/CMP3B/IBIAS3/RA4
    
    //Ib- 
    ANSELCbits.ANSELC1 = 1;
    TRISCbits.TRISC1 = 1;   //Pin 28: OA3IN-/AN13/CMP1B/ISRC0/RP49/PMA7/RC1
    
    //Ib+ 
    ANSELCbits.ANSELC2 = 1;
    TRISCbits.TRISC2 = 1;   //Pin 29: OA3IN+/AN14/CMP2B/ISRC1/RP50/PMD13/PMA13/RC2
    
    //Ia Out
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 0;   //Pin 41: OA2OUT/AN1/AN7/ANA0/CMP1D/CMP2D/CMP3D/RP34/SCL3/INT0/RB2
    
    //Ia- 
    ANSELBbits.ANSELB3 = 1;
    TRISBbits.TRISB3 = 1;   //Pin 43: PGD2/OA2IN-/AN8/RP35/RB3
    
    //Ia+ 
    ANSELBbits.ANSELB4 = 1;
    TRISBbits.TRISB4 = 1;   //Pin 45: PGC2/OA2IN+/RP36/RB4
    
    //Op-Amp Configuration
    AMPCON1Hbits.NCHDIS2 = 0;    //Wide input range for Op Amp #2
    AMPCON1Lbits.AMPEN2 = 1;     //Enables Op Amp #2
  
    AMPCON1Hbits.NCHDIS3 = 0;    //Wide input range for Op Amp #3
    AMPCON1Lbits.AMPEN3 = 1;     //Enables Op Amp #3
    
    AMPCON1Lbits.AMPON = 1;      //Enables op amp modules if their respective AMPENx bits are also asserted
    
#else
    //Amplified Ib 
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;   // Pin 23: OA3OUT/AN4/CMP3B/IBIAS3/RA4
    
    //Amplified Ia 
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;   //Pin 41: OA2OUT/AN1/AN7/ANA0/CMP1D/CMP2D/CMP3D/RP34/SCL3/INT0/RB2
    
#endif
    
    // Potentiometer input - used as Speed Reference
    // POT : DIM #28
    // PIN : AN17/ANN1/IBIAS1/RP54/PMD12/PMA12/RC6
    ANSELCbits.ANSELC6 = 1;
    TRISCbits.TRISC6 = 1;   
 
    /*DC Bus Voltage Signals*/
    // DIM #39, PIN : AN15/CMP2A/IBIAS2/RP51/PMD11/PMA11/RC3
    ANSELCbits.ANSELC3 = 1;
    TRISCbits.TRISC3 = 1; 
    
    /* Digital SIGNALS */   
    // DIGITAL INPUT/OUTPUT PINS
    // Inverter Control - PWM Outputs
    // PWM1L : DIM #03  RP47/PWM1L/RB15
    // PWM1H : DIM #01  RP46/PWM1H/RB14
    // PWM2L : DIM #07  RP45/PWM2L/RB13
    // PWM2H : DIM #05  TDI/RP44/PWM2H/RB12
    // PWM3L : DIM #04 TCK/RP43/PWM3L/RB11
    // PWM3H : DIM #02  TMS/RP42/PWM3H/RB10
    TRISBbits.TRISB14 = 0 ;          
    TRISBbits.TRISB15 = 0 ;         
    TRISBbits.TRISB12 = 0 ;          
    TRISBbits.TRISB13 = 0 ;           
    TRISBbits.TRISB10 = 0 ;          
    TRISBbits.TRISB11 = 0 ;    
    
            
    // FAULT Pins
    // FAULT : DIM #40
    TRISDbits.TRISD12 = 1;           // RP76/RD12 via R37
    _PCI9R = 76;                    //PCI fault assigned to PCI9 
    
    // Debug LEDs
    // LED2 : 
    TRISEbits.TRISE13 = 0;           // PIN:64 - RE13
    // LED1 : 
    TRISEbits.TRISE12 = 0;           // PIN:62 - RE12

    // Push button Switches
    
    // SW1 : 
    TRISEbits.TRISE10 = 1;           // PIN:57 - RE10
    // SW2 : 
    TRISEbits.TRISE11 = 1;           // PIN:59 - RE11
	/** Diagnostic Interface for MCHV-230VAC-1.5kW Development Board.
        Re-map UART Channels to the device pins connected to the following 
        PIM pins on the Motor Control Development Boards .
        UART_RX : PIN #14 - ANN2/RP77/RD13 (Input)
        UART_TX : PIN #13 - RP78/PCI21/RD14(Output)   */
    _U1RXR = 77;
    _RP78R = 0b000001;
        
}

// </editor-fold>
