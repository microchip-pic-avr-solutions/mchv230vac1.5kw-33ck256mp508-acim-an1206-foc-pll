// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * MAIN Generated Driver Header File
 * 
 * @file      system.h
 * 
 * @defgroup  systemdriver System Driver
 * 
 * @brief     System driver using dsPIC MCUs.
 *
 * @version   Driver Version 1.0.1
 *
 * @skipline  Device : dsPIC33CK256MP508
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

#include "xc.h"
#include "stdint.h"


#ifndef SYSTEM_H
#define	SYSTEM_H


 #define CORCON_MODE_ENABLEALLSATNORMAL_ROUNDBIASED 0x00E2    /**< Enable saturation for ACCA, ACCB
                                                             and Dataspace write, enable normal
                                                             ACCA/ACCB saturation mode and set
                                                             rounding to Biased (conventional)
                                                             mode. Rest of CORCON settings are
                                                             set to the default POR values.
                                                             */

/**
 * Gets the CORCON register value
 * @brief   This inline function gets the value of CPU core control register
 * @return CORCON register value
 */
inline static uint16_t HAL_CORCON_RegisterValue_Get(void) 
{    
    return CORCON;
}

/**
 * Sets the value of the CORCON register
 * @brief      This inline function sets the value of CPU core control register
 * @param reg_value register value to be set for the CORCON register
 */
inline static void HAL_CORCON_RegisterValue_Set(uint16_t reg_value) 
{
    CORCON = reg_value;
}

/**
* Initializes the CORCON module
 * @brief     This inline function sets the CPU core control register operating mode 
 *            to a value that is decided by the SYSTEM_CORCON_MODES argument.
*/
inline static void HAL_CORCON_Initialize(void) 
{
    CORCON = (CORCON & 0x00F2) | CORCON_MODE_ENABLEALLSATNORMAL_ROUNDBIASED;
}



#endif	/* SYSTEM_H */
/**
 End of File
*/