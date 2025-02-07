// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pi.c
 *
 * @brief This module implements Proportional Integral Control (PI).
 *
 * Component: pi
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
#include "xc.h"
#include "motor_control_types.h"

#include "pi.h"
#include "system.h"

// </editor-fold>


inline static int16_t saturatedSubtract(int16_t, int16_t);
inline static void writeAccB32(int32_t);
inline static int32_t readAccA32();

volatile register int a_Reg asm("A");
volatile register int b_Reg asm("B");

void MCAPP_ControllerPIUpdate(int16_t in_Ref, int16_t in_Meas, 
        MCAPP_PISTATE_T *state,  int16_t *out)
{
    int16_t error;
    /* non saturated output */
    int16_t out_nonsat;
    /* saturated output */
    int16_t out_sat;
    uint16_t saveCorcon = HAL_CORCON_RegisterValue_Get();
    
    /* Init CORCON register */
    HAL_CORCON_Initialize();

    /* Calculate error */
    error = saturatedSubtract(in_Ref, in_Meas); 

    /* Read state->integrator into B */
    writeAccB32(state->integrator);

    /* Calculate (Kp * error * 2^Nkp), store in A and out_Buffer */
    a_Reg = __builtin_mpy(error, state->kp, 0, 0, 0, 0, 0, 0);
    a_Reg = __builtin_sftac(a_Reg, -state->nkp);
    a_Reg = __builtin_addab(a_Reg, b_Reg);
    out_nonsat = __builtin_sacr(a_Reg, 0);

    /* Limit the output */
    out_sat = UTIL_LimitS16(out_nonsat, state->outMin, state->outMax);
    
    *out = out_sat;
    
    
    /* Calculate integrator term and add it to previous value */
  
    /* Calculate (error * Ki) and store in A */
    a_Reg = __builtin_mpy(error, state->ki, 0, 0, 0, 0, 0, 0);
    a_Reg = __builtin_sftac(a_Reg, -state->nki);

    /* Calculate (excess * Kc), subtract from (error * Ki) and store in A */
    error = out_nonsat - out_sat;
    a_Reg = __builtin_msc(a_Reg, error, state->kc,0,0,0,0,0,0,0,0); //Returns the value of accumulator minus the result of a x b.

    /* Add (error * Ki)-(excess * Kc) to the integrator value in B */
    a_Reg = __builtin_addab(a_Reg,b_Reg);

    state->integrator = readAccA32();


    HAL_CORCON_RegisterValue_Set(saveCorcon);
    
}

void MCAPP_ControllerPIReset(MCAPP_PISTATE_T *state, int16_t value)
{
    state->integrator = (((int32_t)value)<<16);
}

void MCAPP_ControllerPIInit(MCAPP_PISTATE_T *state)
{
    state->integrator = 0 ;
}

/** * subtracts two 16-bit numbers but saturates the results * (requires saturation mode to be set) */
inline static int16_t saturatedSubtract(int16_t x1, int16_t x2)
{
    a_Reg = __builtin_lac(x1, 0);
    b_Reg = __builtin_lac(x2, 0);
    a_Reg = __builtin_subab(a_Reg, b_Reg);
    return __builtin_sacr(a_Reg, 0);
}
/** * Write accumulator B */
inline static void writeAccB32(int32_t input)
{
#if __XC16_VERSION__ >= 1026
    const int32_t tmp = input;
    asm volatile ("" :: "r"(tmp)); 
    b_Reg = __builtin_lacd(tmp, 0);
#else
    uint32_t temp_dword;
    uint16_t temp_word;
    temp_dword = 0xFFFF0000 & input;
    temp_dword = temp_dword >> 16;
    temp_word = (uint16_t)temp_dword;
    b_Reg = __builtin_lac(temp_word, 0);
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("" : "+w"(b_Reg):); 
    temp_word = (uint16_t)(0xFFFF & input);
    ACCBL = temp_word;
#endif
}
/** * Read accumulator A */
inline static int32_t readAccA32()
{
#if __XC16_VERSION__ >= 1026
    const int32_t tmp = __builtin_sacd(a_Reg, 0);
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("");
    return tmp;
#else
    int32_t result;
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("" : "+w"(a_Reg):); 
    result = ACCAH;
    result <<= 16;
    result |= ACCAL; return result;
#endif
}


