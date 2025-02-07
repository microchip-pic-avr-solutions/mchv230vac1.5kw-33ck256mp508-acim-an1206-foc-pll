// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file id_ref.h
 *
 * @brief This header file lists data type definitions and interface functions 
 * of the flux reference generation module.
 *
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


#ifndef __ID_REF_H
#define __ID_REF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "measure.h"
#include "pi.h"
 
    
typedef struct
{
    int16_t
        IdRef,              /* Id Current reference */
        IdRefFilt,          /* Filtered Id Current reference */
        IdRefFiltConst,     /* Filter constant for Id */
        voltageMag,         /* Voltage vector magnitude */
        voltageMagRef;      /* Voltage vector magnitude reference */
            
    int32_t
        IdRefFiltStateVar;  /* Accumulation variable for IdRef filter */
    
    MCAPP_PISTATE_T FWeakPI;

} MCAPP_FWEAK_VOLTAGE_FB_T;


typedef struct
{
    int16_t
        IdRef,              /* Id Current reference */
        IdRefMin,           /* Lower limit on IdRef */   
        FdWeakStartSpeed;   /* Field weakening start speed */
    
    int32_t 
        FWeakConstant;     
    
} MCAPP_FWEAK_TYPE1_T;
    
    

typedef struct
{
    int16_t 
        qNominalMagCurrent,     /* Nominal magnetization current */
        qMinMagCurrent,         /* Minimum magnetization current */
        qIdRefOffset,
        IdRef;              /* Id Current reference */
    
    MCAPP_FWEAK_TYPE1_T
        fWeakType1;
    
    MCAPP_FWEAK_VOLTAGE_FB_T
        fWeakType2;
    
} IDREFGEN_T;

//extern MCAPP_FWEAK_TYPE1_T fWeakType1;
extern IDREFGEN_T idRefGen;

void IdRefGenerationInit(IDREFGEN_T *);
void IdRefGeneration(IDREFGEN_T *);


void FluxWeakeningInit(void);
int16_t FluxWeakening(void);

int16_t FluxWeakening2(void);
int16_t FluxWeakening3(void);
int16_t FluxWeakening4(void);

#ifdef __cplusplus
}
#endif

#endif /* end of __ID_REF_H */
