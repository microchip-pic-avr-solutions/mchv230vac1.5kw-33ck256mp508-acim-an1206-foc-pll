// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file id_ref.c
 *
 * @brief This module implements field weakening algorithms to 
 * generate id current reference required for extended speed operation of 
 * Induction Motors.
 *
 * Component: ID REFERENCE GENERATION
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

#include <stdint.h>

/* _Q15abs and _Q15sqrt function use */
#include <libq.h>
#include "motor_control_noinline.h"
#include "id_ref.h"
#include "userparms.h"
#include "control.h"

#include "estim.h"

#include "pi.h"

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">
#undef ID_REFERNCE_FILTER_ENABLE /* Id reference filter enable flag */
#define INVLORATIO_MAX 32766
#define INVLORATIO_MIN 16383
#define Q14_1   16382
// </editor-fold>

IDREFGEN_T idRefGen;

static void FluxControlVoltFeedback(MCAPP_FWEAK_VOLTAGE_FB_T *);
static void FluxControlInvSpeed(MCAPP_FWEAK_TYPE1_T *);
static void InitFluxControlVoltFeedback(MCAPP_FWEAK_VOLTAGE_FB_T *);


/**
* <B> Function: IdRefGenerationInit(IDREFGEN_T * )  </B>
*
* @brief Function to reset variables used for Field Weakening Control.
*        .
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> IdRefGenerationInit(&IdRefGen); </CODE>
*
*/
void IdRefGenerationInit(IDREFGEN_T *pIdRefGen)
{
  pIdRefGen->qNominalMagCurrent = NORM_NOMINAL_IDREF;
  pIdRefGen->qMinMagCurrent = NORM_MINIMUM_IDREF;
  pIdRefGen->qIdRefOffset = 0;
  pIdRefGen->fWeakType1.FdWeakStartSpeed = FD_WEAK_START_SPEED;
          
  InitFluxControlVoltFeedback(&pIdRefGen->fWeakType2);
}

/**
* <B> Function: IdRefGeneration(IDREFGEN_T * )  </B>
*
* @brief Function implementing Field Weakening Control
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> IdRefGeneration(&idRefGen); </CODE>
*
*/
void IdRefGeneration(IDREFGEN_T *pIdRefGen)
{   
    
    FluxControlInvSpeed(&pIdRefGen->fWeakType1); 
    FluxControlVoltFeedback(&pIdRefGen->fWeakType2);

#ifdef FD_WEAK_TYPE_VOLTAGE_FB
    pIdRefGen->IdRef = pIdRefGen->fWeakType2.IdRef - idRefGen.qIdRefOffset;
#else
    pIdRefGen->IdRef = pIdRefGen->fWeakType1.IdRef - idRefGen.qIdRefOffset;
#endif
    
    if(idRefGen.qIdRefOffset > 0){
        idRefGen.qIdRefOffset--;
    }
    else if(idRefGen.qIdRefOffset < 0){
        idRefGen.qIdRefOffset++;
    }
}



static void InitFluxControlVoltFeedback(MCAPP_FWEAK_VOLTAGE_FB_T *pFdWeak)
{
    pFdWeak->FWeakPI.kp = FD_WEAK_PI_KP;
    pFdWeak->FWeakPI.ki = FD_WEAK_PI_KI; 
    pFdWeak->FWeakPI.kc = Q15(0.999);
    pFdWeak->FWeakPI.nkp = FD_WEAK_PI_KPSCALE;
    pFdWeak->FWeakPI.nki = 0;
    pFdWeak->FWeakPI.outMax = NORM_NOMINAL_IDREF;
    pFdWeak->FWeakPI.outMin = NORM_MINIMUM_IDREF;
    MCAPP_ControllerPIReset(&pFdWeak->FWeakPI, NORM_NOMINAL_IDREF);
    
    pFdWeak->IdRefFiltConst = FD_WEAK_IDREF_FILT_CONST;
    pFdWeak->voltageMagRef = FD_WEAK_VOLTAGE_REF;
}



/**
* <B> Function: FluxControlInvSpeed(MCAPP_FWEAK_TYPE1_T * )  </B>
*
* @brief Function implementing Field Weakening Control with 
* conventional inverse proportion to speed method 
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> FluxControlInvSpeed(&fieldWeak); </CODE>
*
*/
static void FluxControlInvSpeed(MCAPP_FWEAK_TYPE1_T *pFdWeak)
{
    int16_t ImagRef ; 
    if( ctrlParm.qVelRef > (Q15_NOMINALSPEED>>1) )
    {
        /* imr_ref = Imr_ref*wm_rated/wm_ref; */
        pFdWeak->FWeakConstant = __builtin_mulss(idRefGen.qNominalMagCurrent, pFdWeak->FdWeakStartSpeed);
        ImagRef = (int16_t)(__builtin_divsd(pFdWeak->FWeakConstant, ctrlParm.qVelRef));
        
        pFdWeak->IdRef = ImagRef;
        if(ImagRef < idRefGen.qMinMagCurrent)
        {
            pFdWeak->IdRef = idRefGen.qMinMagCurrent;
        }
        else if(ImagRef > idRefGen.qNominalMagCurrent)
        {
            pFdWeak->IdRef = idRefGen.qNominalMagCurrent;
        }
    }
    else
    {
        pFdWeak->IdRef = idRefGen.qNominalMagCurrent ;
    }

}



static void FluxControlVoltFeedback(MCAPP_FWEAK_VOLTAGE_FB_T *pFdWeak)
{    

    int16_t vdSqr, vqSqr, IdRefOut; 

    /* Compute voltage vector magnitude */
    vdSqr  = (int16_t)(__builtin_mulss(vdq.d, vdq.d) >> 15);
    vqSqr  = (int16_t)(__builtin_mulss(vdq.q, vdq.q) >> 15);
    pFdWeak->voltageMag = _Q15sqrt(vdSqr+vqSqr);
    
    if((ctrlParm.qVelRef > (Q15_NOMINALSPEED>>1)))
    { 
        /* Compute PI output: pFdWeak->IdRef */
        MCAPP_ControllerPIUpdate(pFdWeak->voltageMagRef, pFdWeak->voltageMag, 
            &pFdWeak->FWeakPI, &IdRefOut ); 
    }
    else
    {
        IdRefOut = idRefGen.qNominalMagCurrent ;
        
        /* Reset PI integrator to Nominal Idrefernce value for smooth transition
         * when switching to PI output computation. */
        MCAPP_ControllerPIReset(&pFdWeak->FWeakPI, idRefGen.qNominalMagCurrent);
    }

    /*Filter for the FW Idref current*/
#ifdef ID_REFERNCE_FILTER_ENABLE
    pFdWeak->IdRefFiltStateVar +=
                    __builtin_mulss((IdRefOut - pFdWeak->IdRefFilt),
                                        pFdWeak->IdRefFiltConst);
    pFdWeak->IdRefFilt = (int16_t)(pFdWeak->IdRefFiltStateVar >> 15);
    pFdWeak->IdRef = pFdWeak->IdRefFilt;
#else
    pFdWeak->IdRef = IdRefOut;
#endif
    
}
