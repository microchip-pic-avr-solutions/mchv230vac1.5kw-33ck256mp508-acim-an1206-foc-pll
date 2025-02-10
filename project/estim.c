// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim.c
 *
 * @brief This module implements PLL estimator
 *
 * Component: ESTIMATOR - PLL
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
#include <libq.h>
#include "motor_control_noinline.h"
#include "userparms.h"
#include "estim.h"
#include "control.h"
#include "measure.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Definitions ">
#define INVPSIRD_DEFAULT 32766 /* Maximum value for Inverse of Rotor Flux linkage*/
#define DECIMATE_NOMINAL_SPEED   NORM_VALUE(NOMINAL_SPEED_RPM/10,MC1_PEAK_SPEED_RPM)
// </editor-fold>

/** Variables */
ESTIM_PARM_T estimator;
MOTOR_ESTIM_PARM_T motorParm;
MC_ALPHABETA_T bemfAlphaBeta;
MC_DQ_T bemfdq;
MC_SINCOS_T sincosThetaEstimator;

// *****************************************************************************

/* Function:
    Estim()

  Summary:
    Motor speed and angle estimator

  Description:
    Estimation of the speed of the motor and field angle based on inverter
    voltages and motor currents.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void Estim(void) 
{
    int16_t deltaEs;
    uint16_t index = (estimator.qDiCounter - 7)&0x0007;
 
    estimator.qValpha = valphabeta.alpha;
    estimator.qVbeta  = valphabeta.beta;
    
    /* dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta
       For lower speed the granularity of difference is higher - the
       difference is made between 2 sampled values @ 8 ADC ISR cycles */
    if (_Q15abs(estimator.qVelEstim) < PLL_VELOCITY_FILTER_THRESHOLD) 
    {
        estimator.qDIalpha = (ialphabeta.alpha -
                estimator.qLastIalphaHS[index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR
           cycle .The following limitation assures a limitation per low speed -
           up to the nominal speed */
        if (estimator.qDIalpha > estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = estimator.qDIlimitLS;
        }
        if (estimator.qDIalpha < -estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = -estimator.qDIlimitLS;
        }
        estimator.qVIndalpha = (int16_t) (__builtin_mulss(motorParm.qSigmaLsDt,
                estimator.qDIalpha) >> (NORM_SIGMALSDT_QVALUE+3));

        estimator.qDIbeta = (ialphabeta.beta - estimator.qLastIbetaHS[index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR cycle
           the following limitation assures a limitation per low speed - up to
           the nominal speed */
        if (estimator.qDIbeta > estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = estimator.qDIlimitLS;
        }
        if (estimator.qDIbeta < -estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = -estimator.qDIlimitLS;
        }
        estimator.qVIndbeta = (int16_t) (__builtin_mulss(motorParm.qSigmaLsDt,
                estimator.qDIbeta) >> (NORM_SIGMALSDT_QVALUE+3));
    }
    else
    {
        estimator.qDIalpha = (ialphabeta.alpha - estimator.qLastIalphaHS[(estimator.qDiCounter)]);
        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
           the following limitation assures a limitation per high speed - up to
           the maximum speed */
        if (estimator.qDIalpha > estimator.qDIlimitHS) 
        {
            estimator.qDIalpha = estimator.qDIlimitHS;
        }
        if (estimator.qDIalpha < -estimator.qDIlimitHS) 
        {
            estimator.qDIalpha = -estimator.qDIlimitHS;
        }
        estimator.qVIndalpha = (int16_t) (__builtin_mulss(motorParm.qSigmaLsDt,
                        estimator.qDIalpha) >> NORM_SIGMALSDT_QVALUE);

        estimator.qDIbeta = (ialphabeta.beta - estimator.qLastIbetaHS[(estimator.qDiCounter)]);

        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
           the following limitation assures a limitation per high speed - up to
           the maximum speed */
        if (estimator.qDIbeta > estimator.qDIlimitHS) 
        {
            estimator.qDIbeta = estimator.qDIlimitHS;
        }
        if (estimator.qDIbeta < -estimator.qDIlimitHS) 
        {
            estimator.qDIbeta = -estimator.qDIlimitHS;
        }
        estimator.qVIndbeta = (int16_t) (__builtin_mulss(motorParm.qSigmaLsDt,
                                  estimator.qDIbeta) >> NORM_SIGMALSDT_QVALUE);
    }    
   
    /* Update the sample history of Ialpha and Ibeta */
    estimator.qDiCounter = (estimator.qDiCounter + 1) & 0x0007;
    estimator.qLastIalphaHS[estimator.qDiCounter] = ialphabeta.alpha;
    estimator.qLastIbetaHS[estimator.qDiCounter] = ialphabeta.beta;

   /* Calculate the BEMF voltage:
     *  Ealphabeta = Valphabeta - Rs*Ialphabeta - SigmaLs*(dIalphabeta/dt)
     * and Ealphabeta scale it down by a factor of two. This scaling is required 
     * to prevent overflow/saturation of BEMF calculation in a few corner cases.
     * Use the previous value of Valphabeta here since this value corresponds to
     * the current value of phase current sample */
    bemfAlphaBeta.alpha =  (estimator.qValpha - (int16_t) (__builtin_mulss(motorParm.qRs, 
                                  ialphabeta.alpha) >> NORM_RS_QVALUE) -
                        estimator.qVIndalpha);
    bemfAlphaBeta.beta =   (estimator.qVbeta -
                        (int16_t) (__builtin_mulss(motorParm.qRs,
                                 ialphabeta.beta) >> NORM_RS_QVALUE) -
                        estimator.qVIndbeta);
    
    /* Calculate sine and cosine components of the rotor flux angle */
    MC_CalculateSineCosine_Assembly_Ram((estimator.qRho + estimator.qRhoOffset),
                                        &sincosThetaEstimator);

    /*  Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark_Assembly(&bemfAlphaBeta, &sincosThetaEstimator, &bemfdq);

    
    const int16_t ddiff = (int16_t) (bemfdq.d - estimator.qEsdf);
    estimator.qEsdStateVar += __builtin_mulss(ddiff, estimator.qKfilterEsdq);
    estimator.qEsdf = (int16_t) (estimator.qEsdStateVar >> 15);

    const int16_t qdiff = (int16_t) (bemfdq.q - estimator.qEsqf);
    estimator.qEsqStateVar += __builtin_mulss(qdiff, estimator.qKfilterEsdq);
    estimator.qEsqf = (int16_t) (estimator.qEsqStateVar >> 15);
    
    /* To avoid Math Trap Error*/
    if(ctrlParm.qIdRef > motorParm.qInvLmSqrbyLrBase)
    {    /* Calculating Inverse of Rotor Flux linkage*/
        motorParm.qInvLmSqrbyLr = motorParm.qInvLmSqrbyLrBase;
        estimator.qinvPsiRd = __builtin_divf(motorParm.qInvLmSqrbyLr,ctrlParm.qIdRef);
        estimator.qinvPsiRdShift = 0;
    }
    else if(ctrlParm.qIdRef > (motorParm.qInvLmSqrbyLrBase>>1))
    {    /* Calculating Inverse of Rotor Flux linkage*/
        motorParm.qInvLmSqrbyLr = motorParm.qInvLmSqrbyLrBase>>1;
        estimator.qinvPsiRd = __builtin_divf(motorParm.qInvLmSqrbyLr,ctrlParm.qIdRef);
        estimator.qinvPsiRdShift = 1;
    }
    else if(ctrlParm.qIdRef > (motorParm.qInvLmSqrbyLrBase>>2))
    {    /* Calculating Inverse of Rotor Flux linkage*/
        motorParm.qInvLmSqrbyLr = motorParm.qInvLmSqrbyLrBase>>2;
        estimator.qinvPsiRd = __builtin_divf(motorParm.qInvLmSqrbyLr,ctrlParm.qIdRef);
        estimator.qinvPsiRdShift = 2;
    }
    else{
        estimator.qinvPsiRd = 32000;
        estimator.qinvPsiRdShift = -15;
    }


     /*  For stability the condition for low speed */
    if (_Q15abs(estimator.qVelEstim) > DECIMATE_NOMINAL_SPEED) 
    {
        /* At speed greater than decimation speed, calculate the estimated
         * velocity based on:
         *  OmegaMr = Invpsi * (Eqfiltered - sgn(Eqfiltered) * Edfiltered)
         */
        if (estimator.qEsqf > 0) 
        {
            deltaEs = (int16_t) (estimator.qEsqf - estimator.qEsdf);
            estimator.qOmegaMr =  (int16_t) (__builtin_mulss(estimator.qinvPsiRd,
                                    deltaEs) >> (15-estimator.qinvPsiRdShift) );
        } 
        else 
        {
            deltaEs = (int16_t) (estimator.qEsqf + estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(estimator.qinvPsiRd,
                                    deltaEs) >> (15-estimator.qinvPsiRdShift));
        }
    }        
    /* if estimator speed<10% => condition VelRef<>0 */
    else 
    {
        /* At speed lower than or equal to decimation speed, calculate the estimated
         * velocity based on:
         *  OmegaMr = (1/ke) * (Eqfiltered - sgn(omega) * Edfiltered)
         * to improve stability.
         */
        if (estimator.qVelEstim > 0) 
        {
            deltaEs = (int16_t) (estimator.qEsqf - estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(estimator.qinvPsiRd,
                                    deltaEs) >> (15-estimator.qinvPsiRdShift));
        } 
        else 
        {
            deltaEs = (int16_t) (estimator.qEsqf + estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(estimator.qinvPsiRd,
                                    deltaEs) >> (15-estimator.qinvPsiRdShift));
        }
    }
 
    estimator.qOmegaMr = estimator.qOmegaMr << (15-NORM_INVLMSQRBYLR_QVALUE);
     
    /* Integrate the estimated rotor flux velocity to get estimated rotor angle */  
    estimator.qRhoStateVar += __builtin_mulss(estimator.qOmegaMr, estimator.qDeltaT);
    estimator.qRho = (int16_t) (estimator.qRhoStateVar >> 15);
    
    /* Estimate the Magnetizing Current Imr = (Id/(TrS + 1))*/
    const int16_t imrdiff = (int16_t) (ctrlParm.qIdRef - estimator.qImrEstim);
    estimator.qImrEstimStateVar += __builtin_mulss(imrdiff,
                                                    estimator.qImrEstimFilterK);
    estimator.qImrEstim = (int16_t) (estimator.qImrEstimStateVar >> 15);
    
    /* Estimate the slip value wslip = ((1/Tr) * (iq/imr))*/
    const int32_t iqTr = __builtin_mulss(motorParm.qInvTr,idq.q);

    if((estimator.qImrEstim > 0))
    {    
        estimator.qOmegaSlip =  __builtin_divsd(iqTr,estimator.qImrEstim);
    }
    else{
        estimator.qOmegaSlip = 0;
    }

    /* Estimate the rotor velocity by subtracting slip from synchronous Speed(rotor flux) */
    estimator.qOmegaRotor = estimator.qOmegaMr - estimator.qOmegaSlip ;
            
    /* Filter the estimated  rotor velocity using a first order low-pass filter */
    const int16_t Omegadiff = (int16_t) (estimator.qOmegaRotor - estimator.qVelEstim);
    estimator.qVelEstimStateVar += __builtin_mulss(Omegadiff, estimator.qVelEstimFilterK);
    estimator.qVelEstim = (int16_t) (estimator.qVelEstimStateVar >> 15);
}
// *****************************************************************************

/* Function:
    InitEstimParm ()

  Summary:
    Initializes Motor speed and angle estimator parameters

  Description:
    Initialization of the parameters of the estimator.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitEstimParm(void) 
{
    /* Constants are defined in userparms.h */

    motorParm.qSigmaLsDtBase = NORM_SIGMALSDT;
    motorParm.qSigmaLsDt = motorParm.qSigmaLsDtBase;
    motorParm.qRs = NORM_RS;

    motorParm.qInvLmSqrbyLrBase = NORM_INVLMSQRBYLR;

    motorParm.qInvTr = NORM_INVTR;
    
    estimator.qRhoStateVar = 0;
    estimator.qRho = 0;
    estimator.qOmegaMr = 0;
    estimator.qDiCounter = 0;
    estimator.qEsdStateVar = 0;
    estimator.qEsqStateVar = 0;
    estimator.qOmegaSlip = 0;
    estimator.qImrEstim = 0;
    estimator.qImrEstimStateVar = 0;
    estimator.qOmegaRotor = 0;
    estimator.qinvPsiRd = 0;
    estimator.qDIlimitHS = D_ILIMIT_HS;
    estimator.qDIlimitLS = D_ILIMIT_LS;
    estimator.qVelEstim = 0;
    estimator.qVelEstimStateVar =0;
    
    estimator.qKfilterEsdq = KFILTER_ESDQ;
    estimator.qVelEstimFilterK = KFILTER_VELESTIM;
    estimator.qImrEstimFilterK = KFILTER_IMRESTIM;
    
    estimator.qDeltaT = NORM_DELTAT;
    estimator.qRhoOffset = 0;    
}
