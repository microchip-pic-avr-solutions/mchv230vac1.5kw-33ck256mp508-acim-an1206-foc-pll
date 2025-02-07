// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
* @file acim_foc.c
*
* @brief This is the main entry to the application.
*
* Component: APPLICATION
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
#include <stdint.h>
#include <stdbool.h>

#include <libq.h>      
#include "motor_control_noinline.h"
#include "diagnostics.h"

#include "clock.h"
#include "pwm.h"
#include "adc.h"
#include "port_config.h"
#include "delay.h"
#include "board_service.h"
#include "hal/measure.h"

#include "general.h"   
#include "userparms.h"
#include "control.h"   
#include "estim.h"
#include "id_ref.h"
#include "PI.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc=" Global Variables ">
UGF_T       uGF;
CTRL_PARM_T ctrlParm;

MC_ALPHABETA_T  valphabeta, ialphabeta;
MC_SINCOS_T     sincosTheta ;
MC_DQ_T         vdq, idq ;
MC_ABC_T        vabc, vabc_DClinkComp, vabcScaled ;
MC_ABC_T        iabc ;
MC_DUTYCYCLEOUT_T pwmDutycycle ;

volatile int16_t thetaElectrical = 0, thetaElectricalOpenLoop = 0;
uint16_t pwmPeriod;

int16_t vqRefOpenLoop; 
int16_t SpeedControlCount=0;
int16_t openloopflag ;
        
MC_PIPARMIN_T piInputIq;
MC_PIPARMOUT_T piOutputIq;
MC_PIPARMIN_T piInputId;
MC_PIPARMOUT_T piOutputId;
MC_PIPARMIN_T piInputOmega;
MC_PIPARMOUT_T piOutputOmega;
MCAPP_PISTATE_T D_Axis_PISTATE , Q_Axis_PISTATE, Speed_PISTATE ;

volatile uint16_t adcDataBuffer;
MCAPP_MEASURE_T measureInputs;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc=" Function Declarations ">
void InitControlParameters(void);
void DoControl( void );
void CalculateParkAngle(void);
void ResetParmeters(void);
void pwmDutyCycleSet(MC_DUTYCYCLEOUT_T *);
void pwmDutyCycleLimit(MC_DUTYCYCLEOUT_T *,uint16_t,uint16_t);
void DCLinkVoltageCompensation(MC_ABC_T *, MC_ABC_T *, int16_t);
void VoltageBaseChange(MC_ABC_T *, MC_ABC_T *);
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Definitions ">
#define POT_NOM_FACTOR  22050 /* Normalizing factor for potentiometer */
// </editor-fold>

// *****************************************************************************
/* Function:
   main()

  Summary:
    main() function

  Description:
    program entry point, calls the system initialization function group 
    containing the buttons polling loop

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

int main ( void )
{
    InitOscillator();
    SetupGPIOPorts();

    /* Turn on LED2 to indicate the device is programmed */
    LED1 = 1;
    /* Initialize Peripherals */
    InitPeripherals();
    DiagnosticsInit();
    BoardServiceInit();
    CORCONbits.SATA = 0;   

    while(1)
    {        
        /* Initialize PI control parameters */
        InitControlParameters();        
        /* Initialize estimator parameters */
        InitEstimParm();
        /* Initialize measurement parameters */
        MCAPP_MeasureCurrentInit(&measureInputs);
        /* Reset parameters used for running motor through Inverter A*/
        ResetParmeters();
      
        while(1)
        {
            DiagnosticsStepMain();
            BoardService();
            
            if(IsPressed_Button1())
            {
                if(uGF.bits.RunMotor == 1)
                {
                    ResetParmeters();
                }
                else
                {
                    EnablePWMOutputsInverterA();
                    uGF.bits.RunMotor = 1;
                }
            }
            /* LED1 is used as motor run Status */
            LED2 = uGF.bits.RunMotor;
        }

    } // End of Main loop
    // should never get here
    while(1){}
}
// *****************************************************************************
/* Function:
    ResetParmsA()

  Summary:
    This routine resets all the parameters required for Motor through Inv-A

  Description:
    Reinitializes the duty cycle,resets all the counters when restarting motor

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ResetParmeters(void)
{
    /* Make sure ADC does not generate interrupt while initializing parameters*/
	DisableADCInterrupt();
   
    INVERTERA_PWM_TRIGA = ADC_SAMPLING_POINT;
    
    /* Re initialize the duty cycle to minimum value */
    INVERTERA_PWM_PDC3 = MIN_DUTY;
    INVERTERA_PWM_PDC2 = MIN_DUTY;
    INVERTERA_PWM_PDC1 = MIN_DUTY;
    DisablePWMOutputsInverterA();
    
    /* Stop the motor   */
    uGF.bits.RunMotor = 0;        
    /* Set the reference speed value to 0 */
    ctrlParm.qVelRef = 0;
    /* Set the closed loop startup status to 0 */
    ctrlParm.startupStatus = 0;
    /* Restart in open loop */
    uGF.bits.OpenLoop = 1;
    /* Change mode */
    uGF.bits.ChangeMode = 1;
    
    /* Initialize PI control parameters */
    InitControlParameters();  
    
    /* Initialize estimator parameters */
    InitEstimParm();
    
    /* Initialize FieldWeakening parameters */
    IdRefGenerationInit(&idRefGen);
    
    /* Initialize measurement parameters */
    MCAPP_MeasureCurrentInit(&measureInputs);
    
    /* Initialize Open-loop theta */
    thetaElectricalOpenLoop = 0;
    
    /* Enable ADC interrupt and begin main loop timing */
    ClearADCIF();
    adcDataBuffer = ClearADCIF_ReadADCBUF();
    EnableADCInterrupt();
}
// *****************************************************************************
/* Function:
    DoControl()

  Summary:
    Executes one PI iteration for each of the three loops Id,Iq,Speed

  Description:
    This routine executes one PI iteration for each of the three loops
    Id,Iq,Speed

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void DoControl( void )
{
    /* Temporary variables  */
    int16_t vdSquared, vqSquaredLimit;
    
    /* Potentiometer value is scaled between Minimum Speed 
     *  and Maximum Speed to set the speed reference*/
    int16_t potValueNormalized   = UTIL_SatShrS16(
                     __builtin_mulss(measureInputs.potValue,POT_NOM_FACTOR),14); 
    
    ctrlParm.qtargetSpeed = (__builtin_mulss(potValueNormalized, 
                    Q15_MAXIMUMSPEED-Q15_MINIMUMSPEED)>>15) + Q15_MINIMUMSPEED;
            
    if(ctrlParm.speedRampCount >= SPEED_RAMP_SKIP_COUNT)
    { 
        /* Ramp generator to limit the change of the speed reference
          the rate of change is defined by CtrlParm.qRefRamp */
        ctrlParm.qDiff = ctrlParm.qVelRef - ctrlParm.qtargetSpeed;
        /* Speed Ref Ramp */
        if (ctrlParm.qDiff < 0)
        {
            /* Set this cycle reference as the sum of
            previously calculated one plus the reference ramp value */
            ctrlParm.qVelRef = ctrlParm.qVelRef + ctrlParm.qRefRamp;
        }
        else
        {
            /* Same as above for speed decrease */
            ctrlParm.qVelRef = ctrlParm.qVelRef - ctrlParm.qRefRamp;
        }
        /* If difference less than half of ref ramp, set reference
        directly from the pot */
        if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRamp << 1))
        {
            ctrlParm.qVelRef = ctrlParm.qtargetSpeed;
        }
        ctrlParm.speedRampCount = 0;
    }
    ctrlParm.speedRampCount++;

    
    if(uGF.bits.OpenLoop)
    {
        /* OPENLOOP:  force rotating angle,Vd and Vq */
        if(uGF.bits.ChangeMode)
        { /* Just changed to open loop */
            uGF.bits.ChangeMode = 0;
            /* Initialize VqRef & VdRef used in estimator */
            ctrlParm.qIqRef = 0;
            ctrlParm.qIdRef = NORM_NOMINAL_IDREF;
            /* Reinitialize variables for initial speed ramp */
            ctrlParm.speedRampCount = 0;
        }

        /* Multiplying the V/F constant with Speed ref to get required voltage to apply */
        vqRefOpenLoop = (int16_t) (__builtin_mulss(VBYF_CONSTANT,
                                      ctrlParm.qVelRef) >> VBYF_CONSTANT_SHIFT);

        /* Limit the vqRefOpenLoop to Open Loop Maximum Modulation index */
        if(vqRefOpenLoop >= VMAX_VBYF_CONTROL)
        {
           vqRefOpenLoop =  VMAX_VBYF_CONTROL;
        } 

        vdq.q  = vqRefOpenLoop;
        vdq.d = 0;
    }

    else /* Closed Loop Vector Control */
    {
        if( uGF.bits.ChangeMode )
        {
            /* Just changed from open loop */
            uGF.bits.ChangeMode = 0;
            /* Initializing or updating Speed controller integrator*/
            MCAPP_ControllerPIReset(&Speed_PISTATE, idq.q);
            /* Initializing or updating the D,Q Current Control Integrator outputs*/
            MCAPP_ControllerPIReset(&D_Axis_PISTATE, vdq.d);
            MCAPP_ControllerPIReset(&Q_Axis_PISTATE, vdq.q);
            /* Reseting commutation angle state variable for estimator*/
            estimator.qRhoStateVar = ctrlParm.startupRamp;
            /* Id reference offset value */
            idRefGen.qIdRefOffset = ctrlParm.qIdRef - idq.d;
        }

        /* If TORQUE MODE skip the speed controller */
        #ifndef	TORQUE_MODE
            /* Execute the speed control loop every Ts/SPEEDCNTR_SAMPLING_FACTOR */
            SpeedControlCount++;
            if(SpeedControlCount>=SPEEDCNTR_SAMPLING_FACTOR)    
            {
                piInputOmega.inMeasure = estimator.qVelEstim;
                piInputOmega.inReference =ctrlParm.qVelRef;

                MCAPP_ControllerPIUpdate(piInputOmega.inReference, 
                    piInputOmega.inMeasure, &Speed_PISTATE, &piOutputOmega.out);

                ctrlParm.qIqRef = piOutputOmega.out;
                SpeedControlCount = 0;
            }   
        #else
            /* Potentiometer value is scaled to Maximum Torque Component Current
             *  to set the Current reference*/
            ctrlParm.qIqRef =(__builtin_mulss(potValueNormalized,
                                Q15_TORQUE_COMPONENT_CURRENT_PEAK)>>15); 
        #endif
        
        /* Calculate flux reference value or perform field weakening control */       
        IdRefGeneration(&idRefGen);
        #ifndef FDWEAK_CONTROL_ENABLE
                ctrlParm.qIdRef = NORM_NOMINAL_IDREF;
        #else
                ctrlParm.qIdRef = idRefGen.IdRef;
        #endif

        /* D axis current controller */
        piInputId.inMeasure     = idq.d;
        piInputId.inReference   = ctrlParm.qIdRef;   
        MCAPP_ControllerPIUpdate(piInputId.inReference, piInputId.inMeasure, 
        &D_Axis_PISTATE, &piOutputId.out);
        
        vdq.d                   = piOutputId.out;

        /* Dynamic d-q adjustment with d component priority vq=sqrt (vs^2 - vd^2) 
         limit vq maximum to the one resulting from the calculation above */
        vdSquared      = (int16_t)(__builtin_mulss(piOutputId.out, 
                                                         piOutputId.out) >> 15);
        vqSquaredLimit = MAX_VOLTAGE_SQUARE - vdSquared;  
        Q_Axis_PISTATE.outMax =  _Q15sqrt(vqSquaredLimit);
        Q_Axis_PISTATE.outMin =  -Q_Axis_PISTATE.outMax;
        
        /* Q axis current controller */
        piInputIq.inMeasure    = idq.q;
        piInputIq.inReference  = ctrlParm.qIqRef;
        MCAPP_ControllerPIUpdate(piInputIq.inReference, piInputIq.inMeasure, 
        &Q_Axis_PISTATE, &piOutputIq.out );  
        
        vdq.q = piOutputIq.out; 
    }      
}

// *****************************************************************************
/* Function:
   _ADCAN19Interrupt()

  Summary:
   _ADCAN19Interrupt() ISR routine

  Description:
    Does speed calculation and executes the vector update loop
    The ADC sample and conversion is triggered by the PWM period.
    The speed calculation assumes a fixed time interval between calculations.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt()
{
    /* Read ADC Buffers */
    measureInputs.current.Ia    = ADCBUF_INV_A_IPHASE1;
    measureInputs.current.Ib    = ADCBUF_INV_A_IPHASE2;
    measureInputs.dcBusVoltage  = ADCBUF_VBUS_A;
    measureInputs.potValue      = ADCBUF_SPEED_REF_A;

    if( uGF.bits.RunMotor )
    {
        /* Calculate Ia, Ib */
        MCAPP_MeasureCurrentCalibrate(&measureInputs);
        iabc.a = measureInputs.current.Ia;
        iabc.b = measureInputs.current.Ib;
        
        /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
        MC_TransformClarke_Assembly(&iabc,&ialphabeta);
        MC_TransformPark_Assembly(&ialphabeta,&sincosTheta,&idq);
                
        /* Speed and field angle estimation */
        Estim();
        /* Calculate control values */
        DoControl();
        /* if open loop */
        if(uGF.bits.OpenLoop == 1)
        {/* the angle is given by park parameter */
            thetaElectrical = thetaElectricalOpenLoop ;
        }
        else
        {/* if closed loop, angle generated by estimator */
            thetaElectrical = estimator.qRho ;
        }
        /* Calculate openloop qTheta reference for VbyF control */
        CalculateParkAngle();
        
        /* Calculate sin and cos of theta (angle) */
        MC_CalculateSineCosine_Assembly_Ram(thetaElectrical ,&sincosTheta);

        /* Perform inverse Clarke and Park transforms and generate phase voltages.*/
        MC_TransformParkInverse_Assembly(&vdq, &sincosTheta, &valphabeta);      
        MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta, &vabc);
        
        /* DC Link voltage compensation */
        DCLinkVoltageCompensation(&vabc, &vabc_DClinkComp, measureInputs.dcBusVoltage); 
        
        /* Scaling vabc to the base of MC_CalculateSpaceVectorPhaseShifted_Assembly */
        VoltageBaseChange(&vabc_DClinkComp, &vabcScaled);
        MC_CalculateSpaceVectorPhaseShifted_Assembly(&vabcScaled, pwmPeriod, &pwmDutycycle);
        
        pwmDutyCycleLimit(&pwmDutycycle,MIN_DUTY,MAX_DUTY);
        pwmDutyCycleSet(&pwmDutycycle);
    }
    else
    {
        INVERTERA_PWM_TRIGA = ADC_SAMPLING_POINT;
        pwmDutycycle.dutycycle3 = MIN_DUTY;
        pwmDutycycle.dutycycle2 = MIN_DUTY;
        pwmDutycycle.dutycycle1 = MIN_DUTY;
        pwmDutyCycleLimit(&pwmDutycycle,MIN_DUTY,MAX_DUTY);
        pwmDutyCycleSet(&pwmDutycycle);
        if (MCAPP_MeasureCurrentOffsetStatus(&measureInputs) == 0)
        {
            MCAPP_MeasureCurrentOffset(&measureInputs);
        }
    }
  
    DiagnosticsStepIsr();
    BoardServiceStepIsr();
    /* Read ADC Buffer to Clear Flag */
	adcDataBuffer = ClearADCIF_ReadADCBUF();
    ClearADCIF();   
}
// *****************************************************************************
/* Function:
    CalculateParkAngle ()

  Summary:
    Function calculates the angle for open loop control

  Description:
    Generate the start sine waves feeding the motor terminals
    Open loop control, forcing the motor to align and to start speeding up .
 
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void CalculateParkAngle(void)
{
    /* if open loop */
    if (uGF.bits.OpenLoop)
    {
       if( estimator.qVelEstim >= Q15_ENDSPEED )
       {
          #ifndef OPEN_LOOP_FUNCTIONING
              uGF.bits.ChangeMode = 1;
              uGF.bits.OpenLoop = 0;
              estimator.qRhoOffset = thetaElectricalOpenLoop - estimator.qRho;
              ctrlParm.startupStatus = 1;
          #endif
       }
        /* the integral of the angle is the estimated angle */
        ctrlParm.startupRamp    += __builtin_mulss(ctrlParm.qVelRef,NORM_DELTAT);
        thetaElectricalOpenLoop = (int16_t) (ctrlParm.startupRamp >> 15);
    }
    /* Switched to closed loop */
    else 
    {
        /* In closed loop slowly decrease the offset add to the estimated angle */
        if(estimator.qRhoOffset > 0)
        {
            estimator.qRhoOffset--;
        }
        else if(estimator.qRhoOffset < 0)
        {
            estimator.qRhoOffset++;
        }
    }
}
// *****************************************************************************
/* Function:
    InitControlParameters()

  Summary:
    Function initializes control parameters

  Description:
    Initialize control parameters: PI coefficients, scaling constants etc.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitControlParameters(void)
{
    ctrlParm.qRefRamp = SPEED_RAMP_RATE_COUNT;
    ctrlParm.speedRampCount = SPEED_RAMP_SKIP_COUNT;
    /* Set PWM period to Loop Time */
    pwmPeriod = LOOPTIME_TCY;
    
    /*Initialize D axis current controller */
    D_Axis_PISTATE.kp = D_CURRCNTR_PTERM;
    D_Axis_PISTATE.ki = D_CURRCNTR_ITERM; 
    D_Axis_PISTATE.kc = Q15(0.999);
    D_Axis_PISTATE.nkp = D_CURRCNTR_PTERM_SCALE;
    D_Axis_PISTATE.nki = D_CURRCNTR_ITERM_SCALE;
    D_Axis_PISTATE.outMax = D_CURRCNTR_OUTMAX;
    D_Axis_PISTATE.outMin = -D_CURRCNTR_OUTMAX;
    D_Axis_PISTATE.integrator = 0;

    /*Initialize Q axis current controller */
    Q_Axis_PISTATE.kp = Q_CURRCNTR_PTERM;
    Q_Axis_PISTATE.ki = Q_CURRCNTR_ITERM;
    Q_Axis_PISTATE.kc = Q15(0.999);
    Q_Axis_PISTATE.nkp = Q_CURRCNTR_PTERM_SCALE;
    Q_Axis_PISTATE.nki = Q_CURRCNTR_ITERM_SCALE;
    Q_Axis_PISTATE.outMax = Q_CURRCNTR_OUTMAX;
    Q_Axis_PISTATE.outMin = -Q_CURRCNTR_OUTMAX;
    Q_Axis_PISTATE.integrator = 0;

    /*Initialize speed controller */
    Speed_PISTATE.kp = SPEEDCNTR_PTERM; 
    Speed_PISTATE.ki = SPEEDCNTR_ITERM; 
    Speed_PISTATE.kc = Q15(0.999);
    Speed_PISTATE.nkp = SPEEDCNTR_PTERM_SCALE;
    Speed_PISTATE.nki = SPEEDCNTR_ITERM_SCALE;
    Speed_PISTATE.outMax = SPEEDCNTR_OUTMAX;
    Speed_PISTATE.outMin = -SPEEDCNTR_OUTMAX;
    Speed_PISTATE.integrator = 0;
    
    SpeedControlCount=0;
}


/**
  Function: DCLinkVoltageCompensation()     

  Description:
        Apply DC link voltage compensation on the forward path.
*/
void DCLinkVoltageCompensation(MC_ABC_T *pvabcIn, MC_ABC_T *pvabcOut, int16_t vdc)
{
    int16_t rVdc, rVdc_scale=0;
    if (vdc > DC_BUS_COMP)
    {
        rVdc = __builtin_divf(DC_BUS_COMP, vdc);
        rVdc_scale = 0;
    }
    else if (vdc > (DC_BUS_COMP>>1))
    {
        rVdc = __builtin_divf((DC_BUS_COMP>>1), vdc);
        rVdc_scale = 1;
    }
    else
    {
        rVdc = 0;
        rVdc_scale = 0;
    }
    pvabcOut->a = (int16_t) (__builtin_mulss(pvabcIn->a, rVdc) >> (15-rVdc_scale));
    pvabcOut->b = (int16_t) (__builtin_mulss(pvabcIn->b, rVdc) >> (15-rVdc_scale));
    pvabcOut->c = (int16_t) (__builtin_mulss(pvabcIn->c, rVdc) >> (15-rVdc_scale));
}

/**
  Function: DCLinkVoltageCompensation()     

  Description:
    Calculates scaled 3 phase reference vectors for 
    MC_CalculateSpaceVectorPhaseShifted_Assembly from 3 phase voltage vectors.
*/
void VoltageBaseChange(MC_ABC_T *pdabc, MC_ABC_T *pdvabc)
{
    /* Multiply by Root(3) */
    pdvabc->a = (int16_t) (__builtin_mulss(pdabc->a, 28377) >> 14);
    pdvabc->b = (int16_t) (__builtin_mulss(pdabc->b, 28377) >> 14);
    pdvabc->c = (int16_t) (__builtin_mulss(pdabc->c, 28377) >> 14);
}

void pwmDutyCycleSet(MC_DUTYCYCLEOUT_T *pPwmDutycycle)
{  
    INVERTERA_PWM_PDC3 = pPwmDutycycle->dutycycle3;
    INVERTERA_PWM_PDC2 = pPwmDutycycle->dutycycle2;
    INVERTERA_PWM_PDC1 = pPwmDutycycle->dutycycle1;
}

void pwmDutyCycleLimit (MC_DUTYCYCLEOUT_T *pPwmDutycycle,uint16_t min,uint16_t max)
{
    pPwmDutycycle->dutycycle1 = UTIL_LimitU16(pPwmDutycycle->dutycycle1,min,max);
    pPwmDutycycle->dutycycle2 = UTIL_LimitU16(pPwmDutycycle->dutycycle2,min,max);
    pPwmDutycycle->dutycycle3 = UTIL_LimitU16(pPwmDutycycle->dutycycle3,min,max);
}





