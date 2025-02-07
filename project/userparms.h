// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file userparms.h
 *
 * @brief This file has definitions to be configured by the user.
 *
 * Board : MCHV-230V AC -1.5kW + dsPIC33CK256MP508 MC DIM
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

#ifndef USERPARMS_H
#define USERPARMS_H

#ifdef __cplusplus
extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include "general.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

    
/** Define macros for operational Modes */
    
/* Define OPEN_LOOP_FUNCTIONING for Open loop continuous functioning, 
 * undefine OPEN_LOOP_FUNCTIONING for closed loop functioning  */
#undef OPEN_LOOP_FUNCTIONING 

/* Define FDWEAK_CONTROL_ENABLE to enable Field weakening control */
#define FDWEAK_CONTROL_ENABLE
    
/* Select Field weakening method 
 * Define the FD_WEAK_TYPE_VOLTAGE_FB macro for voltage feedback based Field 
 * weakening control. Undefine the macro for conventional inverse speed 
 * based Field weakening control
 * */    
#define FD_WEAK_TYPE_VOLTAGE_FB
    
 
/* Definition for torque mode - for a separate tuning of the current PI
controllers, tuning mode will disable the speed PI controller */
#undef TORQUE_MODE
    
/* Define for Internal OP-AMP DIM. For external, undefine */
#define     INTERNAL_OPAMP_CONFIG   

/** Board Parameters */
/** Board Parameters */
#define     MC1_PEAK_VOLTAGE            453.3  /* Peak measurement voltage of the board */
#define     MC1_PEAK_CURRENT            5     /* Peak measurement current of the board */
         
    
/****************************** Motor Parameters ******************************/

/* Define to run ZD Induction motor*/    
#define ZD_412K25A_MOTOR
    
#undef LEESON_MOTOR

/* The following values are given in the xls attached file */     
#ifdef ZD_412K25A_MOTOR
    #define POLEPAIRS           2  /* Motor's number of pole pairs */
    #define NOMINAL_SPEED_RPM   1300 /* Nominal speed of the motor in RPM */
    #define MAXIMUM_SPEED_RPM   1.3*1300 /* Maximum speed of the motor in RPM */
    #define MINIMUM_SPEED_RPM   500 /* Minimum speed of the motor in RPM*/
    /*Motor Rated Line - Line RMS Voltage*/
    #define NOMINAL_VOLTAGE_L_L  (float)220
    /* Motor Rated Phase Current RMS in Amps */
    #define NOMINAL_CURRENT_PHASE_RMS (float) 0.25
    /* Motor Rated Magnetization Current RMS in Amps */
    #define NOMINAL_MAGNITIZING_CURRENT_RMS (float) 0.14
    /* Motor Rated Torque Component of Current RMS in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_RMS (float) 0.2

    /* Base values entered in .xlsx file*/
                                     /* Base Current */
    #define MC1_BASE_VOLTAGE    325  /* Vdc Base voltage = Rated voltage*1.414*/
    #define MC1_PEAK_SPEED_RPM  2.5*NOMINAL_SPEED_RPM   /* Base Speed in RPM */

    /*Maximum utilizable Voltage in V/F Control (open loop Control)*/
    #define VMAX_VBYF_CONTROL   (int16_t)((float)0.9*18918) /* 0.9* Vdclink/root3, 32767/sqrt(3) = 18918*/
    /*Maximum utilizable Voltage Limit in closed loop control*/
    #define VMAX_CLOSEDLOOP_CONTROL  (int16_t)((float)0.8*18918) /* 0.85* Vdclink/root3 */ 
    /* Motor Rated Torque Component of Current Peak in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK (float) (NOMINAL_TORQUE_COMPONENT_CURRENT_RMS * 1.414)

    /* The following values are given in the xls attached file */
    /* Open loop VbyF control parameters */
    #define	VBYF_CONSTANT       23668
    #define	VBYF_CONSTANT_SHIFT	14
    /* PLL Estimator Parameters */
    #define	NORM_RS     14620
    #define	NORM_RS_QVALUE	13
    #define	NORM_SIGMALSDT      10526
    #define	NORM_SIGMALSDT_QVALUE	6
    #define	NORM_INVLMSQRBYLR           1229
    #define	NORM_INVLMSQRBYLR_QVALUE	15
    #define	NORM_DELTAT	355
    #define	KFILTER_IMRESTIM	68
    #define	NORM_INVTR	2001
    #define	D_ILIMIT_HS	142
    #define	D_ILIMIT_LS	1136

    
    /* PI controllers tuning values - */     
    /* D Control Loop Coefficients */
    #define	Q_CURRCNTR_PTERM	5263
    #define	Q_CURRCNTR_ITERM	 1827
    #define	Q_CURRCNTR_PTERM_SCALE	5
    #define	Q_CURRCNTR_ITERM_SCALE  0
    #define Q_CURRCNTR_OUTMAX      VMAX_CLOSEDLOOP_CONTROL

    /* Q Control Loop Coefficients */
    #define D_CURRCNTR_PTERM        Q_CURRCNTR_PTERM
    #define D_CURRCNTR_PTERM_SCALE  Q_CURRCNTR_PTERM_SCALE
    #define D_CURRCNTR_ITERM        Q_CURRCNTR_ITERM
    #define D_CURRCNTR_ITERM_SCALE  Q_CURRCNTR_ITERM_SCALE
    #define D_CURRCNTR_OUTMAX       VMAX_CLOSEDLOOP_CONTROL

/**********************  support xls file definitions end *********************/
    
    /* Velocity Control Loop Coefficients */    
    #define SPEEDCNTR_PTERM        Q15(0.3)
    #define	SPEEDCNTR_PTERM_SCALE   0
    #define SPEEDCNTR_ITERM         8//Q15(0.00005)
    #define SPEEDCNTR_ITERM_SCALE   0
    #define SPEEDCNTR_OUTMAX        NORM_VALUE(NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK,MC1_PEAK_CURRENT)
  
    /* Estimated speed filter constant */
    #define KFILTER_VELESTIM 170

    /* End speed rpm for open loop to closed loop transition */
    #define     END_SPEED_RPM       300  
#endif    
   

#ifdef LEESON_MOTOR
    #define POLEPAIRS           1  /* Motor's number of pole pairs */
    #define NOMINAL_SPEED_RPM   3450 /* Nominal speed of the motor in RPM */
    #define MAXIMUM_SPEED_RPM   1.3*3450 /* Maximum speed of the motor in RPM */
    #define MINIMUM_SPEED_RPM   500 /* Minimum speed of the motor in RPM*/
    /*Motor Rated Line - Line RMS Voltage*/
    #define NOMINAL_VOLTAGE_L_L  (float)230
    /* Motor Rated Phase Current RMS in Amps */
    #define NOMINAL_CURRENT_PHASE_RMS (float)  1.4
    /* Motor Rated Magnetization Current RMS in Amps */
    #define NOMINAL_MAGNITIZING_CURRENT_RMS (float) 0.79
    /* Motor Rated Torque Component of Current RMS in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_RMS (float) 1.15

    /* Base values entered in .xlsx file*/
                                     /* Base Current */
    #define MC1_BASE_VOLTAGE    325  /* Vdc Base voltage = Rated voltage*1.414*/
    #define MC1_PEAK_SPEED_RPM  2.5*NOMINAL_SPEED_RPM   /* Base Speed in RPM */

    /*Maximum utilizable Voltage in V/F Control (open loop Control)*/
    #define VMAX_VBYF_CONTROL   (int16_t)((float)0.9*18918) /* 0.9* Vdclink/root3, 32767/sqrt(3) = 18918*/
    /*Maximum utilizable Voltage Limit in closed loop control*/
    #define VMAX_CLOSEDLOOP_CONTROL  (int16_t)((float)0.82*18918) /* 0.85* Vdclink/root3 */ 
    /* Motor Rated Torque Component of Current Peak in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK (float) (NOMINAL_TORQUE_COMPONENT_CURRENT_RMS * 1.414)

    /* The following values are given in the xls attached file */
    /* Open loop VbyF control parameters */
    #define	VBYF_CONSTANT       23652
    #define	VBYF_CONSTANT_SHIFT	14
    /* PLL Estimator Parameters */
    #define	NORM_RS	3781
    #define	NORM_RS_QVALUE	15
    #define	NORM_SIGMALSDT	19762
    #define	NORM_SIGMALSDT_QVALUE	11
    #define	NORM_INVLMSQRBYLR	5567
    #define	NORM_INVLMSQRBYLR_QVALUE	15
    #define	NORM_DELTAT	471
    #define	KFILTER_IMRESTIM	14
    #define	NORM_INVTR	309
    #define	D_ILIMIT_HS	188
    #define	D_ILIMIT_LS	1507

    
    /* PI controllers tuning values - */     
    /* D Control Loop Coefficients */
    #define	Q_CURRCNTR_PTERM        13723
    #define	Q_CURRCNTR_ITERM        656
    #define	Q_CURRCNTR_PTERM_SCALE	2
    #define	Q_CURRCNTR_ITERM_SCALE  0
    #define Q_CURRCNTR_OUTMAX      VMAX_CLOSEDLOOP_CONTROL

    /* Q Control Loop Coefficients */
    #define D_CURRCNTR_PTERM        Q_CURRCNTR_PTERM
    #define D_CURRCNTR_PTERM_SCALE  Q_CURRCNTR_PTERM_SCALE
    #define D_CURRCNTR_ITERM        Q_CURRCNTR_ITERM
    #define D_CURRCNTR_ITERM_SCALE  Q_CURRCNTR_ITERM_SCALE
    #define D_CURRCNTR_OUTMAX       VMAX_CLOSEDLOOP_CONTROL

/**********************  support xls file definitions end *********************/
    
    /* Velocity Control Loop Coefficients */    
    #define SPEEDCNTR_PTERM        Q15(0.5)
    #define	SPEEDCNTR_PTERM_SCALE   1
    #define SPEEDCNTR_ITERM         5//Q15(0.00005)
    #define SPEEDCNTR_ITERM_SCALE   0
    #define SPEEDCNTR_OUTMAX        NORM_VALUE(NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK,MC1_PEAK_CURRENT)
  
    /* Estimated speed filter constant */
    #define KFILTER_VELESTIM 250

    /* End speed rpm for open loop to closed loop transition */
    #define     END_SPEED_RPM       450  
#endif    
   
     
/*************************************************************************/    
    
/* DC bus compensation factor */    
#define DC_BUS_COMP     (int16_t)((float)MC1_BASE_VOLTAGE*32767/MC1_PEAK_VOLTAGE)

/* Maximum voltage square */
#define MAX_VOLTAGE_SQUARE   (int16_t)( (float)VMAX_CLOSEDLOOP_CONTROL*VMAX_CLOSEDLOOP_CONTROL/32767 )
  
/* Minimum motor open loop speed converted into Normalized Minimum speed */        
#define Q15_MINIMUMSPEED  NORM_VALUE(MINIMUM_SPEED_RPM,MC1_PEAK_SPEED_RPM)   
/* Maximum motor speed converted into Normalized Maximum speed */
#define Q15_MAXIMUMSPEED NORM_VALUE(MAXIMUM_SPEED_RPM,MC1_PEAK_SPEED_RPM)
/* Nominal motor speed converted into Normalized Nominal speed */
#define Q15_NOMINALSPEED NORM_VALUE(NOMINAL_SPEED_RPM,MC1_PEAK_SPEED_RPM)
/* END_SPEED_RPM converted into Normalized End speed */
#define Q15_ENDSPEED NORM_VALUE(END_SPEED_RPM,MC1_PEAK_SPEED_RPM)
    
/* Speed controller sampling factor: 
 * Sampling Time for speed controller = Ts/SPEEDCNTR_SAMPLING_FACTOR */
#define SPEEDCNTR_SAMPLING_FACTOR   1
    
/* Filters constants definitions  */
/* BEMF filter for d-q components @ low speeds */
#define KFILTER_ESDQ 3*164
/* BEMF filter for d-q components @ high speed - Flux Weakening case */
#define KFILTER_ESDQ_FW 3*164

/* Normalized value of Veloctiy Threshhold used in Estimator*/    
#define PLL_VELOCITY_FILTER_THRESHOLD  Q15_NOMINALSPEED

/* Speed Reference Ramp parameters*/
#define     SPEED_RAMP_RATE_COUNT      1  /* Speed change rate in perunit counts */
#define     SPEED_RAMP_SKIP_COUNT      6 /* Sample time multiplier for  */
/* Speed rampe rate(Mechanical RPM/sec) = 
 * = (SPEED_RAMP_RATE_COUNT/(LOOPTIME_SEC * SPEED_RAMP_SKIP_COUNT)) *(MC1_PEAK_SPEED_RPM/32767) */
    
/* Motor nominal magnetizing Component of Current Peak in Amps */
#define NOMINAL_MAGNITIZING_CURRENT_PEAK (float) (NOMINAL_MAGNITIZING_CURRENT_RMS * 1.414)

/*Normalized Rated Magnetization Current*/    
#define NORM_NOMINAL_IDREF  NORM_VALUE(NOMINAL_MAGNITIZING_CURRENT_PEAK,MC1_PEAK_CURRENT)
#define NORM_MINIMUM_IDREF  NORM_NOMINAL_IDREF>>1; /*NORM_MINIMUM_IDREF = NORM_NOMINAL_IDREF/2 */

/* Field Weakening Parameters */
    
/*Parameters for inverse speed Field weakening control*/  
#define FD_WEAK_START_SPEED   NORM_VALUE(((float)NOMINAL_SPEED_RPM*0.7), MC1_PEAK_SPEED_RPM)  
    
/*Parameters for voltage feedback based Field weakening control*/   
#define FD_WEAK_VOLTAGE_REF  (int16_t)((float)VMAX_CLOSEDLOOP_CONTROL*0.9) 
#define FD_WEAK_PI_KP               305
#define FD_WEAK_PI_KPSCALE          1
#define FD_WEAK_PI_KI               2
#define FD_WEAK_IDREF_FILT_CONST    350
    
/* Filter Constant for Idref Generation*/    
#define IDREF_FILT_CONST    1000    


/* Normalized value of Iq Limit*/
#define Q15_TORQUE_COMPONENT_CURRENT_PEAK  NORM_VALUE(NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK, MC1_PEAK_CURRENT)   


#ifdef __cplusplus
}
#endif

#endif /* __USERPARMS_H */
