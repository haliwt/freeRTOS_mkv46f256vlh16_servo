.\mk64f12\ftm.o: ..\..\..\..\Libraries\drivers\K\src\ftm.c
.\mk64f12\ftm.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\math.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\drivers\K\inc\ftm.h
.\mk64f12\ftm.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\ftm.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\ftm.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
.\mk64f12\ftm.o: ..\..\..\..\Libraries\drivers\K\inc\gpio.h
                                                                                  F16_FCi(f16Val, i16Sh) 
#define MLIB_ShLBiSat_F16_Ci(f16Val, i16Sh) MLIB_ShLBiSat_F16_FCi(f16Val, i16Sh)   
#define MLIB_ShRBi_F16_Ci(f16Val, i16Sh)    MLIB_ShRBi_F16_FCi(f16Val, i16Sh) 
#define MLIB_ShRBiSat_F16_Ci(f16Val, i16Sh) MLIB_ShRBiSat_F16_FCi(f16Val, i16Sh) 
  
/***************************************************************************//*!
*
* This function returns the f16Val input shifted by the number of i16Sh to the left. 
* If the i16Sh is negative, the input is shifted to the right. The function
* does not saturate the output.  
* 
*******************************************************************************/ 
static inline frac16_t MLIB_ShLBi_F16_FCi(register frac16_t f16Val, register int16_t i16Sh)
{
    return (i16Sh<(int16_t)0) ? MLIB_ShR_F16_Ci(f16Val, (uint16_t)(-i16Sh)) :
                                MLIB_ShL_F16_Ci(f16Val, (uint16_t)(i16Sh));
}
 
/***************************************************************************//*!
*
* This function returns the f16Val input shifted by the number of i16Sh to the left. 
* If the i16Sh is negative, the input is shifted to the right. The function
* saturates the output.  
* 
*******************************************************************************/
static inline frac16_t MLIB_ShLBiSat_F16_FCi(register frac16_t f16Val, register int16_t i16Sh)
{
    return (i16Sh<(int16_t)0) ? MLIB_ShR_F16_Ci(f16Val, (uint16_t)(-i16Sh)) : 
                                MLIB_ShLSat_F16_Asmi(f16Val, (uint16_t)(i16Sh));
}
 
/***************************************************************************//*!
*
* This function returns the f16Val input shifted by the number of i16Sh to the right. 
* If the i16Sh is negative, the input is shifted to the left. The function
* does not saturate the output.  
* 
*******************************************************************************/ 
static inline frac16_t MLIB_ShRBi_F16_FCi(register frac16_t f16Val, register int16_t i16Sh)
{
    return (i16Sh<(int16_t)0) ? MLIB_ShL_F16_Ci(f16Val, (uint16_t)(-i16Sh)) : 
                                MLIB_ShR_F16_Ci(f16Val, (uint16_t)(i16Sh));
}
 
/***************************************************************************//*!
*
* This function returns the f16Val input shifted by the number of i16Sh to the right. 
* If the i16Sh is negative, the input is shifted to the left. The function
* saturates the output.  
* 
*******************************************************************************/
static inline frac16_t MLIB_ShRBiSat_F16_FCi(register frac16_t f16Val, register int16_t i16Sh)
{
    return (i16Sh<(int16_t)0) ? MLIB_ShLSat_F16_Asmi(f16Val, (uint16_t)(-i16Sh)) :
                                MLIB_ShR_F16_Ci(f16Val, (uint16_t)(i16Sh));
}
 
#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_BISHIFT_F16_H_ */
