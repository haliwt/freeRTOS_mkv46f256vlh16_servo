.\mk64f12\rtc.o: ..\..\..\..\Libraries\drivers\K\src\rtc.c
.\mk64f12\rtc.o: ..\..\..\..\Libraries\drivers\K\inc\rtc.h
.\mk64f12\rtc.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\rtc.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\rtc.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\rtc.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\rtc.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\rtc.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\rtc.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\rtc.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\rtc.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\rtc.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
                                                                                                                                                                                                            6Mult2) MLIB_MulNeg_F16_FCi(f16Mult1, f16Mult2)
  
/***************************************************************************//*!
*
* f16Out = f16Mult1 * f16Mult2
* Without saturation
*******************************************************************************/ 
static inline frac16_t MLIB_Mul_F16_FCi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
    return((frac16_t)(((int32_t)(f16Mult1)*(int32_t)(f16Mult2))>>15));
}
 
/***************************************************************************//*!
*
* f16Out = f16Mult1 * f16Mult2
* 
*******************************************************************************/
static inline frac16_t MLIB_MulNeg_F16_FCi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
    return((frac16_t)(((int32_t)(f16Mult1)*(int32_t)(-f16Mult2))>>15));
}
 
#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_MUL_F16_H_ */
