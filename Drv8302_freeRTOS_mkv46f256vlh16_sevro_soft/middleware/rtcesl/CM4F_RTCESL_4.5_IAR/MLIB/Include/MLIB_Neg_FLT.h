.\mk64f12\sd.o: ..\..\..\..\Libraries\drivers\K\src\sd.c
.\mk64f12\sd.o: ..\..\..\..\Libraries\drivers\K\inc\sd.h
.\mk64f12\sd.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\sd.o: ..\..\..\..\Libraries\drivers\K\inc\gpio.h
.\mk64f12\sd.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\sd.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\sd.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\sd.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\sd.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\sd.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\sd.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\sd.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\sd.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
                                                                                                                                                              ****************//*!
*
* fltOut = -fltVal
* The output saturation is not implemented, thus in case 
* the negation of input value is outside the (-2^128, 2^128) 
* interval, the output value will overflow without any detection.
*******************************************************************************/ 
static inline float_t MLIB_Neg_FLT_FCi(register float_t fltVal)
{
    return((float_t)(-fltVal));
}
 
#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_NEG_FLT_H_ */
