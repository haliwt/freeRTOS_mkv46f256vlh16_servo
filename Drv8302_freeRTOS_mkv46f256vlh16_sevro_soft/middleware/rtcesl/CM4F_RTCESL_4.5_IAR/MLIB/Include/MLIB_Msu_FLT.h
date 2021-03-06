.\mk64f12\pit.o: ..\..\..\..\Libraries\drivers\K\src\pit.c
.\mk64f12\pit.o: ..\..\..\..\Libraries\drivers\K\inc\pit.h
.\mk64f12\pit.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\pit.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\pit.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\pit.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\pit.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\pit.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\pit.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\pit.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\pit.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\pit.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
                                                                                                                                                                                                             fltMult1, fltMult2)
  
/***************************************************************************//*!
*
* fltOut = fltAccum - ( fltMult1 * fltMult2)
* The output saturation is not implemented, thus in case 
* the MSU of input values is outside the (-2^128, 2^128) 
* interval, the output value will overflow without any detection.
*******************************************************************************/  
static inline float_t MLIB_Msu_FLT_FCi(register float_t fltAccum, 
                                       register float_t fltMult1, register float_t fltMult2)
{
    return((float_t) (fltAccum - (fltMult1 * fltMult2)));
}
 
#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_MSU_FLT_H_ */
