.\mk64f12\crc.o: ..\..\..\..\Libraries\drivers\K\src\crc.c
.\mk64f12\crc.o: ..\..\..\..\Libraries\drivers\K\inc\crc.h
.\mk64f12\crc.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\crc.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\crc.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\crc.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\crc.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\crc.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\crc.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\crc.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\crc.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\crc.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
                                                                                                                                                                                                            *************************************************************//*!
* @brief     This function returns the square root of input value.
*
* @param     in fltVal - The input value.
*
* @return    The function returns the square root of the input value. The
*            return value is float range. If input is <= 0, then the function 
*            returns 0. The function uses VSQRT instruction in Cortex M FPU.
*
* @remarks   The function uses VSQRT instruction in Cortex M FPU.
*
****************************************************************************/  
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline float_t GFLIB_Sqrt_FLT_FAsmi(register float_t fltVal)
{
    #if defined(__CC_ARM)                        /* For ARM Compiler */
        __asm volatile{ vsqrt.f32 fltVal, fltVal };
    #else
        __asm volatile( "vsqrt.f32 %0, %0"
                       : "+t"(fltVal):);
    #endif
    
    return fltVal;
}
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__cplusplus)
}
#endif

#endif  /* _MLIB_SQRT_FLT_ASM_H_*/
