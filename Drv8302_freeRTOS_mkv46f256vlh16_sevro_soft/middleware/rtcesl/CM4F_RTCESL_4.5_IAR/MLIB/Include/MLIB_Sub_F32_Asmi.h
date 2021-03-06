.\mk64f12\system_mk64f12.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.c
.\mk64f12\system_mk64f12.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\system_mk64f12.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\system_mk64f12.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\system_mk64f12.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\system_mk64f12.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\system_mk64f12.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\system_mk64f12.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
                                                                                                                                                                                                                                                                                                                                                           **************************************************//*!
*
* f32Out = f32Min - f32Sub
* With saturation
*******************************************************************************/ 
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline frac32_t MLIB_SubSat_F32_FAsmi(register frac32_t f32Min, register frac32_t f32Sub)
{
    #if defined(__CC_ARM)                                   /* For ARM Compiler */
        __asm volatile{ qsub f32Min, f32Min, f32Sub};       /* Subtracts with saturation */
    #else
        __asm volatile( "qsub %0, %0, %1 \n"                /* Subtracts with saturation */
                        : "+l"(f32Min): "l"(f32Sub));
    #endif

    return f32Min;
}
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_SUB_F32_ASM_H_ */
