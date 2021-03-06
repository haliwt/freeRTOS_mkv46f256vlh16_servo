.\mk64f12\pdb.o: ..\..\..\..\Libraries\drivers\K\src\pdb.c
.\mk64f12\pdb.o: ..\..\..\..\Libraries\drivers\K\inc\pdb.h
.\mk64f12\pdb.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\pdb.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\pdb.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\pdb.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\pdb.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\pdb.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\pdb.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\pdb.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\pdb.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\pdb.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
                                                                                                                                                                                                              MLIB_Msu4Sat_F32ssss_FAsmi(f16MinMul1, f16MinMul2, f16SubMul1, f16SubMul2)

/***************************************************************************//*!
*
* f32Out = (f16MinMul1 * f16MinMul2) - (f16SubMul1 * f16SubMul2)
* With saturation
*******************************************************************************/
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline frac32_t MLIB_Msu4Sat_F32ssss_FAsmi(register frac16_t f16MinMul1,register frac16_t f16MinMul2,
                                                  register frac16_t f16SubMul1,register frac16_t f16SubMul2)
{
    register frac32_t f32Out=0;
    #if defined(__CC_ARM)                                         /* For ARM Compiler */
        __asm volatile{ smulbb f32Out, f16MinMul1, f16MinMul2     /* f16MinMul1 * f16MinMul2 */
                        smulbb f16MinMul1, f16SubMul1, f16SubMul2 /* f16SubMul1 * f16SubMul2 */
                        qsub f32Out, f32Out, f16MinMul1           /* Subtraction with saturation */
                        qadd f32Out, f32Out, f32Out };            /* Addition with saturation */
    #else
        __asm volatile( "smulbb %1, %1, %2 \n"                    /* f16MinMul1 * f16MinMul2 */
                        "smulbb %2, %3, %4 \n"                    /* f16SubMul1 * f16SubMul2 */
                        "qsub %2, %1, %2 \n"                      /* Subtraction with saturation */
                        "qadd %0, %2, %2 \n"                      /* Addition with saturation */
                        : "=l"(f32Out), "+l"(f16MinMul1), "+l"(f16MinMul2): "l"(f16SubMul1), "l"(f16SubMul2));
    #endif

    return f32Out;
}
RTCESL_INLINE_OPTIM_RESTORE 


#if defined(__cplusplus)
}
#endif

#endif  /* _MLIB_MSU4_F32_ASM_H_ */
