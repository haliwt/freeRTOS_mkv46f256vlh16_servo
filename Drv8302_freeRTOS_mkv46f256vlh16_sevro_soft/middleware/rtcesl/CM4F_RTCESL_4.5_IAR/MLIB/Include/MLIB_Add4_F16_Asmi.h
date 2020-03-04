.\mk64f12\flexbus.o: ..\..\..\..\Libraries\drivers\K\src\flexbus.c
.\mk64f12\flexbus.o: ..\..\..\..\Libraries\drivers\K\inc\flexbus.h
.\mk64f12\flexbus.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\flexbus.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\flexbus.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\flexbus.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\flexbus.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\flexbus.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\flexbus.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\flexbus.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\flexbus.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\flexbus.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
                                                                                                                                                    Add4_F16_FAsmi(f16Add1, f16Add2, f16Add3, f16Add4)
#define MLIB_Add4Sat_F16_Asmi(f16Add1, f16Add2, f16Add3, f16Add4)               \
        MLIB_Add4Sat_F16_FAsmi(f16Add1, f16Add2, f16Add3, f16Add4)

/***************************************************************************//*!
*  Summation of four inputs
*     f16Out = f16Add1 + f16Add2 + f16Add3 + f16Add4
*  function is not saturated
*******************************************************************************/
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline frac16_t MLIB_Add4_F16_FAsmi(register frac16_t f16Add1, register frac16_t f16Add2,
                                           register frac16_t f16Add3, register frac16_t f16Add4)
{
    #if defined(__CC_ARM)                               /* For ARM Compiler */
        __asm volatile{add f16Add1, f16Add1, f16Add2    /* f16Add1 = f16Add1 + f16Add2 */
                       add f16Add1, f16Add1, f16Add3    /* f16Add1 = f16Add1 + f16Add3 */
                       add f16Add1, f16Add1, f16Add4};  /* f16Add1 = f16Add1 + f16Add4 */

    #else
        __asm volatile( "add %0, %0, %1 \n"             /* f16Add1 = f16Add1 + f16Add2 */
                        "add %0, %0, %2 \n"             /* f16Add1 = f16Add1 + f16Add3 */
                        "add %0, %0, %3 \n"             /* f16Add1 = f16Add1 + f16Add4 */
                        : "+l"(f16Add1): "l"(f16Add2), "l"(f16Add3), "l"(f16Add4));
    #endif

    return f16Add1;
}
RTCESL_INLINE_OPTIM_RESTORE 

/***************************************************************************//*!
*  Saturated summation of four inputs
*     f16Out = f16Add1 + f16Add2 + f16Add3 + f16Add4
*******************************************************************************/
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline frac16_t MLIB_Add4Sat_F16_FAsmi(register frac16_t f16Add1, register frac16_t f16Add2,
                                              register frac16_t f16Add3, register frac16_t f16Add4)
{        
    #if defined(__CC_ARM)                                   /* For ARM Compiler */
        __asm volatile{ sxth f16Add1, f16Add1               /* Transforms 16-bit input f16Val to 32-bit */
                        sxth f16Add2, f16Add2               /* Transforms 16-bit input f16Val to 32-bit */
                        sxth f16Add3, f16Add3               /* Transforms 16-bit input f16Val to 32-bit */
                        sxth f16Add4, f16Add4               /* Transforms 16-bit input f16Val to 32-bit */
                        add f16Add1, f16Add1, f16Add3       /* f16Add1 + f16Add3 */
                        add f16Add2, f16Add2, f16Add4       /* f16Add2 + f16Add4 */
                        add f16Add1, f16Add1, f16Add2       /* f16Add1 + f16Add3 + f16Add2 + f16Add4 */
                        ssat f16Add1, #16, f16Add1 };       /* Saturation */
    #else
        __asm volatile( "sxth %0, %0 \n"                     /* Transforms 16-bit input f16Val to 32-bit */
                        "sxth %1, %1 \n"                     /* Transforms 16-bit input f16Val to 32-bit */
                        "sxth %2, %2 \n"                     /* Transforms 16-bit input f16Val to 32-bit */
                        "sxth %3, %3 \n"                     /* Transforms 16-bit input f16Val to 32-bit */
                        "add %0, %0, %2 \n"                  /* f16Add1 + f16Add3 */
                        "add %1, %1, %3 \n"                  /* f16Add2 + f16Add4 */
                        "add %0, %0, %1 \n"                  /* f16Add1 + f16Add3 + f16Add2 + f16Add4 */
                        "ssat %0, #16, %0 \n"                /* Saturation */
                        : "+l"(f16Add1), "+l"(f16Add2), "+l"(f16Add3), "+l"(f16Add4):);
    #endif

    return f16Add1;
}
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__cplusplus)
}
#endif

#endif  /* _MLIB_ADD4_F16_ASM_H_ */
