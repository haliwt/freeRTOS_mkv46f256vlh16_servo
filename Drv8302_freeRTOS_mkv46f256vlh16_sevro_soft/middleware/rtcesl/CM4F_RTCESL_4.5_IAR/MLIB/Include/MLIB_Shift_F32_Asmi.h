.\mk64f12\spi.o: ..\..\..\..\Libraries\drivers\K\src\spi.c
.\mk64f12\spi.o: ..\..\..\..\Libraries\drivers\K\inc\spi.h
.\mk64f12\spi.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\spi.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\spi.o: ..\..\..\..\Libraries\drivers\K\inc\gpio.h
.\mk64f12\spi.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\spi.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\spi.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\spi.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\spi.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\spi.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\spi.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\spi.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
                                                                                                                                               16Sh) MLIB_ShLSat_F32_FAsmi(f32Val, u16Sh)

/***************************************************************************//*!
*
* This function returns the f32Val input shifted by 1 to the left. 
*              The function saturates the output.
*
*******************************************************************************/ 
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline frac32_t MLIB_Sh1LSat_F32_FAsmi(register frac32_t f32Val)
{
    #if defined(__CC_ARM)                              /* For ARM Compiler */
        __asm volatile{ qadd f32Val, f32Val, f32Val }; /* Addition with saturation */
    #else
        __asm volatile("qadd %0, %0, %0 \n"            /* Addition with saturation */
                        : "+l"(f32Val):);
    #endif

    return f32Val;
}
RTCESL_INLINE_OPTIM_RESTORE 

/***************************************************************************//*!
*
* This function returns the f32Val input shifted by the number of u16Sh to the left. 
*          The function saturates the output.
*
*******************************************************************************/  
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline frac32_t MLIB_ShLSat_F32_FAsmi(register frac32_t f32Val, register uint16_t u16Sh)
{
    register frac32_t f32Out=0;
    register frac32_t f32SatVal = f32Val;
    #if defined(__CC_ARM)                                    /* For ARM Compiler */
        __asm volatile{ ssat f32SatVal, #2, f32SatVal        /* Saturates f32SatVal to 2-bit value */
                        lsl f32SatVal, f32SatVal, #30        /* f32SatVal << 30 */
                        qadd f32SatVal, f32SatVal, f32SatVal /* Addition with saturation */
                        lsl f32Out, f32Val, u16Sh            /* f32Out = f32Val << u16Sh */
                        asr u16Sh, f32Out, u16Sh             /* u16Sh = f32Out << u16Sh */
                        cmp u16Sh, f32Val                    /* Compares u16Sh and f32Val */
                        it ne                                /* If u16Sh != f32Val, then returns the saturate value */
                        movne f32Out, f32SatVal };
    #else
        __asm volatile( "ssat %1, #2, %1 \n"                 /* Saturates f32SatVal to 2-bit value */
                        "lsl %1, %1, #30 \n"                 /* f32SatVal << 30 */
                        "qadd %1, %1, %1 \n"                 /* Addition with saturation */
                        "lsl %0, %2, %3 \n"                  /* f32Out = f32Val << u16Sh */
                        "asr %3, %0, %3 \n"                  /* u16Sh = f32Out << u16Sh */
                        "cmp %3, %2 \n"                      /* Compares u16Sh and f32Val */
                        "it ne \n"                           /* If u16Sh != f32Val, then returns the saturate value */
                        "movne %0, %1 \n"
                        : "=&l"(f32Out), "+l"(f32SatVal), "+l"(f32Val), "+l"(u16Sh):);
    #endif

    return f32Out;
}
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_SHIFT_F32_ASM_H_ */
