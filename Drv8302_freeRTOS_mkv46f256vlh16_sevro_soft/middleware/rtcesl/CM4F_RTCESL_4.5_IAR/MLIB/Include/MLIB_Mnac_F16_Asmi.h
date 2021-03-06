.\mk64f12\main.o: ..\src\main.c
.\mk64f12\main.o: ..\..\..\..\Libraries\drivers\K\inc\gpio.h
.\mk64f12\main.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\main.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\main.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\main.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\main.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\main.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\main.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\main.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\main.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\main.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
.\mk64f12\main.o: ..\..\..\..\Libraries\drivers\K\inc\uart.h
.\mk64f12\main.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdio.h
.\mk64f12\main.o: ..\..\..\..\Libraries\drivers\K\inc\cpuidy.h
                              Mnac_F16_FAsmi( f16Accum, f16Mult1, f16Mult2)
#define MLIB_MnacSat_F16_Asmi( f16Accum, f16Mult1, f16Mult2)                   \
        MLIB_MnacSat_F16_FAsmi( f16Accum, f16Mult1, f16Mult2)
  
/***************************************************************************//*!
*
* f16Out = - f16Accum + ( f16Mult1 * f16Mult2)
* Without saturation
*******************************************************************************/
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline frac16_t MLIB_Mnac_F16_FAsmi(register frac16_t f16Accum, 
                                           register frac16_t f16Mult1, register frac16_t f16Mult2)
{
    #if defined(__CC_ARM)                                    /* For ARM Compiler */
        __asm volatile{ sxth f16Accum, f16Accum              /* Transforms 16-bit input f16Val to 32-bit */
                        smulbb f16Mult1, f16Mult1, f16Mult2  /* f16Mult1 * f16Mult2 */
                        asr f16Mult1, f16Mult1, #15          /* f16Mult1 * f16Mult2 >> 15 */ 
                        sub f16Accum, f16Mult1, f16Accum};   /* f16Accum = f16Mult1 * f16Mult2 - f16Accum*/
    #else
        __asm volatile( "sxth %0, %0 \n"                     /* Transforms 16-bit input f16Val to 32-bit */
                        "smulbb %1, %1, %2 \n"               /* f16Mult1 * f16Mult2 */
                        "asr %1, %1, #15 \n"                 /* f16Mult1 * f16Mult2 >> 15 */ 
                        "sub %0, %1, %0 \n"                  /* f16Accum = f16Mult1 * f16Mult2 - f16Accum*/
                        : "+l"(f16Accum), "+l"(f16Mult1): "l"(f16Mult2));
    #endif

    return f16Accum;
}
RTCESL_INLINE_OPTIM_RESTORE 

/***************************************************************************//*!
*
* f16Out = - f16Accum + ( f16Mult1 * f16Mult2)
* With saturation
*******************************************************************************/
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
static inline frac16_t MLIB_MnacSat_F16_FAsmi(register frac16_t f16Accum, 
                                              register frac16_t f16Mult1, register frac16_t f16Mult2)
{
    #if defined(__CC_ARM)                                    /* For ARM Compiler */
        __asm volatile{ sxth f16Accum, f16Accum              /* Transforms 16-bit input f16Val to 32-bit */
                        smulbb f16Mult1, f16Mult1, f16Mult2  /* f16Mult1 * f16Mult2 */
                        asr f16Mult1, f16Mult1, #15          /* f16Mult1 * f16Mult2 >> 15 */ 
                        sub f16Accum, f16Mult1, f16Accum     /* f16Accum = f16Mult1 * f16Mult2 - f16Accum*/
                        ssat f16Accum, #16, f16Accum};       /* Saturation */
    #else
        __asm volatile( "sxth %0, %0 \n"                     /* Transforms 16-bit input f16Val to 32-bit */
                        "smulbb %1, %1, %2 \n"               /* f16Mult1 * f16Mult2 */
                        "asr %1, %1, #15 \n"                 /* f16Mult1 * f16Mult2 >> 15 */ 
                        "sub %0, %1, %0 \n"                  /* f16Accum = f16Mult1 * f16Mult2 - f16Accum*/
                        "ssat %0, #16, %0 \n"                /* Saturation */
                        : "+l"(f16Accum), "+l"(f16Mult1): "l"(f16Mult2));
    #endif

    return f16Accum;
}
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_MNAC_F16_ASM_H_ */
