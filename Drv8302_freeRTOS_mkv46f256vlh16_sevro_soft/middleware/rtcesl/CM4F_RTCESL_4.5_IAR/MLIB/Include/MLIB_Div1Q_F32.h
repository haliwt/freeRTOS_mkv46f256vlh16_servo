.\mk64f12\gpio.o: ..\..\..\..\Libraries\drivers\K\src\gpio.c
.\mk64f12\gpio.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\gpio.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\gpio.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\gpio.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\gpio.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\gpio.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\gpio.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\gpio.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\gpio.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\gpio.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
.\mk64f12\gpio.o: ..\..\..\..\Libraries\drivers\K\inc\gpio.h
                                                                                                                                                                                              Q_F16ls_FCi(f32Num, f16Denom)  
#define MLIB_Div1Q_F32ls_C(f32Num, f16Denom)      MLIB_Div1Q_F32ls_FC(f32Num, f16Denom)
#define MLIB_Div1QSat_F16ls_Ci(f32Num, f16Denom)  MLIB_Div1QSat_F16ls_FCi(f32Num, f16Denom)  
#define MLIB_Div1QSat_F32ls_C(f32Num, f16Denom)   MLIB_Div1QSat_F32ls_FC(f32Num, f16Denom)
#define MLIB_Div1Q_F16ll_Asm(f32Num, f32Denom)    MLIB_Div1Q_F16ll_FAsm(f32Num, f32Denom)
#define MLIB_Div1Q_F32_Asm(f32Num, f32Denom)      MLIB_Div1Q_F32_FAsm(f32Num, f32Denom)
#define MLIB_Div1QSat_F16ll_Asm(f32Num, f32Denom) MLIB_Div1QSat_F16ll_FAsm(f32Num, f32Denom)
#define MLIB_Div1QSat_F32_Asm(f32Num, f32Denom)   MLIB_Div1QSat_F32_FAsm(f32Num, f32Denom)

/*******************************************************************************
* Exported function prototypes
*******************************************************************************/  
extern frac32_t MLIB_Div1Q_F32ls_FC(frac32_t f32Num, frac16_t f16Denom);
extern frac32_t MLIB_Div1QSat_F32ls_FC(frac32_t f32Num, frac16_t f16Denom);
extern frac16_t MLIB_Div1Q_F16ll_FAsm(frac32_t f32Num, frac32_t f32Denom);
extern frac32_t MLIB_Div1Q_F32_FAsm(frac32_t f32Num, frac32_t f32Denom);
extern frac16_t MLIB_Div1QSat_F16ll_FAsm(frac32_t f32Num, frac32_t f32Denom);
extern frac32_t MLIB_Div1QSat_F32_FAsm(frac32_t f32Num, frac32_t f32Denom);

/***************************************************************************//*!
* @brief  32-bit numerator, 16-bit denominator inputs 16-output 1-quadrant
*         division function
*
* @param  in  frac32_t f32Num  - Numerator in <-1;1) in frac32_t
*             frac16_t f16Denom- Denominator in <-1;1) in frac16_t
*
* @return This function returns - frac16_t value <-1;1)
*       
* @remarks  This function divides two fractional inputs:
*           result = f32Num / f16Denom.
*           The function does not saturate the output.
*           If the denominator is 0, the output is 0x7FFF.
*
*******************************************************************************/
static inline frac16_t MLIB_Div1Q_F16ls_FCi(register frac32_t f32Num, register frac16_t f16Denom)
{
    if (f16Denom == (frac16_t)0)
    {
        return(frac16_t)(INT16_MAX);
    }
    else
    {
        return(frac16_t)((frac32_t)((int32_t)(f32Num) / (int32_t)(f16Denom)) >> 1);
    }
}
 
/***************************************************************************//*!
* @brief  32-bit numerator, 16-bit denominator inputs 32-output single quadrant
*         division function
* 
* @param  in  frac32_t f32Num  - Numerator in <-1;1) in frac32_t
*             frac16_t f16Denom- Denominator in <-1;1) in frac16_t                      
*
* @return This function returns- frac16_t value <0;1)
*       
* @remarks  This function divides two  non-negative fractional inputs:
*           result = f32Num / f16Denom.
*           The function saturates the output if f32Num > f16Denom
*           to 0x7FFF FFFF. 
*
*******************************************************************************/
static inline frac16_t MLIB_Div1QSat_F16ls_FCi(register frac32_t f32Num, register frac16_t f16Denom)
{
    if (f16Denom == (frac16_t)0)
    {
        return(frac16_t)(INT16_MAX);
    }
    else
    {
        if (f32Num >= (((frac32_t)f16Denom)<<16))
        {
            return(frac16_t)(INT16_MAX);
        }
        else
        {
            return(frac16_t)((frac32_t)((int32_t)(f32Num) / (int32_t)(f16Denom)) >> 1);
        }
    }
}
 
#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_DIV1Q_F32_H_ */
