.\mk64f12\common.o: ..\..\..\..\Libraries\drivers\K\src\common.c
.\mk64f12\common.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk64f12\common.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdint.h
.\mk64f12\common.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stdbool.h
.\mk64f12\common.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\stddef.h
.\mk64f12\common.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK64F12.h
.\mk64f12\common.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk64f12\common.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk64f12\common.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk64f12\common.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk64f12\common.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK64F12.h
.\mk64f12\common.o: C:\Keil_v5\ARM\ARMCC\Bin\..\include\string.h
                                                                                                                                                                                         \
        GFLIB_Limit_F32_FCi(f32Val, f32LLim, f32ULim)

/***************************************************************************//*!
*
* @brief    Limit function 32-bit version
* 
* @param    in   frac32_t f32Val  - Argument in <-1;1) in frac32_t
*                frac32_t f32LLim - Lower limit in <-1;1) in frac32_t
*                frac32_t f32ULim - Upper limit in <-1;1) in frac32_t
*                         
* @return   This function returns - frac32_t value <-1;1)
*       
* @remarks  This function trims the argument according to the upper f32ULim and 
*           lower f32LLim limits. The upper limit must >= lower limit.
*
****************************************************************************/  
static inline frac32_t GFLIB_Limit_F32_FCi(frac32_t f32Val, 
                                           frac32_t f32LLim, frac32_t f32ULim)
{
    if(f32Val > f32ULim) 
    {
        return(f32ULim);
    }
    if(f32Val < f32LLim) 
    {
        return(f32LLim);
    }
   
    return(f32Val);
}
 
#if defined(__cplusplus)
}
#endif

#endif /* _GFLIB_LIMIT_F32_H_ */

