<?xml version="1.0" encoding="iso-8859-1"?>

<workspace>
  <project>
    <path>$WS_DIR$\templete.ewp</path>
  </project>
  <batchBuild/>
</workspace>


                                                                                                                                                                                                                                                                                                                                                              plusplus)
extern "C" {
#endif
  
/*******************************************************************************
* Includes
*******************************************************************************/
#include "GDFLIB_FilterExp_F32.h" 
#include "GDFLIB_FilterIIR1_F32.h"
#include "GDFLIB_FilterIIR1_F32_Asmi.h"
#include "GDFLIB_FilterIIR2_F32.h"
#include "GDFLIB_FilterIIR3_F32.h"
#include "GDFLIB_FilterIIR4_F32.h"
#include "GDFLIB_FilterMA_A32.h"
   
/*******************************************************************************
* Macros 
*******************************************************************************/
#define GDFLIB_FilterExpInit_F16(f16InX, psParam)                              \
        GDFLIB_FilterExpInit_F16_Ci(f16InX, psParam)             
#define GDFLIB_FilterExp_F16(f16InX, psParam)                                  \
        GDFLIB_FilterExp_F16_Ci(f16InX, psParam)    
#define GDFLIB_FilterIIR1Init_F16(psParam)                                     \
        GDFLIB_FilterIIR1Init_F16_Ci(psParam)                                  
#define GDFLIB_FilterIIR1_F16(f16InX, psParam)                                 \
        GDFLIB_FilterIIR1_F16_Asmi(f16InX, psParam)
#define GDFLIB_FilterIIR2Init_F16(psParam)                                     \
        GDFLIB_FilterIIR2Init_F16_C(psParam)                                  
#define GDFLIB_FilterIIR2_F16(f16InX, psParam)                                 \
        GDFLIB_FilterIIR2_F16_Asm(f16InX, psParam)
#define GDFLIB_FilterIIR3Init_F16(psParam)                                     \
        GDFLIB_FilterIIR3Init_F16_C(psParam)                                  
#define GDFLIB_FilterIIR3_F16(f16InX, psParam)                                 \
        GDFLIB_FilterIIR3_F16_C(f16InX, psParam)                              
#define GDFLIB_FilterIIR4Init_F16(psParam)                                     \
        GDFLIB_FilterIIR4Init_F16_C(psParam)                                  
#define GDFLIB_FilterIIR4_F16(f16InX, psParam)                                 \
        GDFLIB_FilterIIR4_F16_C(f16InX, psParam)             
#define GDFLIB_FilterMAInit_F16(f16InitVal, psParam)                           \
        GDFLIB_FilterMAInit_F16_Ci(f16InitVal, psParam)                        
#define GDFLIB_FilterMA_F16(f16InX, psParam)                                   \
        GDFLIB_FilterMA_F16_Ci(f16InX, psParam)         
            
#if defined(__cplusplus)
}
#endif

#endif /* _GDFLIB_H_ */
