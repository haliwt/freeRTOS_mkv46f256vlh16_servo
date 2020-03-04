#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "adc.h"
#include "vref.h"

int main(void)
{
    DelayInit();
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);
    UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
    printf("VREF Test please connect ADC1_SE16 & VERF_OUT\r\n");
    /* 初始化 VREF 输出1.2V基准电压 */
    VREF_QuickInit();
    ADC_QuickInit(ADC1_SE16, kADC_SingleDiff12or13);
    uint32_t val;
    while(1)
    {
        val = ADC_QuickReadValue(ADC1_SE16);
        printf("ADC:%d | %0.3fV  \r", val, (double)val*3.300/(1<<12));
        GPIO_ToggleBit(HW_GPIOE, 6);
        DelayMs(500);
    }
}


                                                                                                                                                                                                                                                                                                                                                                                                       \
        AMCLIB_TrackObsrv_A32af_FC(a32ThetaErr, psCtrl)                                  
#define AMCLIB_TrackObsrvInit_A32af_Ci(a32ThetaInit, psCtrl)                   \
        AMCLIB_TrackObsrvInit_A32af_FCi(a32ThetaInit, psCtrl)
            
/*******************************************************************************
* Types
*******************************************************************************/          
typedef struct
{
    frac32_t f32Theta;   /* Estimated position */
    float_t fltSpeed;    /* Estimated speed - first integrator output */  
    float_t fltI_1;      /* State variable of observer controller part */
    float_t fltIGain;    /* Observer integral gain */
    float_t fltPGain;    /* Observer proportional gain */
    float_t fltThGain;   /* Observer gain for output integrator of position */
}AMCLIB_TRACK_OBSRV_T_FLT;
  
/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern acc32_t AMCLIB_TrackObsrv_A32af_FC(acc32_t a32ThetaErr, 
                                          AMCLIB_TRACK_OBSRV_T_FLT *psCtrl);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    Tracking observer initialization
*
* @param  in  - acc32_t a32ThetaInit  - init angle <-1;1) corresponds to <-pi;pi)
* @param  ptr - AMCLIB_TRACK_OBSRV_T_FLT *psCtrl
*               - frac32_t f32Theta - Estimated position 
*               - float_t fltSpeed  - Estimated speed 
*               - float_t fltI_1    - Internal integrator 
*               - float_t fltIGain  - Integ. constant to get speed from error 
*               - float_t fltPGain  - Prop. constant to get speed from angle 
*               - float_t fltThGain - Constant to get angle from speed 
*
* @return None
*                         
* @remarks   Initializes the structure of the tracking observer with an angle
*            according to following rules:   
*           
*   f32Theta = a32ThetaInit << 16 
*   fltSpeed = 0.0F
*   fltI_1   = 0.0F
*
****************************************************************************/
static inline void AMCLIB_TrackObsrvInit_A32af_FCi(acc32_t a32ThetaInit,
                                                   AMCLIB_TRACK_OBSRV_T_FLT *psCtrl)
{
    psCtrl -> f32Theta = ((frac32_t)((frac16_t)a32ThetaInit))<<16; 
    psCtrl -> fltSpeed = 0.0F;
    psCtrl -> fltI_1   = 0.0F;        
}
 
#if defined(__cplusplus) 
}
#endif

#endif  /* _AMCLIB_TRACK_OBSRV_FLT_H_ */
