#include "gpio.h"
#include "common.h"
#include "uart.h"

/* CH Kinetis固件库 V2.50 版本 */
/* 修改主频 请使用 CMSIS标准文件 startup_MKxxxx.c 中的 CLOCK_SETUP 宏 */

/* UART 快速初始化结构所支持的引脚* 使用时还是推荐标准初始化 */
/*
 UART1_RX_PE01_TX_PE00   
 UART0_RX_PF17_TX_PF18   
 UART3_RX_PE05_TX_PE04   
 UART5_RX_PF19_TX_PF20   
 UART5_RX_PE09_TX_PE08   
 UART2_RX_PE17_TX_PE16   
 UART4_RX_PE25_TX_PE24   
 UART0_RX_PA01_TX_PA02   
 UART0_RX_PA15_TX_PA14   
 UART3_RX_PB10_TX_PB11   
 UART0_RX_PB16_TX_PB17   
 UART1_RX_PC03_TX_PC04   
 UART4_RX_PC14_TX_PC15   
 UART3_RX_PC16_TX_PC17   
 UART2_RX_PD02_TX_PD03   
 UART0_RX_PD06_TX_PD07   
 UART2_RX_PF13_TX_PF14   
 UART5_RX_PD08_TX_PD09   
*/

 /*
     实验名称：UART串口
     实验平台：渡鸦开发板
     板载芯片：MK60DN512ZVQ10
 实验效果：使用开发板上的UART接口和电脑通信，
        开发板每隔500ms将数据发送到电脑的串口调试助手上
        同时开发板上还伴随着小灯的闪烁。
        串口的通信速度为115200 8位有效数据位
    注意：需要使用USB数据线连接到电脑端和开发板的UART接口
         电脑端需要安装CH340驱动和串口调试助手
*/

int main(void)
{
    uint32_t instance; /*存放 UART 的模块号 */
    DelayInit();
    DelayMs(10);
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);
    
    /* 初始化UART 使用快速初始化方式 波特率 115200 其他配置默认 返回初始化后 UART的模块号 */
    instance = UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
    
    /* 当使用串口初始化后 printf 被默认连接到第一个被初始化的串口上*/
    printf("UART%d OK! Hello Kinetis\r\n", instance);
    
    while(1)
    {
        /* 串口 按字节发送 数据 注意 HW_UART0必须是已经初始化过的模块 否则 将产生错误*/
        UART_WriteByte(instance, 'h');
        UART_WriteByte(instance, 'e');
        UART_WriteByte(instance, 'l');
        UART_WriteByte(instance, 'l');
        UART_WriteByte(instance, 'o');
        UART_WriteByte(instance, '\r');
        UART_WriteByte(instance, '\n');
        /* 闪烁小灯 */
        GPIO_ToggleBit(HW_GPIOE, 6);
        DelayMs(500);
    }
}


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           psParam->fltFltBfrY[0] = (float_t)0.0F;       
    psParam->fltFltBfrX[0] = (float_t)0.0F; 
}
 
/***************************************************************************//*!
*
* @brief  The function clears the buffer of 2nd order float IIR filter 
*
* @param  ptr   GDFLIB_FILTER_IIR1_T_FLT *psParam
*               - pointer to the structure of the filter
*
*                 GDFLIB_FILTER_IIR1_T_FLT data structure:
*                 GDFLIB_FILTER_IIR1_COEFF_T_FLT  sFltCoeff   
*                 - filter coefficients in float_t as real value
*                   - B coefficients:
*                       float_t  fltB0
*                       float_t  fltB1
*                   - A coefficient, the sign is negated
*                       float_t  fltA1
*                   float_t fltFltBfrY[1] - y history buffer in float_t
*                   float_t fltFltBfrX[1] - x history buffer in float_t
* @param  in    float_t fltInX - Input signal at the step K step
*
* @return   This function filtered value - float_t value 
*
* @remarks      Filter algorithm:                                           
*                                                                       
*             -----------                 
*      x(k)  |           |  y(k)           
*    --------|    IIR1   |---------        
*            |           |                 
*             -----------                  
*                                                            
*    x(k) - input signal                    
*    y(k) - filtered output signal              
*                                                                    
*    Filter equation:
*    y(n) = b0*x(n) + b1*x(n-1) - a1*y(n-1)
*                                                                        
*    All B coefficients are not scaled, i.e. if the desired b1 coefficient
*    is 1.756 the value is then:  b1 = 1.786F ;
*    
*    All A coefficients must be negated but are not scaled, i.e. if the desired a2 coefficient 
*    is 3.789, the value is then: a2 = -3.789F ; 
*   
*******************************************************************************/
static inline float_t GDFLIB_FilterIIR1_FLT_FCi(float_t fltInX,
                                                GDFLIB_FILTER_IIR1_T_FLT *psParam)
{
    register float_t fltAcc;
    
    /*
    * Implemented equation assuming real (not devided) filter coeficients:
    * y(k) = b0*x(k) + b1*x(k-1) + a1*y(k-1)
    *
    * Calculation steps:
    * Acc =       b0 * x(k)
    * Acc = Acc + b1 * x(k-1)     
    * Acc = Acc + a1 * y(k-1)    
    * y(k)= Acc 
    */
    
    /* Acc =  b0 * x(k) */  
    fltAcc = MLIB_Mul_FLT ( psParam->sFltCoeff.fltB0, fltInX);             
    
    /* Acc = Acc + b1 * x(k-1) */ 
    fltAcc = MLIB_Mac_FLT (fltAcc, psParam->sFltCoeff.fltB1, psParam->fltFltBfrX[0]); 
    
    /* Acc = Acc + a1 * y(k-1) */   
    fltAcc = MLIB_Mac_FLT (fltAcc, psParam->sFltCoeff.fltA1, psParam->fltFltBfrY[0]);  
                     
    /* Storing filter states in the buffers */
    psParam->fltFltBfrX[0] = fltInX;
    psParam->fltFltBfrY[0] = fltAcc;
        
    /* Returning float value of internal accumulator */
    return(fltAcc);    
}
 
#if defined(__cplusplus)
}
#endif

#endif /* _GDFLIB_FILTERIIR1_FLT_H_ */
