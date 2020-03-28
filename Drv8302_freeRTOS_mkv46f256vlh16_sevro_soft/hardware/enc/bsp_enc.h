#ifndef __BSP_ENC_H__
#define __BSP_ENC_H__
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_enc.h"
#include "fsl_xbara.h"
#include "fsl_debug_console.h"

#define DEMO_ENC_BASEADDR   ENC

void ENC_PhaseAB_Init(void);


extern __IO uint32_t gCurPosValue; //g = 'global'








#endif 