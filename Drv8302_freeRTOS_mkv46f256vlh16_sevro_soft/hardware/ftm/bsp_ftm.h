#ifndef __BSP_FTM_H__
#define __BSP_FTM_H__
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_port.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* The Flextimer instance/channel used for board */
#define DEMO_FTM_BASEADDR       FTM1

/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

#define DEMO_QUAD_DECODER_MODULO 65535U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/




void FTM_PhaseAB_Init(void);



#endif 
