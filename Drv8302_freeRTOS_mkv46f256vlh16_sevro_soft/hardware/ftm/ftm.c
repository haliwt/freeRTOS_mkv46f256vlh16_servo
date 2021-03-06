#include "ftm.h"



static void FTM_BOARD_InitPins(void)
{
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
   

    /* PORTA12 (pin 42) is configured as FTM1_QD_PHA */
    PORT_SetPinMux(PORTA, 12U, kPORT_MuxAlt7);

    /* PORTA13 (pin 43) is configured as FTM1_QD_PHB */
    PORT_SetPinMux(PORTA, 13U, kPORT_MuxAlt7);

    
}
void FTM_PhaseAB_Init(void)
{
   
    ftm_config_t ftmInfo;
    ftm_phase_params_t phaseParamsConfigStruct;

	 FTM_BOARD_InitPins();
   /* Initialize FTM module */
    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = kFTM_Prescale_Divide_32;
    FTM_Init(DEMO_FTM_BASEADDR, &ftmInfo);

    /* Set the modulo values for Quad Decoder. */
    FTM_SetQuadDecoderModuloValue(DEMO_FTM_BASEADDR, 0U, DEMO_QUAD_DECODER_MODULO);

    /* Enable the Quad Decoder mode. */
    phaseParamsConfigStruct.enablePhaseFilter = true;
    phaseParamsConfigStruct.phaseFilterVal    = 0U;
    phaseParamsConfigStruct.phasePolarity     = kFTM_QuadPhaseNormal;
    FTM_SetupQuadDecode(DEMO_FTM_BASEADDR, &phaseParamsConfigStruct, /* Phase A. */
                        &phaseParamsConfigStruct,                    /* Phase B. */
                        kFTM_QuadPhaseEncode);

}
