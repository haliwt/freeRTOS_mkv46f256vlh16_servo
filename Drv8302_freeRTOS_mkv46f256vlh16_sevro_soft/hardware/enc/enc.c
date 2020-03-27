#include "enc.h"

static void ENC_BOARD_InitPins(void);

void ENC_PhaseAB_Init(void)
{
    enc_config_t mEncConfigStruct;
  
    XBARA_Init(XBARA);
    ENC_BOARD_InitPins();
   
   PRINTF("\r\nENC Basic Example.\r\n");

    /* Initialize the ENC module. */
    ENC_GetDefaultConfig(&mEncConfigStruct);
    ENC_Init(DEMO_ENC_BASEADDR, &mEncConfigStruct);
    ENC_DoSoftwareLoadInitialPositionValue(DEMO_ENC_BASEADDR); /* Update the position counter with initial value. */

    PRINTF("Press any key to get the encoder values ...\r\n");

}
static void ENC_BOARD_InitPins(void)
{
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);
   

    /* PORTC1 (pin 71) is configured as XBARIN11 */
    PORT_SetPinMux(PORTC, 1U, kPORT_MuxAlt6);  // PTC1 -> XB_IN11 -> XBARA_OUT44 -> ENC_PHA

    /* PORTC2 (pin 72) is configured as XBARIN6 */
    PORT_SetPinMux(PORTC, 2U, kPORT_MuxAlt6); // PTC2 -> XB_IN6  -> XBARA_OUT45 -> ENC_PHB

    /* PORTC6 (pin 78) is configured as XBARIN3 */
    PORT_SetPinMux(PORTC, 6U, kPORT_MuxAlt4); // PTC6 -> XB_IN3  -> XBARA_OUT46 -> ENC_INDEX
                                              //XBARA_OUT47 -> ENC_HOME
                                              //XBARA_OUT48 -> ENC_TRIGGER(Capture)

   
    /* XBARIN6 input pin output assigned to XBARA_IN6 input is connected
     * to XBARA_OUT44 output assigned to ENC0 quadrature waveform phase A */
    XBARA_SetSignalsConnection(XBARA, kXBARA_InputXbarIn6, kXBARA_OutputEnc0PhA);
    /* XBARIN11 input pin output assigned to XBARA_IN11 input is connected
     * to XBARA_OUT45 output assigned to ENC0 quadrature waveform phase B */
    XBARA_SetSignalsConnection(XBARA, kXBARA_InputXbarIn11, kXBARA_OutputEnc0PhB);
    /* XBARIN3 input pin output assigned to XBARA_IN3 input is connected
     * to XBARA_OUT46 output assigned to ENC0 refresh/reload */
    XBARA_SetSignalsConnection(XBARA, kXBARA_InputXbarIn3, kXBARA_OutputEnc0Index);
  
}