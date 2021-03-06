.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\drivers\K\src\pit.c
.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\drivers\K\inc\pit.h
.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk60d10\flash\pit.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stdint.h
.\mk60d10\flash\pit.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stdbool.h
.\mk60d10\flash\pit.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stddef.h
.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK60D10.h
.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk60d10\flash\pit.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK60D10.h
                                                                                                                                    cK_1 = FRAC32(0.0);
        GFLIB_IntegratorInit_F16(0, &sPpMeasFcn->sSpeedIntegrator);
        sPpMeasFcn->ui16Active = TRUE;
        sPpMeasFcn->f16SpeedElRamp = FRAC16(0.0);
        sPpMeasFcn->ui16PpDetermined = 0;
    }

    /* Set Id required */
    *(sPpMeasFcn->pf16IdReq) = sPpMeasFcn->f16IdReqOpenLoop;

    /* Else start incrementing position */
    if(sPpMeasFcn->ui16WaitingSteady == 0)
    {
        /* Ramp electrical speed */
        sPpMeasFcn->f16SpeedElRamp = GFLIB_Ramp_F16(sPpMeasFcn->f16SpeedElReq, &sPpMeasFcn->sSpeedElRampParam);
        /* Integrate electrical speed to get electrical position */
        *sPpMeasFcn->pf16PosEl = GFLIB_Integrator_F16(sPpMeasFcn->f16SpeedElRamp, &sPpMeasFcn->sSpeedIntegrator);
    }

    /* If position overflows, wait 2400ms in zero position */
    if(((*sPpMeasFcn->pf16PosEl < FRAC16(0.0)) && (sPpMeasFcn->f16PosElLast > FRAC16(0.0))) || (sPpMeasFcn->ui16WaitingSteady == 1))
    {
        *sPpMeasFcn->pf16PosEl = FRAC16(-1.0);

        /* Initialise waiting */
        if(sPpMeasFcn->ui16WaitingSteady == 0)
        {
            sPpMeasFcn->ui16LoopCounter = 0;
            sPpMeasFcn->ui16WaitingSteady = 1;
        }

        sPpMeasFcn->ui16LoopCounter++;

        /* Escape waiting in steady position after 2400 ms */
        if(sPpMeasFcn->ui16LoopCounter > M1_TIME_2400MS)
        {
            *sPpMeasFcn->pf16PosEl   = FRAC16(0.0);
            sPpMeasFcn->f16PosElLast = FRAC16(0.0);
            sPpMeasFcn->ui16WaitingSteady = 0;
        }
    }

    /* Save last position */
    sPpMeasFcn->f16PosElLast = sPpMeasFcn->f16PosElCurrent;
    sPpMeasFcn->f16PosElCurrent = *sPpMeasFcn->pf16PosEl;

    if(sPpMeasFcn->ui16PpDetermined > 0)
    {
        /* When finished exit the function */
        sPpMeasFcn->ui16Active = FALSE;
    }
}