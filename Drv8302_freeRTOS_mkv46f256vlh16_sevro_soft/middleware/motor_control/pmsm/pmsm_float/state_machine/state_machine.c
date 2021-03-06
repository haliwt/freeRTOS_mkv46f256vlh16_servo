.\mk60d10\flash\main.o: ..\src\main.c
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\drivers\K\inc\gpio.h
.\mk60d10\flash\main.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stdint.h
.\mk60d10\flash\main.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stdbool.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\drivers\K\inc\uart.h
.\mk60d10\flash\main.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stdio.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\devices\inc\ads7843.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\devices\inc\ili9320.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\drivers\K\inc\spi.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk60d10\flash\main.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stddef.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK60D10.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK60D10.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\utilities\emWin\Include\gui.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\utilities\emWin\Include\GUI_ConfDefaults.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\utilities\emWin\Include\GUIConf.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\utilities\emWin\Include\GUI_Type.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\utilities\emWin\Include\LCD.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\utilities\emWin\Include\Global.h
.\mk60d10\flash\main.o: ..\..\..\..\Libraries\utilities\emWin\Include\GUI_Version.h
                                                                                                                                                                                                                                                                 _FAULT_CLEAR) > 0)
    {
        /* Clear INIT_DONE, FAULT, FAULT_CLEAR flags */
        psAppCtrl->uiCtrl &= ~(SM_CTRL_INIT_DONE | SM_CTRL_FAULT | SM_CTRL_FAULT_CLEAR);

        /* User Fault to Init transition function */
        psAppCtrl->psTrans->FaultStop();

        /* Init state */
        psAppCtrl->eState = kSM_AppStop;
    }
}

/*!
 * @brief State machine Init state in fast
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
static void SM_StateInitFast(sm_app_ctrl_t *psAppCtrl)
{
    /* User Init function */
    psAppCtrl->psStateFast->Init();

    /* if fault flag */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
    {
        /* User Init to Fault transition function */
        psAppCtrl->psTrans->InitFault();

        /* Fault state */
        psAppCtrl->eState = kSM_AppFault;
    }
    /* if INIT_DONE flag */
    else if ((psAppCtrl->uiCtrl & SM_CTRL_INIT_DONE) > 0)
    {
        /* Clear INIT_DONE, START_STOP, OM_CHANGE, STOP_ACK, RUN_ACK flags */
        psAppCtrl->uiCtrl &= ~(SM_CTRL_INIT_DONE | SM_CTRL_STOP | SM_CTRL_START | SM_CTRL_STOP_ACK | SM_CTRL_RUN_ACK);

        /* User Init to Stop transition function */
        psAppCtrl->psTrans->InitStop();

        /* Stop state */
        psAppCtrl->eState = kSM_AppStop;
    }
}

/*!
 * @brief State machine Stop state in fast
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
static void SM_StateStopFast(sm_app_ctrl_t *psAppCtrl)
{
    /* User Stop function */
    psAppCtrl->psStateFast->Stop();

    /* if fault */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
    {
        /* User Stop to Fault transition function */
        psAppCtrl->psTrans->StopFault();

        /* Fault state */
        psAppCtrl->eState = kSM_AppFault;
    }
    else if ((psAppCtrl->uiCtrl & SM_CTRL_START) > 0)
    {
        /* User Stop to Run transition function, user must set up the SM_CTRL_RUN_ACK
        flag to allow the RUN state */
        psAppCtrl->psTrans->StopRun();

        /* Clears the START command */
        if ((psAppCtrl->uiCtrl & SM_CTRL_RUN_ACK) > 0)
        {
            /* Clears the RUN_ACK flag */
            psAppCtrl->uiCtrl &= ~(SM_CTRL_RUN_ACK | SM_CTRL_START);

            /* Run state */
            psAppCtrl->eState = kSM_AppRun;
        }
    }
}

/*!
 * @brief State machine Run state in fast
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
static void SM_StateRunFast(sm_app_ctrl_t *psAppCtrl)
{
    /* User Run function */
    psAppCtrl->psStateFast->Run();

    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
    {
        /* User Run to Fault transition function */
        psAppCtrl->psTrans->RunFault();

        /* Fault state */
        psAppCtrl->eState = kSM_AppFault;
    }
    else if ((psAppCtrl->uiCtrl & SM_CTRL_STOP) > 0)
    {
        /* User Run to Stop transition function, user must set up the SM_CTRL_STOP_ACK
        flag to allow the STOP state */
        psAppCtrl->psTrans->RunStop();

        if ((psAppCtrl->uiCtrl & SM_CTRL_STOP_ACK) > 0)
        {
            /* Clears the STOP_ACK flag */
            psAppCtrl->uiCtrl &= ~(SM_CTRL_STOP_ACK | SM_CTRL_STOP);

            /* Run state */
            psAppCtrl->eState = kSM_AppStop;
        }
    }
}

/*!
 * @brief State machine Fault state in slow
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
static void SM_StateFaultSlow(sm_app_ctrl_t *psAppCtrl)
{
    psAppCtrl->psStateSlow->Fault();
}

/*!
 * @brief State machine Init state in slow
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
static void SM_StateInitSlow(sm_app_ctrl_t *psAppCtrl)
{
    psAppCtrl->psStateSlow->Init();
}

/*!
 * @brief State machine Stop state in slow
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
static void SM_StateStopSlow(sm_app_ctrl_t *psAppCtrl)
{
    psAppCtrl->psStateSlow->Stop();
}

/*!
 * @brief State machine Run state in slow
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
static void SM_StateRunSlow(sm_app_ctrl_t *psAppCtrl)
{
    psAppCtrl->psStateSlow->Run();
}

