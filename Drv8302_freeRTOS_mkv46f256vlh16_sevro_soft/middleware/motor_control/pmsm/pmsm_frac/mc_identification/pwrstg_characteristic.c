C_CLKS_MASK) � FTM_SC_CPWMS_MASK 0x20u � FTM_SC_CPWMS_SHIFT 5 � FTM_SC_TOIE_MASK 0x40u � FTM_SC_TOIE_SHIFT 6 � FTM_SC_TOF_MASK 0x80u � FTM_SC_TOF_SHIFT 7 � FTM_CNT_COUNT_MASK 0xFFFFu � FTM_CNT_COUNT_SHIFT 0 � FTM_CNT_COUNT(x) (((uint32_t)(((uint32_t)(x))<<FTM_CNT_COUNT_SHIFT))&FTM_CNT_COUNT_MASK) � FTM_MOD_MOD_MASK 0xFFFFu � FTM_MOD_MOD_SHIFT 0 � FTM_MOD_MOD(x) (((uint32_t)(((uint32_t)(x))<<FTM_MOD_MOD_SHIFT))&FTM_MOD_MOD_MASK) � FTM_CnSC_DMA_MASK 0x1u � FTM_CnSC_DMA_SHIFT 0 � FTM_CnSC_ELSA_MASK 0x4u � FTM_CnSC_ELSA_SHIFT 2 � FTM_CnSC_ELSB_MASK 0x8u � FTM_CnSC_ELSB_SHIFT 3 � FTM_CnSC_MSA_MASK 0x10u � FTM_CnSC_MSA_SHIFT 4 � FTM_CnSC_MSB_MASK 0x20u � FTM_CnSC_MSB_SHIFT 5 � FTM_CnSC_CHIE_MASK 0x40u � FTM_CnSC_CHIE_SHIFT 6 � FTM_CnSC_CHF_MASK 0x80u � FTM_CnSC_CHF_SHIFT 7 � FTM_CnV_VAL_MASK 0xFFFFu � FTM_CnV_VAL_SHIFT 0 � FTM_CnV_VAL(x) (((uint32_t)(((uint32_t)(x))<<FTM_CnV_VAL_SHIFT))&FTM_CnV_VAL_MASK) � FTM_CNTIN_INIT_MASK 0xFFFFu � FTM_CNTIN_INIT_SHIFT 0 � FTM_CNTIN_INIT(x) (((uint32_t)(((uint32_t)(x))<<FTM_CNTIN_INIT_SHIFT))&FTM_CNTIN_INIT_MASK) � FTM_STATUS_CH0F_MASK 0x1u � FTM_STATUS_CH0F_SHIFT 0 � FTM_STATUS_CH1F_MASK 0x2u � FTM_STATUS_CH1F_SHIFT 1 � FTM_STATUS_CH2F_MASK 0x4u � FTM_STATUS_CH2F_SHIFT 2 � FTM_STATUS_CH3F_MASK 0x8u � FTM_STATUS_CH3F_SHIFT 3 � FTM_STATUS_CH4F_MASK 0x10u � FTM_STATUS_CH4F_SHIFT 4 � FTM_STATUS_CH5F_MASK 0x20u � FTM_STATUS_CH5F_SHIFT 5 � FTM_STATUS_CH6F_MASK 0x40u � FTM_STATUS_CH6F_SHIFT 6 �!FTM_STATUS_CH7F_MASK 0x80u �!FTM_STATUS_CH7F_SHIFT 7 �!FTM_MODE_FTMEN_MASK 0x1u �!FTM_MODE_FTMEN_SHIFT 0 �!FTM_MODE_INIT_MASK 0x2u �!FTM_MODE_INIT_SHIFT 1 �!FTM_MODE_WPDIS_MASK 0x4u �!FTM_MODE_WPDIS_SHIFT 2 �!FTM_MODE_PWMSYNC_MASK 0x8u �!FTM_MODE_PWMSYNC_SHIFT 3 �!FTM_MODE_CAPTEST_MASK 0x10u �!FTM_MODE_CAPTEST_SHIFT 4 �!FTM_MODE_FAULTM_MASK 0x60u �!FTM_MODE_FAULTM_SHIFT 5 �!FTM_MODE_FAULTM(x) (((uint32_t)(((uint32_t)(x))<<FTM_MODE_FAULTM_SHIFT))&FTM_MODE_FAULTM_MASK) �!FTM_MODE_FAULTIE_MASK 0x80u �!FTM_MODE_FAULTIE_SHIFT 7 �!FTM_SYNC_CNTMIN_MASK 0x1u �!FTM_SYNC_CNTMIN_SHIFT 0 �!FTM_SYNC_CNTMAX_MASK 0x2u �!FTM_SYNC_CNTMAX_SHIFT 1 �!FTM_SYNC_REINIT_MASK 0x4u �!FTM_SYNC_REINIT_SHIFT 2 �!FTM_SYNC_SYNCHOM_MASK 0x8u �!FTM_SYNC_SYNCHOM_SHIFT 3 �!FTM_SYNC_TRIG0_MASK 0x10u �!FTM_SYNC_TRIG0_SHIFT 4 �!FTM_SYNC_TRIG1_MASK 0x20u �!FTM_SYNC_TRIG1_SHIFT 5 �!FTM_SYNC_TRIG2_MASK 0x40u �!FTM_SYNC_TRIG2_SHIFT 6 �!FTM_SYNC_SWSYNC_MASK 0x80u �!FTM_SYNC_SWSYNC_SHIFT 7 �!FTM_OUTINIT_CH0OI_MASK 0x1u �!FTM_OUTINIT_CH0OI_SHIFT 0 �!FTM_OUTINIT_CH1OI_MASK 0x2u �!FTM_OUTINIT_CH1OI_SHIFT 1 �!FTM_OUTINIT_CH2OI_MASK 0x4u �!FTM_OUTINIT_CH2OI_SHIFT 2 �!FTM_OUTINIT_CH3OI_MASK 0x8u �!FTM_OUTINIT_CH3OI_SHIFT 3 �!FTM_OUTINIT_CH4OI_MASK 0x10u �!FTM_OUTINIT_CH4OI_SHIFT 4 �!FTM_OUTINIT_CH5OI_MASK 0x20u �!FTM_OUTINIT_CH5OI_SHIFT 5 �!FTM_OUTINIT_CH6OI_MASK 0x40u �!FTM_OUTINIT_CH6OI_SHIFT 6 �!FTM_OUTINIT_CH7OI_MASK 0x80u �!FTM_OUTINIT_CH7OI_SHIFT 7 �!FTM_OUTMASK_CH0OM_MASK 0x1u �!FTM_OUTMASK_CH0OM_SHIFT 0 �!FTM_OUTMASK_CH1OM_MASK 0x2u �!FTM_OUTMASK_CH1OM_SHIFT 1 �!FTM_OUTMASK_CH2OM_MASK 0x4u �!FTM_OUTMASK_CH2OM_SHIFT 2 �!FTM_OUTMASK_CH3OM_MASK 0x8u �!FTM_OUTMASK_CH3OM_SHIFT 3 �!FTM_OUTMASK_CH4OM_MASK 0x10u �!FTM_OUTMASK_CH4OM_SHIFT 4 �!FTM_OUTMASK_CH5OM_MASK 0x20u �!FTM_OUTMASK_CH5OM_SHIFT 5 �!FTM_OUTMASK_CH6OM_MASK 0x40u �!FTM_OUTMASK_CH6OM_SHIFT 6 �!FTM_OUTMASK_CH7OM_MASK 0x80u �!FTM_OUTMASK_CH7OM_SHIFT 7 �!FTM_COMBINE_COMBINE0_MASK 0x1u �!FTM_COMBINE_COMBINE0_SHIFT 0 �!FTM_COMBINE_COMP0_MASK 0x2u �!FTM_COMBINE_COMP0_SHIFT 1 �!FTM_COMBINE_DECAPEN0_MASK 0x4u �!FTM_COMBINE_DECAPEN0_SHIFT 2 �!FTM_COMBINE_DECAP0_MASK 0x8u �!FTM_COMBINE_DECAP0_SHIFT 3 �!FTM_COMBINE_DTEN0_MASK 0x10u �!FTM_COMBINE_DTEN0_SHIFT 4 �!FTM_COMBINE_SYNCEN0_MASK 0x20u �!FTM_COMBINE_SYNCEN0_SHIFT 5 �!FTM_COMBINE_FAULTEN0_MASK 0x40u �!FTM_COMBINE_FAULTEN0_SHIFT 6 �!FTM_COMBINE_COMBINE1_MASK 0x100u �!FTM_COMBINE_COMBINE1_SHIFT 8 �!FTM_COMBINE_COMP1_MASK 0x200u �!FTM_COMBINE_COMP1_SHIFT 9 �!FTM_COMBINE_DECAPEN1_MASK 0x400u �!FTM_COMBINE_DECAPEN1_SHIFT 10 �!FTM_COMBINE_DECAP1_MASK 0x800u �!FTM_COMBINE_DECAP1_SHIFT 11 �!FTM_COMBINE_DTEN1_MASK 0x1000u �!FTM_COMBINE_DTEN1_SHIFT 12 �!FTM_COMBINE_SYNCEN1_MASK 0x2000u �!FTM_COMBINE_SYNCEN1_SHIFT 13 �!FTM_COMBINE_FAULTEN1_MASK 0x4000u �!FTM_COMBINE_FAULTEN1_SHIFT 14 �!FTM_COMBINE_COMBINE2_MASK 0x10000u �!FTM_COMBINE_COMBINE2_SHIFT 16 �!FTM_COMBINE_COMP2_