MA_CITER_ELINKYES_LINKCH(x) (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_LINKCH_SHIFT))&DMA_CITER_ELINKYES_LINKCH_MASK) �DMA_CITER_ELINKYES_ELINK_MASK 0x8000u �DMA_CITER_ELINKYES_ELINK_SHIFT 15 �DMA_DLAST_SGA_DLASTSGA_MASK 0xFFFFFFFFu �DMA_DLAST_SGA_DLASTSGA_SHIFT 0 �DMA_DLAST_SGA_DLASTSGA(x) (((uint32_t)(((uint32_t)(x))<<DMA_DLAST_SGA_DLASTSGA_SHIFT))&DMA_DLAST_SGA_DLASTSGA_MASK) �DMA_CSR_START_MASK 0x1u �DMA_CSR_START_SHIFT 0 �DMA_CSR_INTMAJOR_MASK 0x2u �DMA_CSR_INTMAJOR_SHIFT 1 �DMA_CSR_INTHALF_MASK 0x4u �DMA_CSR_INTHALF_SHIFT 2 �DMA_CSR_DREQ_MASK 0x8u �DMA_CSR_DREQ_SHIFT 3 �DMA_CSR_ESG_MASK 0x10u �DMA_CSR_ESG_SHIFT 4 �DMA_CSR_MAJORELINK_MASK 0x20u �DMA_CSR_MAJORELINK_SHIFT 5 �DMA_CSR_ACTIVE_MASK 0x40u �DMA_CSR_ACTIVE_SHIFT 6 �DMA_CSR_DONE_MASK 0x80u �DMA_CSR_DONE_SHIFT 7 �DMA_CSR_MAJORLINKCH_MASK 0xF00u �DMA_CSR_MAJORLINKCH_SHIFT 8 �DMA_CSR_MAJORLINKCH(x) (((uint16_t)(((uint16_t)(x))<<DMA_CSR_MAJORLINKCH_SHIFT))&DMA_CSR_MAJORLINKCH_MASK) �DMA_CSR_BWC_MASK 0xC000u �DMA_CSR_BWC_SHIFT 14 �DMA_CSR_BWC(x) (((uint16_t)(((uint16_t)(x))<<DMA_CSR_BWC_SHIFT))&DMA_CSR_BWC_MASK) �DMA_BITER_ELINKNO_BITER_MASK 0x7FFFu �DMA_BITER_ELINKNO_BITER_SHIFT 0 �DMA_BITER_ELINKNO_BITER(x) (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKNO_BITER_SHIFT))&DMA_BITER_ELINKNO_BITER_MASK) �DMA_BITER_ELINKNO_ELINK_MASK 0x8000u �DMA_BITER_ELINKNO_ELINK_SHIFT 15 �DMA_BITER_ELINKYES_BITER_MASK 0x1FFu �DMA_BITER_ELINKYES_BITER_SHIFT 0 �DMA_BITER_ELINKYES_BITER(x) (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_BITER_SHIFT))&DMA_BITER_ELINKYES_BITER_MASK) �DMA_BITER_ELINKYES_LINKCH_MASK 0x1E00u �DMA_BITER_ELINKYES_LINKCH_SHIFT 9 �DMA_BITER_ELINKYES_LINKCH(x) (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_LINKCH_SHIFT))&DMA_BITER_ELINKYES_LINKCH_MASK) �DMA_BITER_ELINKYES_ELINK_MASK 0x8000u �DMA_BITER_ELINKYES_ELINK_SHIFT 15 �DMA_BASE (0x40008000u) �DMA0 ((DMA_Type *)DMA_BASE) �DMA_BASE_PTR (DMA0) �DMA_BASES { DMA0 } �DMA_CR DMA_CR_REG(DMA0) �DMA_ES DMA_ES_REG(DMA0) �DMA_ERQ DMA_ERQ_REG(DMA0) �DMA_EEI DMA_EEI_REG(DMA0) �DMA_CEEI DMA_CEEI_REG(DMA0) �DMA_SEEI DMA_SEEI_REG(DMA0) �DMA_CERQ DMA_CERQ_REG(DMA0) �DMA_SERQ DMA_SERQ_REG(DMA0) �DMA_CDNE DMA_CDNE_REG(DMA0) �DMA_SSRT DMA_SSRT_REG(DMA0) �DMA_CERR DMA_CERR_REG(DMA0) �DMA_CINT DMA_CINT_REG(DMA0) �DMA_INT DMA_INT_REG(DMA0) � DMA_ERR DMA_ERR_REG(DMA0) � DMA_HRS DMA_HRS_REG(DMA0) � DMA_DCHPRI3 DMA_DCHPRI3_REG(DMA0) � DMA_DCHPRI2 DMA_DCHPRI2_REG(DMA0) � DMA_DCHPRI1 DMA_DCHPRI1_REG(DMA0) � DMA_DCHPRI0 DMA_DCHPRI0_REG(DMA0) � DMA_DCHPRI7 DMA_DCHPRI7_REG(DMA0) � DMA_DCHPRI6 DMA_DCHPRI6_REG(DMA0) � DMA_DCHPRI5 DMA_DCHPRI5_REG(DMA0) � DMA_DCHPRI4 DMA_DCHPRI4_REG(DMA0) � DMA_DCHPRI11 DMA_DCHPRI11_REG(DMA0) � DMA_DCHPRI10 DMA_DCHPRI10_REG(DMA0) � DMA_DCHPRI9 DMA_DCHPRI9_REG(DMA0) � DMA_DCHPRI8 DMA_DCHPRI8_REG(DMA0) � DMA_DCHPRI15 DMA_DCHPRI15_REG(DMA0) � DMA_DCHPRI14 DMA_DCHPRI14_REG(DMA0) � DMA_DCHPRI13 DMA_DCHPRI13_REG(DMA0) � DMA_DCHPRI12 DMA_DCHPRI12_REG(DMA0) � DMA_TCD0_SADDR DMA_SADDR_REG(DMA0,0) � DMA_TCD0_SOFF DMA_SOFF_REG(DMA0,0) � DMA_TCD0_ATTR DMA_ATTR_REG(DMA0,0) � DMA_TCD0_NBYTES_MLNO DMA_NBYTES_MLNO_REG(DMA0,0) � DMA_TCD0_NBYTES_MLOFFNO DMA_NBYTES_MLOFFNO_REG(DMA0,0) � DMA_TCD0_NBYTES_MLOFFYES DMA_NBYTES_MLOFFYES_REG(DMA0,0) � DMA_TCD0_SLAST DMA_SLAST_REG(DMA0,0) � DMA_TCD0_DADDR DMA_DADDR_REG(DMA0,0) � DMA_TCD0_DOFF DMA_DOFF_REG(DMA0,0) � DMA_TCD0_CITER_ELINKNO DMA_CITER_ELINKNO_REG(DMA0,0) � DMA_TCD0_CITER_ELINKYES DMA_CITER_ELINKYES_REG(DMA0,0) � DMA_TCD0_DLASTSGA DMA_DLAST_SGA_REG(DMA0,0) � DMA_TCD0_CSR DMA_CSR_REG(DMA0,0) � DMA_TCD0_BITER_ELINKNO DMA_BITER_ELINKNO_REG(DMA0,0) � DMA_TCD0_BITER_ELINKYES DMA_BITER_ELINKYES_REG(DMA0,0) � DMA_TCD1_SADDR DMA_SADDR_REG(DMA0,1) � DMA_TCD1_SOFF DMA_SOFF_REG(DMA0,1) � DMA_TCD1_ATTR DMA_ATTR_REG(DMA0,1) � DMA_TCD1_NBYTES_MLNO DMA_NBYTES_MLNO_REG(DMA0,1) � DMA_TCD1_NBYTES_MLOFFNO DMA_NBYTES_MLOFFNO_REG(DMA0,1) � DMA_TCD1_NBYTES_MLOFFYES DMA_NBYTES_MLOFFYES_REG(DMA0,1) � DMA_TCD1_SLAST DMA_SLAST_REG(DMA0,1) � DMA_TCD1_DADDR DMA_DADDR_REG(DMA0,1) � DMA_TCD1_DOFF DMA_DOFF_REG(DMA0,1) � DMA_TCD1_CITER_ELINKNO DMA_CITER_ELINKNO_REG(DMA0,1) � DMA_TCD1_CITER_ELINKYES DMA_CITER_ELINKYES_REG(DMA0,1) � DMA_TCD1_DLASTSGA DMA_DLAST_SGA_REG(DMA0,1) � DMA_TCD1_CSR DMA_CSR_REG(DMA0,1) � DMA_TCD1_BITER_ELINKNO DMA_BITER_ELINKNO_REG(DMA0,1) � DMA_TCD1_BITER_ELINKYES DMA_BITER_ELINKYES_REG(DMA0,1) � DMA_TCD2_SADDR DMA_SADDR_REG(DMA0,2) � DMA_TCD2_SOFF DMA_SOFF_REG(DMA0,2) � DMA_TCD2_ATTR DMA_ATTR_REG(DMA0,2) � DMA_TCD2_NBYTES_MLNO DMA_NBYTES_MLNO_REG(DMA0,2) � DMA_TCD2_NBYTES_MLOFFNO DMA_NBYTES_MLOFFNO_REG(DMA0,2) � DMA_TCD2_NBYTES_MLOFFYES DMA_NBYTES_MLOFFYES_REG(DMA0,2) � DMA_TCD2_SLAST DMA_SLAST_REG(DMA0,2) � DMA_TCD2_DADDR DMA_DADDR_REG(DMA0,2) � DMA_TCD2_DOFF DMA_DOFF_REG(DMA0,2) � DMA_TCD2_CITER_ELINKNO DMA_CITER_ELINKNO_REG(DMA0,2) � DMA_TCD2_CITER_ELINKYES DMA_CITER_ELINKYES_REG(DMA0,2) � DMA_TCD2_DLASTSGA DMA_DLAST_SGA_REG(DMA0,2) � DMA_TCD2_CSR DMA_CSR_REG(DMA0,2) � DMA_TCD2_BITER_ELINKNO DMA_BITER_ELINKNO_REG(DMA0,2) � DMA_TCD2_BITER_ELINKYES DMA_BITER_ELINKYES_REG(DMA0,2) � DMA_TCD3_SADDR DMA_SADDR_REG(DMA0,3) � DMA_TCD3_SOFF DMA_SOFF_REG(DMA0,3) � DMA_TCD3_ATTR DMA_ATTR_REG(DMA0,3) � DMA_TCD3_NBYTES_MLNO DMA_NBYTES_MLNO_REG(DMA0,3) � DMA_TCD3_NBYTES_MLOFFNO DMA_NBYTES_MLOFFNO_REG(DMA0,3) � DMA_TCD3_NBYTES_MLOFFYES DMA_NBYTES_MLOFFYES_REG(DMA0,3) � DMA_TCD3_SLAST DMA_SLAST_REG(DMA0,3) � DMA_TCD3_DADDR DMA_DADDR_REG(DMA0,3) � DMA_TCD3_DOFF DMA_DOFF_REG(DMA0,3) � DMA_TCD3_CITER_ELINKNO DMA_CITER_ELINKNO_REG(DMA0,3) � DMA_TCD3_CITER_ELINKYES DMA_CITER_ELINKYES_REG(DMA0,3) � DMA_TCD3_DLASTSGA DMA_DLAST_SGA_REG(DMA0,3) � DMA_TCD3_CSR DMA_CSR_REG(DMA0,3) � DMA_TCD3_BITER_ELINKNO DMA_BITER_ELINKNO_REG(DMA0,3) � DMA_TCD3_BITER_ELINKYES DMA_BITER_ELINKYES_REG(DMA0,3) � DMA_TCD4_SADDR DMA_SADDR_REG(DMA0,4) � DMA_TCD4_SOFF DMA_SOFF_REG(DMA0,4) � DMA_TCD4_ATTR DMA_ATTR_REG(DMA0,4) � DMA_TCD4_NBYTES_MLNO DMA_NBYTES_MLNO_REG(DMA0,4) � DMA_TCD4_NBYTES_MLOFFNO DMA_NBYTES_MLOFFNO_REG(DMA0,4) � DMA_TCD4_NBYTES_MLOFFYES DMA_NBYTES_MLOFFYES_REG(DMA0,4) � DMA_TCD4_SLAST DMA_SLAST_REG(DMA0,4) � DMA_TCD4_DADDR DMA_DADDR_REG(DMA0,4) � DMA_TCD4_DOFF DMA_DOFF_REG(DMA0,4) � DMA_TCD4_CITER_ELINKNO DMA_CITER_ELINKNO_REG(DMA0,4) � DMA_TCD4_CITER_ELINKYES DMA_CITER_ELINKYES_REG(DMA0,4) � DMA_TCD4_DLASTSGA DMA_DLAST_SGA_REG(DMA0,4) � DMA_TCD4_CSR DMA_CSR_REG(DMA0,4) � DMA_TCD4_BITER_ELINKNO DMA_BITER_ELINKNO_REG(DMA0,4) � DMA_TCD4_BITER_ELINKYES DMA_BITER_ELINKYES_REG(DMA0,4) � DMA_TCD5_SADDR DMA_SADDR_REG(DMA0,5) � DMA_TCD5_SOFF DMA_SOFF_REG(DMA0,5) � DMA_TCD5_ATTR DMA_ATTR_REG(DMA0,5) � DMA_TCD5_NBYTES_MLNO DMA_NBYTES_MLNO_REG(DMA0,5) � DMA_TCD5_NBYTES_MLOFFNO DMA_NBYTES_MLOFFNO_REG(DMA0,5) � DMA_TCD5_NBYTES_MLOFFYES DMA_NBYTES_MLOFFYES_REG(DMA0,5) � DMA_TCD5_SLAST DMA_SLAST_REG(DMA0,5) � DMA_TCD5_DADDR DMA_DADDR_REG(DMA0,5) � DMA_TCD5_DOFF DMA_DOFF_REG(DMA0,5) � DMA_TCD5_CITER_ELINKNO DMA_CITER_ELINKNO_REG(DMA0,5) � DMA_TCD5_CITER_ELINKYES DMA_CITER_ELINKYES_REG(DMA0,5) � DMA_TCD5_DLASTSGA DMA_DLAST_SGA_REG(DMA0,5) � DMA_TCD5_CSR DMA_CSR_REG(DMA0,5) � DMA_TCD5_BITER_ELINKNO DMA_BITER