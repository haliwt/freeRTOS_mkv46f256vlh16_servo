.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\devices\src\nrf24l01.c
.\mk60d10\flash\nrf24l01.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\string.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\devices\inc\nrf24l01.h
.\mk60d10\flash\nrf24l01.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stdint.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\drivers\K\inc\spi.h
.\mk60d10\flash\nrf24l01.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stdbool.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\drivers\K\inc\common.h
.\mk60d10\flash\nrf24l01.o: C:\Keil_v5\ARM\ARMCC\bin\..\include\stddef.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\startup\DeviceSupport\MK60D10.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmInstr.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cmFunc.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\startup\CoreSupport\core_cm4_simd.h
.\mk60d10\flash\nrf24l01.o: ..\..\..\..\Libraries\startup\DeviceSupport\system_MK60D10.h
                                                                                                                                                                                                                                                                                                                                                                                                                                                 *
* @return   As all command handlers, the return value should be the buffer 
*           pointer where the response output finished (except checksum)
*
******************************************************************************/

FMSTR_BPTR FMSTR_SetUpScope(FMSTR_BPTR pMessageIO)
{
    FMSTR_BPTR pResponse = pMessageIO;
    FMSTR_U8 i, sz, nVarCnt;

    /* uninitialize scope */
    pcm_nScopeVarCount = 0U;

    /* seek the setup data */
    pMessageIO = FMSTR_SkipInBuffer(pMessageIO, 2U);
    
    /* scope variable count  */
    pMessageIO = FMSTR_ValueFromBuffer8(&nVarCnt, pMessageIO);

    /* scope variable information must fit into our buffers */
    if(!nVarCnt || nVarCnt > (FMSTR_U8)FMSTR_MAX_SCOPE_VARS)
    {
        return FMSTR_ConstToBuffer8(pResponse, FMSTR_STC_INVBUFF);
    }
    
    /* get all addresses and sizes */
    for(i=0U; i<nVarCnt; i++)
    {
        /* variable size */
        pMessageIO = FMSTR_ValueFromBuffer8(&sz, pMessageIO);
        pcm_pScopeVarSize[i] = sz;
        
        /* variable address */
        pMessageIO = FMSTR_AddressFromBuffer(&pcm_pScopeVarAddr[i], pMessageIO);

        /* valid numeric variable sizes only */
        if(sz == 0U || sz > 8U)
        {
            return FMSTR_ConstToBuffer8(pResponse, FMSTR_STC_INVSIZE);
        }
        
#if FMSTR_CFG_BUS_WIDTH > 1U
        /* even sizes only */
        if(sz & 0x1)
        {
            return FMSTR_ConstToBuffer8(pResponse, FMSTR_STC_INVSIZE);
        }
#endif
        
#if FMSTR_USE_TSA && FMSTR_USE_TSA_SAFETY
        if(!FMSTR_CheckTsaSpace(pcm_pScopeVarAddr[i], (FMSTR_SIZE8) sz, 0U))
        {
            return FMSTR_ConstToBuffer8(pResponse, FMSTR_STC_EACCESS);
        }
#endif
        
    }

    /* activate scope */
    pcm_nScopeVarCount = nVarCnt;

    /* return just a status */
    return FMSTR_ConstToBuffer8(pResponse, FMSTR_STS_OK);
}

/**************************************************************************//*!
*
* @brief    Handling READSCOPE command
*
* @param    pMessageIO - original command (in) and response buffer (out) 
*
* @return   As all command handlers, the return value should be the buffer 
*           pointer where the response output finished (except checksum)
*
******************************************************************************/

FMSTR_BPTR FMSTR_ReadScope(FMSTR_BPTR pMessageIO)
{
    FMSTR_U8 i;
    
    if(!pcm_nScopeVarCount)
    {
        return FMSTR_ConstToBuffer8(pMessageIO, FMSTR_STC_NOTINIT);
    }
    
    /* success */
    pMessageIO = FMSTR_ConstToBuffer8(pMessageIO, FMSTR_STS_OK);
    
    for (i=0U; i<pcm_nScopeVarCount; i++)
    {
        pMessageIO = FMSTR_CopyToBuffer(pMessageIO, pcm_pScopeVarAddr[i], pcm_pScopeVarSize[i]);
    } 
        
    /* return end position */
    return pMessageIO;  
}

#else  /* (FMSTR_USE_SCOPE) && !(FMSTR_DISABLE) */

/*lint -efile(766, freemaster_protocol.h) include file is not used in this case */

#endif /* (FMSTR_USE_SCOPE) && !(FMSTR_DISABLE) */

