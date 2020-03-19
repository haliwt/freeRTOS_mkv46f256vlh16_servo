/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause  MKV46F256VLH16
 */

/*System includes.*/
#include <stdio.h>
//#include "amclib_FP.h"
//#include "mlib_FP.h"
//#include "gflib_FP.h"
//#include "gdflib_FP.h"
//#include "gmclib_FP.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_debug_console.h"

#include "fsl_pwm.h"
#include "pin_mux.h"
#include "fsl_xbara.h"
#include "led.h"
#include "key.h"
#include "bldc.h"
#include "adc.h"
#include "pollingusart.h"
#include "output.h"
#include "input.h"
#include "enc.h"
#include "ftm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DRV8302 1

#define MAX_LOG_LENGTH 20

//#define ENC_PHASE 1
#define FTM_PHASE  2


output_t recoder_number;


static void vTaskUSART(void *pvParameters);
//static void vTaskSUBJ(void *pvParameters);
static void vTaskBLDC(void *pvParameters);
static void vTaskCOTL(void *pvParameters);


static void AppTaskCreate (void);
//static void AppObjCreate (void);

/*
**********************************************************************************************************
											锟斤拷锟斤拷锟斤拷锟斤拷
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskUSART = NULL;
static TaskHandle_t xHandleTaskSUBJ =  NULL;
static TaskHandle_t xHandleTaskBLDC = NULL;
static TaskHandle_t xHandleTaskCOTL = NULL;

//static QueueHandle_t xQueue1 = NULL;
//static QueueHandle_t xQueue2 = NULL;

__IO uint32_t gCurPosValue;


typedef struct Msg
{
	uint8_t  ucMessageID;
	uint8_t  usData[4];
	uint8_t  usInNum;
	
}MSG_T;

MSG_T   g_tMsg; /*  */




/*******************************************************************************
 *
 * Code
 *
 * @brief Main function
 *
******************************************************************************/
int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    #ifdef ENC_PHASE
    	ENC_PhaseAB_Init();
    #endif
   
    BOARD_InitDebugConsole();
	
    
   
    LED_Init();
    KEY_Init();
    DelayInit();
    HALL_Init();
    
  //  SD315AI_SO12_Input_Init();
    
    HallSensor_GetPinState();
    OUTPUT_Fucntion_Init();
    ADC_CADC_Init();
    ABC_POWER_OUTPUT_Init();
    /* Set the PWM Fault inputs to a low value */
    PWM_BLDC_Init();
  //  USART_POLLING_Init();
     #ifdef FTM_PHASE 
		FTM_PhaseAB_Init();
	#endif
     
    /* 创建任务 */
	AppTaskCreate();
    
    /* 锟斤拷锟斤拷锟斤拷锟斤拷通锟脚伙拷锟斤拷 */
  //	AppObjCreate();// WT.EDIT log_init(10, MAX_LOG_LENGTH);
    /*启动调度，开始执行任务*/
    vTaskStartScheduler();
    for (;;)
        ;
}

/******************************************************************************************************
*	函数名: vTaskTaskUSART
*	功能说明：接口消息处理
*	形参: pvParameters 是在创造该任务时传递的形参
*	返回值: 无
*   优先级： 1 ：数值越小，优先级越低
********************************************************************************************************/
static void vTaskUSART(void *pvParameters)
{
 // TickType_t xLastWakeTime;
 // const TickType_t xFrequency = 300;
  uint8_t i,D0;
  MSG_T *ptMsg;
  BaseType_t xResult;
  TickType_t ucUsartValue;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* 等待时间200ms */
  /* 锟斤拷始锟斤拷锟结构锟斤拷指锟斤拷 */
	ptMsg = &g_tMsg;
	
	/* 锟斤拷始锟斤拷锟斤拷锟斤拷 */
	ptMsg->ucMessageID = 0;
    for(i=0;i<4;i++)
        D0 =ptMsg->usData[i]=0;

  while(1)
    {

		UART_ReadBlocking(DEMO_UART, ptMsg->usData, 4);
#if 1
        for(i=0;i<4;i++){
		          D0=ptMsg->usData[i];
			
				  printf("ptMsg->usData[i]=%#x  \r\n",ptMsg->usData[i]);
        }
#endif 		
		if(D0 == 0x01) //'1' = 0x31
         {
             
		      xTaskNotify(xHandleTaskCOTL,      /* notify to who's handle of name */
								ptMsg->usData[1],              /* send to value  */
								eSetValueWithOverwrite);/* send to order "select item" */
              PRINTF("Send to xHanderCONT is OK \n");
               
         }
        if(D0== 0x02)
        {
           
	      xTaskNotify(xHandleTaskCOTL,      				/* notify to who's name handle  */
							ptMsg->usData[2],              	/* send to value */
							eSetValueWithOverwrite);		/* set up order "modle item" */
		  					PRINTF("Send to xHanderCONT DIR CCW \n");
        }
		if(D0==03)/*锟斤拷锟斤拷PID 锟斤拷锟斤拷*/
        {
           
	      xTaskNotify(xHandleTaskCOTL,      /* 目锟斤拷锟斤拷锟斤拷 */
							ptMsg->usData[3],              /* 锟斤拷锟斤拷锟斤拷锟斤拷 */
							eSetValueWithOverwrite);/* 锟较达拷目锟斤拷锟斤拷锟斤拷没锟斤拷执锟叫ｏ拷锟结被锟斤拷锟斤拷 */
		  		PRINTF("Send to xHanderCONT StartUp \n");
        }
		
	  vTaskDelay(xMaxBlockTime);// vTaskDelayUntil(&xLastWakeTime, xFrequency);
   
  }
}
/*********************************************************************************************************
*	函数名: vTaskBLDC
*	函数功能: BLDC电机运行函数
*	形参: pvParameters 创建该函数时的形参
*	返回值: 无
*   优先级: 2 (数值越小，优先级越低)

*********************************************************************************************************/
static void vTaskBLDC(void *pvParameters)
{
    
    uint32_t ucValue;
   
	TickType_t xLastWakeTime;
	 volatile uint32_t encoder_count         = 0U;
	 volatile bool encoder_direction         = false;
	const TickType_t xFrequency = 1;
    xLastWakeTime = xTaskGetTickCount();
    volatile uint16_t pwm_f=0;
	uint16_t sampleMask;
	BaseType_t xResult;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(5); /* Blocked times 1ms */
	uint32_t ucConValue;
   
	
	while(1)
    {       

     
        xResult = xTaskNotifyWait(0x00000000,      
						          0xFFFFFFFF,      
						          &ucConValue,        /* send to data  */
						          xMaxBlockTime);  /* delay max times*/
	  if(xResult == pdPASS)
		{
			/* Receivce data printf  */
          printf("vTaskBLDC  = %#x\r\n",ucConValue );
		  ucValue = ucConValue ;
		}
    
		if(ucValue==0xa0){ 
			
				  PWM_Duty = 50 -g_tMsg.usInNum ;
				  if(PWM_Duty <=0)PWM_Duty =0;
				  #ifdef DRV8302
				  	GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,1);
				  #endif 
				  uwStep = HallSensor_GetPinState();
				  HALLSensor_Detected_BLDC(); 

				  sampleMask++;
				  #ifdef ENC_PHASE
				    gCurPosValue = ENC_GetPositionValue(DEMO_ENC_BASEADDR);
				  #endif 
				  #ifdef FTM_PHASE
				  	//FTM_GetQuadDecoderCounterValue(DEMO_FTM_BASEADDR);
				  #endif 
		}
		else{
			 #ifdef DRV8302
		 	  GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,0);
		    #endif 
	      DelayMs(50);
	      GPIO_PortToggle(GPIOD,1<<BOARD_LED1_GPIO_PIN);
	      DelayMs(50);
			
		}
		if(sampleMask==100){

				               xTaskNotify(xHandleTaskCOTL,      
											sampleMask,              
											eSetValueWithOverwrite);
				#ifdef ENC_PHASE
					printf("Current position value: %ld\r\n", ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR));
			        printf("Position differential value: %d\r\n", (int16_t)ENC_GetHoldPositionDifferenceValue(DEMO_ENC_BASEADDR));
			        printf("Position revolution value: %d\r\n", ENC_GetHoldRevolutionValue(DEMO_ENC_BASEADDR));
				#endif

			   #ifdef FTM_PHASE
			         /* Read counter value */
			        encoder_count = FTM_GetQuadDecoderCounterValue(DEMO_FTM_BASEADDR);
			        /* Clear counter */
			      //  FTM_ClearQuadDecoderCounterValue(DEMO_FTM_BASEADDR);
			        /* Read direction */
			        if (FTM_GetQuadDecoderFlags(DEMO_FTM_BASEADDR) & kFTM_QuadDecoderCountingIncreaseFlag)
			        {
			            encoder_direction = true;
			        }
			        else
			        {
			            encoder_direction = false;
			        }
			      if (encoder_direction)
			        {
			            PRINTF("Encoder direction:+++++++++++\r\n");
			        }
			        else
			        {
			            PRINTF("Encoder direction:!!!!!!!!!!!! -\r\n");
			        }

			        PRINTF("Get current counter: %d\r\n", encoder_count);
			    #endif
		}
		if(sampleMask >=100)sampleMask =0;
		 

	
        taskYIELD();// // vTaskDelayUntil(&xLastWakeTime, xFrequency); // vTaskDelay(xMaxBlockTime);   // taskYIELD();//      
      }
 } 

/*********************************************************************************************************
*
*	函数名: vTaskCOTL
*	函数功能: 按键输入检测功能
*	形参: pvParameters 
*	返回值: 无
*   优先级：3
*
*********************************************************************************************************/
static void vTaskCOTL(void *pvParameters)
{
   
     uint8_t ucKeyCode=0,pid_s=0;
     uint8_t start_s =0;
     volatile uint32_t encoder_count         = 0U;
	 volatile bool encoder_direction         = false;
      uint32_t mCurPosValue;
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
     TickType_t xLastWakeTime;

   // const TickType_t xFrequency = 200;
    xLastWakeTime = xTaskGetTickCount();
    BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100); /* 设置阻塞时间10ms */
	int32_t ucControl=0;
	int32_t rlValue,ulValue;
	

	while(1)
    {
	    
	      ucKeyCode = KEY_Scan(0);
           
        
		  xResult = xTaskNotifyWait(0x00000000,      
						            0xFFFFFFFF,      
						            &rlValue,        /* 锟芥储ulNotifiedValue锟斤拷ulvalue锟斤拷 */
						            xMaxBlockTime);  /* 阻塞时间 */
		
		if( xResult == pdPASS )
		{
            ulValue=rlValue;
			if(ulValue >=0)
				printf("vTaskCOTL = %#x\r\n", ulValue);
			else printf("-vTaskCOTL = -%#x\r\n", ulValue);
			/****************digitalkey coder******************/
			
			 if(ulValue == 0x64){
					g_tMsg.usInNum ++ ;
					if(g_tMsg.usInNum == 50)g_tMsg.usInNum =0;
			 }
			 /* This read operation would capture all the position counter to responding hold registers. */
      
			#if 0
	        /* Read the position values. */
	        printf("Current position value: %ld\r\n", gCurPosValue);
	        printf("Position differential value: %d\r\n", (int16_t)ENC_GetHoldPositionDifferenceValue(DEMO_ENC_BASEADDR));
	        printf("Position revolution value: %d\r\n", ENC_GetHoldRevolutionValue(DEMO_ENC_BASEADDR));
			#endif 
				#if 1
			      
			        /* Read counter value */
			        encoder_count = FTM_GetQuadDecoderCounterValue(DEMO_FTM_BASEADDR);
			        /* Clear counter */
			      //  FTM_ClearQuadDecoderCounterValue(DEMO_FTM_BASEADDR);
			        /* Read direction */
			        if (FTM_GetQuadDecoderFlags(DEMO_FTM_BASEADDR) & kFTM_QuadDecoderCountingIncreaseFlag)
			        {
			            encoder_direction = true;
			        }
			        else
			        {
			            encoder_direction = false;
			        }
			      if (encoder_direction)
			        {
			            PRINTF("Encoder direction:+++++++++++\r\n");
			        }
			        else
			        {
			            PRINTF("Encoder direction:!!!!!!!!!!!! -\r\n");
			        }

			        PRINTF("Get current counter: %d\r\n", encoder_count);
			       // FTM_ClearQuadDecoderCounterValue(DEMO_FTM_BASEADDR);
					#endif 
	    }
		
		else{
				LED2 = ! LED2;
		}

      /*Key of function */  
		if(ucKeyCode !=KEY_UP) 
				
			{
               switch(ucKeyCode)//if(ptMsg->ucMessageID == 0x32)
                { 
                 
                  case DIR_CW_PRES : /*PTE29-DIR_CW_PRES ://Dir =1 ,PTE29-CW,KEY1*/

				  PRINTF("DIR_CW_PRES key \r\n");
				   Dir =CCW; //CCW = 1
				
				  	break;

				 case START_PRES:
                   PRINTF("START_PRES key \r\n");
				    start_s ++;
		          if(start_s == 1)
		          {
                    
                     ucControl =0xa0;
    				 recoder_number.break_f=0;
    				 xTaskNotify(xHandleTaskBLDC,           /* Who's name handleTask */
    								ucControl,              /* ulvalue of value */
    								eSetValueWithOverwrite);/* xTaskNotify of set up mode */
                    PRINTF("START KEY IS SEND WORKS \n");
				  }
				  else 
				  {
                     ucControl = 0xa1;
					 start_s =0;
					  xTaskNotify(xHandleTaskBLDC,          /* Who's name hanleTask */
									ucControl,              /* ulvalue of value */
									eSetValueWithOverwrite);/* mode set up */
					 PRINTF("START KEY IS STOP\n");
				  }
                 
				  break;
				  
				 case DIR_CCW_PRES: //Dir = 0;PTE24 =CCW KEY3

			      recoder_number.dir_change++;
	  			 PRINTF(" DIR_change = %d  \r\n", recoder_number.dir_change);
				 Dir =CW; //CW = -1
	  			 if(recoder_number.dir_change == 1)
	   				{
                        LED1 =0;
						LED2 =0;
				    }
				 else 
				   {
                      recoder_number.dir_change =0;
					  
				   }
			
           		break;
				
			 case PID_INPUT_PRES ://4
				  PRINTF("PID_INPUT_PRES key \r\n");
				  uint8_t ucControl=0xa1;
                
				 xTaskNotifyFromISR( xHandleTaskBLDC,
                        ucControl,
                        eSetValueWithOverwrite,
                        &xHigherPriorityTaskWoken );
    
                 portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
				break;
				
			 
			}
        
	}
     taskYIELD();// vTaskDelayUntil(&xLastWakeTime, xFrequency);//taskYIELD();//
   }//end whilt(1)
}
/********************************************************************************************************
*	函数名: AppTaskCreate
*	函数功能: 创建应用任务
*	参数：无
*	返回值：无
*********************************************************************************************************/
static void AppTaskCreate (void)
{
    xTaskCreate( vTaskUSART,   									/* 任务函数     */
                 "vTaskUserIF",     							/* 任务名    */
                 configMINIMAL_STACK_SIZE + 166,               	/* 任务栈大小，单位word，4个字节 */
                 NULL,              							/* 任务参数 */
                 tskIDLE_PRIORITY+1,                 			/* 任务优先级 */
                 &xHandleTaskUSART );  							/* 任务句柄     */

	xTaskCreate( vTaskBLDC,    									/* 任务函数 */
                 "vTaskBLDC",  									/* 任务名    */
                 configMINIMAL_STACK_SIZE + 934,         		/* 任务栈大小，单位word，4个字节 */
                 NULL,        									/* 任务参数      */
                 tskIDLE_PRIORITY+2,           					/* 任务优先级 */
                 &xHandleTaskBLDC); 							/* 任务句柄 */

	xTaskCreate( vTaskCOTL,    									/* 任务函数  */
                 "vTaskCOTL",  									/* 任务名    */
                 configMINIMAL_STACK_SIZE + 166,         		/* 任务栈大小，单位word，4个字节 */
                 NULL,        									/* 任务参数 */
                 tskIDLE_PRIORITY+3,           					/* 任务优先级 */
                 &xHandleTaskCOTL); 							/* 任务句柄      */

}

/******************************************************************************
 *
 * Function Name:BARKE_KEY_IRQ_HANDLER(void)
 * Function Active: Interrpt brake input key 
 * @brief Interrupt service fuction of switch.
 *
 * This function toggles the LED
 *
******************************************************************************/
#if 1
void BARKE_KEY_IRQ_HANDLER(void )//void BOARD_BRAKE_IRQ_HANDLER(void)
{

	 uint8_t ucControl =0xff;
	 BaseType_t xYieldRequired;
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BRAKE_KEY_GPIO, 1U << BRAKE_KEY_GPIO_PIN );
    /* Change state of button. */
    #ifdef DRV8302
		GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,0);
	#endif 
    PMW_AllClose_ABC_Channel();
    PRINTF("Interrupt Occurs!!!! \r\n");

    
  	 xYieldRequired =xTaskResumeFromISR(xHandleTaskBLDC);
     /* 退出中断后，是否需要执行任务切换*/
    if(xYieldRequired==pdTRUE)
		portYIELD_FROM_ISR(xYieldRequired);
	
   
	                  
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
	  __DSB();
#endif

}
#endif 


