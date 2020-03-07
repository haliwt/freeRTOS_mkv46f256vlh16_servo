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
//#include "usart_edma_rb.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MAX_LOG_LENGTH 20


output_t recoder_number;


static void vTaskUSART(void *pvParameters);
static void vTaskSUBJ(void *pvParameters);
static void vTaskBLDC(void *pvParameters);
static void vTaskCOTL(void *pvParameters);


static void AppTaskCreate (void);
//static void AppObjCreate (void);

/*
**********************************************************************************************************
											变量声明
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskUSART = NULL;
static TaskHandle_t xHandleTaskSUBJ =  NULL;
static TaskHandle_t xHandleTaskBLDC = NULL;
static TaskHandle_t xHandleTaskCOTL = NULL;

//static QueueHandle_t xQueue1 = NULL;
//static QueueHandle_t xQueue2 = NULL;



typedef struct Msg
{
	uint8_t  ucMessageID;
	uint8_t  usData[10];
	
}MSG_T;

MSG_T   g_tMsg; /* 定义一个结构体用于消息队列 */




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
    USART_POLLING_Init();
     
    /* 创建任务 */
	AppTaskCreate();
    
    /* 创建任务通信机制 */
//	AppObjCreate();// WT.EDIT log_init(10, MAX_LOG_LENGTH);
    
    vTaskStartScheduler();
    for (;;)
        ;
}

/******************************************************************************************************
*	函 数 名: vTaskTaskUSART
*	功能说明: 接口消息处理。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 1  (数值越小优先级越低，最低优先级)
********************************************************************************************************/
static void vTaskUSART(void *pvParameters)
{
 // TickType_t xLastWakeTime;
 // const TickType_t xFrequency = 300;
  uint8_t i,ch;
  MSG_T *ptMsg;
 
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1); /* 设置最大等待时间为5ms */
  /* 初始化结构体指针 */
	ptMsg = &g_tMsg;
	
	/* 初始化数组 */
	ptMsg->ucMessageID = 0;
    for(i=0;i<4;i++)
        ch =ptMsg->usData[i];

  while(1)
    {
    
      printf("vTaskUSART-1 \r\n");
   
       UART_ReadBlocking(DEMO_UART, ptMsg->usData, 4);
      
        for(i=0;i<4;i++)
		{
          //ch=ptMsg->usData[i];
		  printf("ptMsg->usData[i]= %d \r\n",ptMsg->usData[i]);

		}
      
        if(ptMsg->usData[0] == 0x01) //'1' = 0x31
         {
             
		      xTaskNotify(xHandleTaskCOTL,      /* 目标任务 */
								ptMsg->usData[2],              /* 发送数据 */
								eSetValueWithOverwrite);/* 上次目标任务没有执行，会被覆盖 */
               printf("Send to xHanderCONT is OK \n");
               
         }
        if(ptMsg->usData[0]== 0x02)
        {
           
	      xTaskNotify(xHandleTaskSUBJ,      /* 目标任务 */
							ptMsg->usData[2],              /* 发送数据 */
							eSetValueWithOverwrite);/* 上次目标任务没有执行，会被覆盖 */
		  					printf("Send to xHanderCONT DIR CCW \n");
        }
		if(ptMsg->usData[0]== 0x3)/*接收PID 参数*/
        {
           
	      xTaskNotify(xHandleTaskSUBJ,      /* 目标任务 */
							ptMsg->usData[2],              /* 发送数据 */
							eSetValueWithOverwrite);/* 上次目标任务没有执行，会被覆盖 */
		  		printf("Send to xHanderCONT StartUp \n");
        }
	    taskYIELD(); //vTaskDelay(xMaxBlockTime);// vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*********************************************************************************************************
*	函 数 名: vTaskBLDC
*	功能说明: 使用函数xQueueReceive接收任务vTaskTaskUserIF发送的消息队列数据(xQueue2)	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 3  
*********************************************************************************************************/
static void vTaskBLDC(void *pvParameters)
{
    
    uint32_t ucValue;
   
	TickType_t xLastWakeTime;
	
	const TickType_t xFrequency = 1;
    xLastWakeTime = xTaskGetTickCount();
    volatile uint16_t pwm_f=0;
	uint16_t sampleMask;
	BaseType_t xResult;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1); /* 设置最大等待时间为300ms */
	uint32_t ucConValue;
    uint16_t dirvalue;
	
	while(1)
    {       

        xResult = xTaskNotifyWait(0x00000000,      
						          0xFFFFFFFF,      
						          &ucConValue,        /* 存储ulNotifiedValue在ulvalue中 */
						          xMaxBlockTime);  /* 最大延迟时间 */
	  if(xResult == pdPASS)
		{
			/* 接收数据成功 */
          printf("vTaskBLDC  = %#x\r\n",ucConValue );
		  ucValue = ucConValue ;
		}
		else
		{
			/* 超时 */
          LED2= !LED2;
		}
    
		if(ucValue==0xa0){ 
			
				  
				  PWM_Duty = 50;
				  GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,1);
				  uwStep = HallSensor_GetPinState();
				  HALLSensor_Detected_BLDC(); 
		}
		else{
			// #ifdef DRV8302
		 	GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,0);
		 //   #endif 
	      DelayMs(50);
	      GPIO_PortToggle(GPIOD,1<<BOARD_LED1_GPIO_PIN);
	      DelayMs(50);
			
		}
     

	
        taskYIELD();// // vTaskDelayUntil(&xLastWakeTime, xFrequency); // vTaskDelay(xMaxBlockTime);   // taskYIELD();//      
      }
 } 

/*********************************************************************************************************
*
*	函 数 名: vTaskCOTL
*	功能说明: 接收物理按键和数字按键的命令
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*
*********************************************************************************************************/
static void vTaskCOTL(void *pvParameters)
{
   
     uint8_t ucKeyCode=0,abc_s=0,pid_s=0;
     uint8_t start_s =0,door_s = 0,wiper_s=0,air_s=0;
     uint8_t hall_s = 0,wheel_s=0,abc_power_s=0;
     
   
     TickType_t xLastWakeTime;

    const TickType_t xFrequency = 200;
    xLastWakeTime = xTaskGetTickCount();
    BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1); /* 设置最大等待时间为5ms */
	uint8_t ucControl=0;
	uint32_t rlValue,ulValue;
	

	while(1)
    {
	    //  printf("vTaskCOTL-3 \r\n");

          
          
		  ucKeyCode = KEY_Scan(0);
           
        
		  xResult = xTaskNotifyWait(0x00000000,      
						            0xFFFFFFFF,      
						            &rlValue,        /* 存储ulNotifiedValue在ulvalue中 */
						            xMaxBlockTime);  /* 最大延迟时间 */
		
		if( xResult == pdPASS )
		{
            ulValue=rlValue;
			printf("vTaskCOTL = %#x\r\n", ulValue);
			/****************digitalkey coder******************/
			 if(ulValue == 0x01)
			 {
                ucKeyCode =DIR_CW_PRES  ; 
			 }
			 if(ulValue == 0x2)
			 {
                ucKeyCode =DIR_CCW_PRES  ; 
			 }
			 if(ulValue == 0x4)
			 {
                ucKeyCode =START_PRES  ; 
			 }
			 if(ulValue == 0x07)
			 {
                ucKeyCode =DIGITAL_ADD_PRES  ; 
			 }
			 if(ulValue == 0x06)
			 {
                ucKeyCode =DIGITAL_REDUCE_PRES  ; 
			 }
			 if(ulValue == 0x09)
			 {
                ucKeyCode =DOOR_PRES ; 
			 }
			 if(ulValue == 0x0a)
			 {
                ucKeyCode =HALL_PRES ; 
			 }
			 if(ulValue == 0x0c)
			 {
                ucKeyCode =PID_INPUT_PRES ; //PID 
			 }
			  if(ulValue == 0x0e)
			 {
                ucKeyCode =WIPERS_PRES ; 
			 }
			  if(ulValue == 0x12)
			 {
                ucKeyCode =AIR_PRES ; 
			 }
			 
		}
		else
		{
			/* 3?ê± */
			LED1=0;
			LED2 =0 ;
		}

      /*处理接收到的按键功能*/  
		if(ucKeyCode !=KEY_UP) 
				
			{
               switch(ucKeyCode)//if(ptMsg->ucMessageID == 0x32)
                { 
                 
                  case DIR_CW_PRES : /*PTE29-DIR_CW_PRES ://Dir =1 ,PTE29-CW,KEY1*/

				  PRINTF("DIR_CW_PRES key \r\n");
				   Dir =1;
				
				  	break;

				 case START_PRES:
                   PRINTF("START_PRES key \r\n");
				    start_s ++;
		          if(start_s == 1)
		          {
                    
                     ucControl =0xa0;
                     abc_s =5; 
    				 recoder_number.break_f=0;
    				 xTaskNotify(xHandleTaskBLDC,           /* 目标任务--标签名xHandleTaskBLDC */
    								ucControl,              /* 发送数据 */
    								eSetValueWithOverwrite);/* 上次目标任务没有执行，会被覆盖 */
                    printf("START KEY IS SEND WORKS \n");
				  }
				  else 
				  {
                     ucControl = 0xa1;
					 start_s =0;
					  xTaskNotify(xHandleTaskBLDC,          /* 目标任务 */
									ucControl,              /* 发送数据 */
									eSetValueWithOverwrite);/* 上次目标任务没有执行，会被覆盖 */
					 printf("START KEY IS STOP\n");
				  }
                 
				  break;
				  
				 case DIR_CCW_PRES: //Dir = 0;PTE24 =CCW KEY3

			      recoder_number.dir_change++;
	  			 PRINTF(" DIR_change = %d  \r\n", recoder_number.dir_change);
				 Dir =0;
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
				  pid_s++;
				  if(pid_s ==1)
				  	{
                       ucControl = 0x0d;
                        xTaskNotify(xHandleTaskUSART,             /* 目标任务 */
                                   ucControl,              /* 发送数据 */
                                   eSetValueWithOverwrite);/* 上次目标任务没有执行，会被覆盖 */
                     }
					else 
					{
                     ucControl = 0x00;
                     pid_s =0 ;
                      xTaskNotify(xHandleTaskUSART,      /* 目标任务 */
                           ucControl,              /* 发送数据 */
                           eSetValueWithOverwrite);/* 上次目标任务没有执行，会被覆盖 */

					}
				break;
				
			 
			}
        
	}
     taskYIELD();// vTaskDelayUntil(&xLastWakeTime, xFrequency);//taskYIELD();//
   }//end whilt(1)
}
/********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************/
static void AppTaskCreate (void)
{
    xTaskCreate( vTaskUSART,   									/* 任务函数  */
                 "vTaskUserIF",     							/* 任务名    */
                 configMINIMAL_STACK_SIZE + 166,               	/* 任务栈大小，单位word，也就是4字节 */
                 NULL,              							/* 任务参数  */
                 tskIDLE_PRIORITY+1,                 			/* 任务优先级 最低*/
                 &xHandleTaskUSART );  							/* 任务句柄  */

	xTaskCreate( vTaskBLDC,    									/* 任务函数  */
                 "vTaskBLDC",  									/* 任务名    */
                 configMINIMAL_STACK_SIZE + 934,         		/* stack大小，单位word，也就是4字节 */
                 NULL,        									/* 任务参数  */
                 tskIDLE_PRIORITY+2,           					/* 任务优先级*/
                 &xHandleTaskBLDC); 							/* 任务句柄  */

	xTaskCreate( vTaskCOTL,    									/* 任务函数  */
                 "vTaskCOTL",  									/* 任务名    */
                 configMINIMAL_STACK_SIZE + 166,         		/* stack大小，单位word，也就是4字节 */
                 NULL,        									/* 任务参数  */
                 tskIDLE_PRIORITY+3,           					/* 任务优先级*/
                 &xHandleTaskCOTL); 							/* 任务句柄  */

}
/********************************************************************
 *
 *	函 数 名: AppObjCreate
 *	功能说明: 创建任务通信机制
 *	形    参: 无
 *	返 回 值: 无
 *
********************************************************************/
#if 0
static void AppObjCreate (void)
{
  
    /* 创建10个uint8_t型消息队列 */
	xQueue1 = xQueueCreate(10, sizeof(uint8_t));
    if( xQueue1 == 0 )
    {
       printf("xQueuel set up fail!!!!"); 
       /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
    }
    #if 0
     /* 创建10个存储指针变量的消息队列，由于CM3/CM4内核是32位机，一个指针变量占用4个字节 */
	xQueue2 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue2 == 0 )
    {
         printf("xQueue2 set up fail!!!!"); /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
    }
    #endif 

}
#endif 
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
  
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BRAKE_KEY_GPIO, 1U << BRAKE_KEY_GPIO_PIN );
    /* Change state of button. */
    
	GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,0);
	recoder_number.break_f =1;
	PRINTF("interrupte has happed  \r\n");
	                  
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif 


