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
#define DRV8302 1

#define MAX_LOG_LENGTH 20


output_t recoder_number;


static void vTaskUSART(void *pvParameters);
//static void vTaskSUBJ(void *pvParameters);
static void vTaskBLDC(void *pvParameters);
static void vTaskCOTL(void *pvParameters);


static void AppTaskCreate (void);
//static void AppObjCreate (void);

/*
**********************************************************************************************************
											��������
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
	uint8_t  usData[4];
	
}MSG_T;

MSG_T   g_tMsg; /* ����һ���ṹ��������Ϣ���� */




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
  //  USART_POLLING_Init();
     
    /* �������� */
	AppTaskCreate();
    
    /* ��������ͨ�Ż��� */
  //	AppObjCreate();// WT.EDIT log_init(10, MAX_LOG_LENGTH);
    /*�������ȣ���ʼִ������*/
    vTaskStartScheduler();
    for (;;)
        ;
}

/******************************************************************************************************
*	������: vTaskTaskUSART
*	����˵�����ӿ���Ϣ����
*	�β�: pvParameters ���ڴ��������ʱ���ݵ��β�
*	����ֵ: ��
*   ���ȼ��� 1 ����ֵԽС�����ȼ�Խ��
********************************************************************************************************/
static void vTaskUSART(void *pvParameters)
{
 // TickType_t xLastWakeTime;
 // const TickType_t xFrequency = 300;
  uint8_t i,D0;
  MSG_T *ptMsg;
  BaseType_t xResult;
  TickType_t ucUsartValue;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* �ȴ�ʱ��300ms */
  /* ��ʼ���ṹ��ָ�� */
	ptMsg = &g_tMsg;
	
	/* ��ʼ������ */
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
		if(D0==03)/*����PID ����*/
        {
           
	      xTaskNotify(xHandleTaskCOTL,      /* Ŀ������ */
							ptMsg->usData[3],              /* �������� */
							eSetValueWithOverwrite);/* �ϴ�Ŀ������û��ִ�У��ᱻ���� */
		  		PRINTF("Send to xHanderCONT StartUp \n");
        }
		
	  vTaskDelay(xMaxBlockTime);// vTaskDelayUntil(&xLastWakeTime, xFrequency);
   
  }
}
/*********************************************************************************************************
*	������: vTaskBLDC
*	��������: BLDC������к���
*	�β�: pvParameters �����ú���ʱ���β�
*	����ֵ: ��
*   ���ȼ�: 2 (��ֵԽС�����ȼ�Խ��)

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
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1); /* �������ȴ�ʱ��Ϊ300ms */
	uint32_t ucConValue;
   
	
	while(1)
    {       

        xResult = xTaskNotifyWait(0x00000000,      
						          0xFFFFFFFF,      
						          &ucConValue,        /* �洢ulNotifiedValue��ulvalue�� */
						          xMaxBlockTime);  /* ����ӳ�ʱ��?*/
	  if(xResult == pdPASS)
		{
			/* Receivce data printf  */
          printf("vTaskBLDC  = %#x\r\n",ucConValue );
		  ucValue = ucConValue ;
		}
		else
		{
			/* ��ʱ */
          LED2= !LED2;
		}
    
		if(ucValue==0xa0){ 
			
				  PWM_Duty = 50;
				  #ifdef DRV8302
				  	GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,1);
				  #endif 
				  uwStep = HallSensor_GetPinState();
				  HALLSensor_Detected_BLDC(); 

				  sampleMask++;
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
					PRINTF("xTask BLDC \r\n");

		}
		if(sampleMask >=100)sampleMask =0;
		 

	
        taskYIELD();// // vTaskDelayUntil(&xLastWakeTime, xFrequency); // vTaskDelay(xMaxBlockTime);   // taskYIELD();//      
      }
 } 

/*********************************************************************************************************
*
*	������: vTaskCOTL
*	��������: ���������⹦��
*	�β�: pvParameters 
*	����ֵ: ��
*   ���ȼ���3
*
*********************************************************************************************************/
static void vTaskCOTL(void *pvParameters)
{
   
     uint8_t ucKeyCode=0,pid_s=0;
     uint8_t start_s =0;
   
     
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
     TickType_t xLastWakeTime;

   // const TickType_t xFrequency = 200;
    xLastWakeTime = xTaskGetTickCount();
    BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(10); /* ��������ʱ��10ms */
	uint8_t ucControl=0;
	uint32_t rlValue,ulValue;
	

	while(1)
    {
	      ucKeyCode = KEY_Scan(0);
           
        
		  xResult = xTaskNotifyWait(0x00000000,      
						            0xFFFFFFFF,      
						            &rlValue,        /* �洢ulNotifiedValue��ulvalue�� */
						            xMaxBlockTime);  /* ����ʱ�� */
		
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
			 if(ulValue == 0x0c)
			 {
                ucKeyCode =PID_INPUT_PRES ; //PID 
			 }
		
			 
		}
		else
		{
			/* 3?���� */
			LED1=0;
			LED2 =0 ;
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
    				 xTaskNotify(xHandleTaskBLDC,           /* Ŀ������--��ǩ��xHandleTaskBLDC */
    								ucControl,              /* �������� */
    								eSetValueWithOverwrite);/* �ϴ�Ŀ������û��ִ�У��ᱻ���� */
                    PRINTF("START KEY IS SEND WORKS \n");
				  }
				  else 
				  {
                     ucControl = 0xa1;
					 start_s =0;
					  xTaskNotify(xHandleTaskBLDC,          /* Ŀ������ */
									ucControl,              /* �������� */
									eSetValueWithOverwrite);/* �ϴ�Ŀ������û��ִ�У��ᱻ���� */
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
*	������: AppTaskCreate
*	��������: ����Ӧ������
*	��������
*	����ֵ����
*********************************************************************************************************/
static void AppTaskCreate (void)
{
    xTaskCreate( vTaskUSART,   									/* ������     */
                 "vTaskUserIF",     							/* ������    */
                 configMINIMAL_STACK_SIZE + 166,               	/* ����ջ��С����λword��4���ֽ� */
                 NULL,              							/* ������� */
                 tskIDLE_PRIORITY+1,                 			/* �������ȼ� */
                 &xHandleTaskUSART );  							/* ������     */

	xTaskCreate( vTaskBLDC,    									/* ������ */
                 "vTaskBLDC",  									/* ������    */
                 configMINIMAL_STACK_SIZE + 934,         		/* ����ջ��С����λword��4���ֽ� */
                 NULL,        									/* �������      */
                 tskIDLE_PRIORITY+2,           					/* �������ȼ� */
                 &xHandleTaskBLDC); 							/* ������ */

	xTaskCreate( vTaskCOTL,    									/* ������  */
                 "vTaskCOTL",  									/* ������    */
                 configMINIMAL_STACK_SIZE + 166,         		/* ����ջ��С����λword��4���ֽ� */
                 NULL,        									/* ������� */
                 tskIDLE_PRIORITY+3,           					/* �������ȼ� */
                 &xHandleTaskCOTL); 							/* ������      */

}
/********************************************************************
 *
 *	������: AppObjCreate
 *	��������: ��������ͨ�Ż���
 *	�����β�: ��
 *	����ֵ: ��
 *
********************************************************************/
#if 0
static void AppObjCreate (void)
{
  
    /* �������� 1*/
	xQueue1 = xQueueCreate(10, sizeof(uint8_t));
    if( xQueue1 == 0 )
    {
       printf("xQueuel set up fail!!!!"); 
       /* ��������ʧ�� */
    } 
    #if 0
     /* ��������2 */
	xQueue2 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue2 == 0 )
    {
         printf("xQueue2 set up fail!!!!"); 
		 /* ��������ʧ��  */
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

	 uint8_t ucControl =0xff;
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BRAKE_KEY_GPIO, 1U << BRAKE_KEY_GPIO_PIN );
    /* Change state of button. */
    #ifdef DRV8302
		GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,0);
	#endif 
    PMW_AllClose_ABC_Channel();
	
    xTaskNotify(xHandleTaskBLDC,          
    			ucControl,              
    			eSetValueWithOverwrite);
    PRINTF("Interrupt Occurs!!!! \r\n");
	                  
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
	  __DSB();
#endif

}
#endif 


