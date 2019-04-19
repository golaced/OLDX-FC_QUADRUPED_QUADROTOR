#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"


extern OS_TMR   * tmr1;			//软件定时器1
extern OS_TMR   * tmr2;			//软件定时器2
extern OS_TMR   * tmr3;			//软件定时器3
void tmr1_callback(OS_TMR *ptmr,void *p_arg); 		  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg); 	  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg); 
	
//设置任务优先级 0->Highest     
//10->开始任务的优先级设置为最低


#define FUSION_TASK_PRIO       		  1 //MEMS
#define BRAIN_TASK_PRIO       			2 //MEMS
#define UART_TASK_PRIO       			  3 //MEMS
#define POSE_FUSION_TASK_PRIO       			  4 //MEMS
//-----------------------LEG解算线程
//设置任务堆栈大小
#define FUSION_STK_SIZE  					64*15
//任务堆栈	
extern OS_STK FUSION_TASK_STK[FUSION_STK_SIZE];
//任务函数
void fusion_task(void *pdata);

//-----------------------LEG解算线程
//设置任务堆栈大小
#define POSE_FUSION_STK_SIZE  					64*100
//任务堆栈	
extern OS_STK POSE_FUSION_TASK_STK[POSE_FUSION_STK_SIZE];
//任务函数
void pose_fusion_task(void *pdata);

//-----------------------LEG解算线程
//设置任务堆栈大小
#define BRAIN_STK_SIZE  					64*20*12
//任务堆栈	
extern OS_STK BRAIN_TASK_STK[BRAIN_STK_SIZE];
//任务函数
void brain_task(void *pdata);

//------------------------UART线程
//设置任务堆栈大小
#define UART_STK_SIZE  					64*20
//任务堆栈	
extern OS_STK UART_TASK_STK[UART_STK_SIZE];
//任务函数
void uart_task(void *pdata);
//


#endif

