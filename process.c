///**
//  ********************************  STM32F0xx  *********************************
//  * @文件名     ： process.c
//  * @作者       ： HarryZeng
//  * @库版本     ： V1.5.0
//  * @文件版本   ： V1.0.0
//  * @日期       ： 2017年04月21日
//  * @摘要       ： 数据处理
//  ******************************************************************************/
///*----------------------------------------------------------------------------
//  更新日志:
//  2017-04-21 V1.0.0:初始版本
//  ----------------------------------------------------------------------------*/
///* 包含的头文件 --------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "process.h"

/*----------------------------------宏定义-------------------------------------*/


/*------------------------------------函数声明---------------------------------------*/
void 	GetRegisterAState(void);
uint32_t 	Read_Value(PWM_Number PWM);
uint8_t  	Read_KG(void);
void  		SetOut(uint8_t OUT);
void  CI_Mode_SelfLearning(void);
void  MARK_Mode_SelfLearning(void);
void 			scan_key(void);
void 			printFlashTest(void);
void ShortCircuitProtection(void);
void WriteFlash(uint32_t addr,uint32_t data);
extern void DelaymsSet(int16_t ms);
/*------------------------------------全局变量---------------------------------------*/
uint32_t ADC_value = 0;
uint8_t 	ShortCircuit=0;
uint8_t 	ConfirmShortCircuit=0;
uint32_t 		ShortCircuitTimer=0;
uint32_t   ShortCircuitCounter = 0;
uint32_t   ShortCircuitLastTime = 0;
/*状态变量*/
uint8_t RegisterA ;
uint8_t RegisterB ;
uint8_t OUT;



PWM_Number CurrentPWM = PWMX; //默认当前PWM通道为X
uint32_t S_Last,S_Current,S_History,S_FINAL;
int CurrentThreshold=100;
int CurrentDifference = Default_Difference;//默认应差值

uint32_t RegisterACounter=0;

uint8_t KeyTime;
uint16_t key_counter=0;
uint16_t scan_tick=0;
uint8_t KeyIndex=0;
uint32_t FLASHData;
uint8_t EnterSelfFlag=0;
extern int16_t adc_dma_tab[6];
extern uint8_t sample_finish;
extern uint8_t TIM1step;

/********************************/
uint32_t CXA_B[2],CYA_B[2],CZA_B[2];
uint32_t SA_B[2];

/***********************************
*FLASH 字节定义
*0x12000032
解析：32位中,最高4位代表PWM通道的选择：PWMX->1，PWMY->2，PWMZ->4
						 紧接着4位就是应差值的选择：20->1,50->2,80->4
						 剩下的24位则保存则阈值
						 如：0x12000032  代表有效通道:PWMX    应差值选:50   阈值:50
************************************/
uint32_t FLASHData = 0x12000032;
/*----------------------------------函数定义----------------------------------------*/
/*****************
*
*主数据处理函数
*
*****************/
void DataProcess(void)
{
	int First_ten_times;
	uint8_t  OUTPin_STATE;
	/*
		FALSH 读取参数
	*/
	printFlashTest();
	
	for(First_ten_times = 0;First_ten_times<10;First_ten_times++) /*刚上电，前十组PWM只算 RegisterA*/
	{
			GetRegisterAState();
	}
	
	while(1)
	{
		/*短路保护判断*/
		ShortCircuitProtection();
		
		while(ConfirmShortCircuit)
		{
				GPIO_WriteBit(OUT_GPIO_Port, OUT_Pin, Bit_RESET); /*发现短路，将OUT引脚拉低*/
				if((ShortCircuitLastTime - ShortCircuitTimer)>=1000)
				{
						ConfirmShortCircuit = 0;
						ShortCircuitCounter = 0;
						ShortCircuit=0;
				}
		}
		
		if(KeyIndex<1)
		{
			RegisterB = Read_KG();
			GetRegisterAState();
		
			/*同或运算*/
			OUT=!(RegisterB^RegisterA);
			/*输出OUT*/
			SetOut(OUT);
			//printf("Normal Work!");
		}
		
		/*按键进入自学习模式*/
		if(KeyTime>0)  
		{
			//printf("first enter ,key time:%d\r\n",KeyTime);
			OUTPin_STATE = GPIO_ReadInputDataBit(OUT_GPIO_Port,OUT_Pin); //读取OUT的值,用于写FLASH时，保持OUT的引脚电平不变
			GPIO_WriteBit(OUT_GPIO_Port, OUT_Pin, (BitAction)OUTPin_STATE);
			CI_Mode_SelfLearning();
		}
	}
}

/********************
*
*判断出RegisterA状态
*
**********************/
void GetRegisterAState(void)
{
	RegisterACounter++;// 用于记录次数
	
	if(RegisterACounter<=1) //如果当前是第一次计算RegisterA，则需要先获取一次Signal，才能做相减
	{
		S_Current = Read_Value(CurrentPWM);
		//printf("第一次进入 registor A\r\n");
		S_Last = S_Current;
	}
	
	S_Current = Read_Value(CurrentPWM);
	S_FINAL = (S_Current + S_Last)/2;    //相加求平均
	
	S_Last = S_Current;

	if(S_FINAL>=CurrentThreshold)
			RegisterA = 1;
	else if(S_FINAL<=(CurrentThreshold - CurrentDifference))
			RegisterA = 0;
		
}

/************************
*
*发出PWM1及所选的PWM通道，读回ADC值
*
*************************/
uint32_t Read_Value(PWM_Number PWM)
{
	/*开启对应的PWM通道*/
	switch (PWM)
	{
	case PWMX:
					PWMX_ON;
					PWMY_OFF;
					PWMZ_OFF;
		break;
	case PWMY:
					PWMX_OFF;
					PWMY_ON;
					PWMZ_OFF;
		break;
	case PWMZ:
					PWMX_OFF;
					PWMY_OFF;
					PWMZ_ON;
		break;
	default:break;
	}
	if(sample_finish) /*DMA中断中，ADC转换完成标记*/
	{
		//ADC_value = (adc_dma_tab[0] * 825)>>10;
		ADC_value = adc_dma_tab[0];
		sample_finish = 0;
	}
	//printf("ADC_value : %d\r\n",ADC_value);
	
	return ADC_value;
}

/***************************************
*
*读取KG拨码开关的值
*
**************************************/
uint8_t  Read_KG(void)
{
	
	uint8_t  KG_STATE;
	
	KG_STATE = GPIO_ReadInputDataBit(KG_GPIO_Port,KG_Pin); //读取KG的值
	
	if(KG_STATE==Bit_SET)
		return 1;
	else
		return 0;
}

/***************************************
*
*设置OUT的输出电平
*
**************************************/
void  SetOut(uint8_t OUT_Value)
{
	if(OUT_Value==1)
	{
		GPIO_WriteBit(OUT_GPIO_Port, OUT_Pin, Bit_SET);
	}
	else
	{
		GPIO_WriteBit(OUT_GPIO_Port, OUT_Pin, Bit_RESET);
	}
}


///***/
//uint32_t SelfLearningGetADC(uint32_t *Self_ADC_Value)
//{
//		while(sample_finish) /*DMA中断中，ADC转换完成标记*/
//		{
//			*Self_ADC_Value = adc_dma_tab[0];
//			sample_finish = 0;
//			return 1;
//		}
//		return 0;
//}
/***************************************
*
*自学习计算
*
**************************************/
	uint32_t SX[4],SY[4],SZ[4];
	float SXA_B[2],SYA_B[2],SZA_B[2];
	float X=0,Y=0,Z=0,BIG=0;
	uint32_t SelfLADC=0;
	uint8_t SelfGetADCWell=0;
	uint32_t temppp;
	extern uint8_t DMAIndex;
	extern int16_t selfADCValue[12];
	
void  MARK_Mode_SelfLearning(void)
{
		uint8_t selfADCIndex=0;
		uint8_t k=0;
		EnterSelfFlag = 1; /*进入自学习标记位*/
		DMAIndex=0;
	
		DelaymsSet(5000);
		if(SelfGetADCWell)
		{
			SelfGetADCWell = 0;
			for(selfADCIndex=0,k=0;k<4;k++)
			{
				SX[k] = selfADCValue[selfADCIndex++];
				SY[k] = selfADCValue[selfADCIndex++];
				SZ[k] = selfADCValue[selfADCIndex++];	
			}
			SXA_B[KeyIndex] = (SX[0]+SX[1]+SX[2]+SX[3])/4; //累加求平均
			SYA_B[KeyIndex] = (SY[0]+SY[1]+SY[2]+SY[3])/4;
			SZA_B[KeyIndex] = (SZ[0]+SZ[1]+SZ[2]+SZ[3])/4;
			
			
		}; /*等待获取四组ADC成功*/
		
		KeyIndex++;  //记录第几次按键   1->SXA,2->SXB
		if(KeyIndex>=2) //第二次按键   //
		{
			//printf("second enter ,key time:%d\r\n",KeyTime);
				KeyIndex = 0;
				/*计算大小，商*/
				/*----------PWMX对比大小--------*/
				if(SXA_B[0]>=SXA_B[1])
				{
						X = (SXA_B[0] / SXA_B[1])*512;
				}
				else
				{
						X = (SXA_B[1] / SXA_B[0])*512;
				}
				/*----------PWMY对比大小--------*/
				if(SYA_B[0]>=SYA_B[1])
				{
						Y = (SYA_B[0] / SYA_B[1])*512;
				}
				else
				{
						Y = (SYA_B[1] / SYA_B[0])*512;
				}
				/*----------PWMZ对比大小--------*/
				if(SZA_B[0]>=SZA_B[1])
				{
						Z = (SZA_B[0] / SZA_B[1])*512;
				}
				else
				{
						Z = (SZA_B[1] / SZA_B[0])*512;
				}
				/*找最大值,计算阈值，判断PWM通道*/
					BIG=(X>Y)?X:Y;
					BIG=(BIG>Z)?BIG:Z;
					FLASHData = 0x00000000;
					if(BIG==X)
					{
						CurrentThreshold = (SXA_B[1] + SXA_B[0])/2;
						CurrentPWM = PWMX;
						FLASHData = FLASHData+CurrentThreshold+0x10000000;
						//printf("CurrentPWM = PWMX,CurrentThreshold:%d\r\n",CurrentThreshold);
					}
					else if(BIG==Y)
					{
						CurrentThreshold = (SYA_B[1] + SYA_B[0])/2;
						CurrentPWM = PWMY;
						FLASHData = FLASHData+CurrentThreshold+0x20000000;
						//printf("CurrentPWM = PWMY,CurrentThreshold:%d\r\n",CurrentThreshold);
					}
					else	
					{
						CurrentThreshold = (SZA_B[1] + SZA_B[0])/2;
						CurrentPWM = PWMZ;
						FLASHData = FLASHData+CurrentThreshold+0x40000000;
						//printf("CurrentPWM = PWMZ,CurrentThreshold:%d\r\n",CurrentThreshold);
					}	
					WriteFlash(0,FLASHData);																	//保存FLASH
					//printf("Save Successfully\r\n");
					KeyIndex = 0;
					EnterSelfFlag = 0;
		}
			KeyTime = 0; //清楚按键标记
}

/*MARK_Mode_SelfLearning*/
uint32_t NXSET,NYSET,NZSET,NS_SET,NXYZ_SET;
void CI_Mode_SelfLearning(void)
{
		uint8_t selfADCIndex=0;
		uint8_t k=0;
		EnterSelfFlag = 1; /*进入自学习标记位*/
		DMAIndex=0;
	
		DelaymsSet(5000);
		if(SelfGetADCWell)
		{
			SelfGetADCWell = 0;
			for(selfADCIndex=0,k=0;k<4;k++)
			{
				SX[k] = selfADCValue[selfADCIndex++];
				SY[k] = selfADCValue[selfADCIndex++];
				SZ[k] = selfADCValue[selfADCIndex++];	
			}
			SXA_B[KeyIndex] = (SX[0]+SX[1]+SX[2]+SX[3])/4; //累加求平均
			SYA_B[KeyIndex] = (SY[0]+SY[1]+SY[2]+SY[3])/4;
			SZA_B[KeyIndex] = (SZ[0]+SZ[1]+SZ[2]+SZ[3])/4;
	
		}; /*等待获取四组ADC成功*/
		
		SA_B[KeyIndex]=SXA_B[KeyIndex]+SYA_B[KeyIndex]+SZA_B[KeyIndex];/*求得SA*/
	
		CXA_B[KeyIndex] = 1024*SXA_B[KeyIndex]/SA_B[KeyIndex];   //1->SXA,2->SXB
		CYA_B[KeyIndex] = 1024*SYA_B[KeyIndex]/SA_B[KeyIndex];
		CZA_B[KeyIndex] = 1024*SZA_B[KeyIndex]/SA_B[KeyIndex];
				
		KeyIndex++;  //记录第几次按键   1->SXA,2->SXB
		
		if(KeyIndex>=2) //第二次按键   //
		{
			//printf("second enter ,key time:%d\r\n",KeyTime);
				KeyIndex = 0;
				
				NXSET =	CXA_B[1] - CXA_B[0];  /*NXSET=CXA-CXB的绝对值*/
				NYSET = CYA_B[1] - CYA_B[0];
			  NZSET = CZA_B[1] - CZA_B[0];
				
				NS_SET = SA_B[1] - SA_B[0];
				
				NXYZ_SET = NXSET+NYSET+NZSET;
				
				CurrentThreshold = 1000 - (NS_SET + NXYZ_SET);
				
//				WriteFlash(0,FLASHData);																	//保存FLASH
//					//printf("Save Successfully\r\n");
//				EnterSelfFlag = 0;
		}
			KeyTime = 0; //清楚按键标记
		
}


/***************************************
*
*扫描按键时间
*
**************************************/
void scan_key(void) 
{ 
	if(SETPin==Bit_SET )
	{
			key_counter++;
	}
	
	else	if (key_counter>middleKEY) 
		{ 
				KeyTime = key_counter; 
				key_counter = 0;
		}
	 else if(key_counter<middleKEY && key_counter>shortKEY)
			{ 
					KeyTime = key_counter; 
					key_counter = 0;
			}
	else	if(key_counter<shortKEY&&key_counter>2)
		{ 
				KeyTime = key_counter;  
				key_counter = 0;
		}
			
}

void WriteFlash(uint32_t addr,uint32_t data)
{
FLASH_Unlock(); //解锁FLASH编程擦除控制器
FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);//清除标志位
FLASH_ErasePage(FLASH_START_ADDR); //擦除指定地址页
FLASH_ProgramWord(FLASH_START_ADDR+(addr*4),data); //从指定页的0地址开始写
FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);//清除标志位
FLASH_Lock(); //锁定FLASH编程擦除控制器
}

//FLASH读取数据测试
uint32_t Flashtemp;
void printFlashTest(void)
{
		uint32_t choose = 0;
		Flashtemp = *(__IO uint32_t*)(FLASH_START_ADDR);
		//DelaymsSet(500);
		//printf("addr:0x%x, data:0x%x\r\n", addr, temp);
		choose = Flashtemp & 0xF0000000;
		//DelaymsSet(500);
		/*读取PWM通道*/
		switch (choose)
		{
			case 0x10000000: CurrentPWM = PWMX;break;
			case 0x20000000: CurrentPWM = PWMY;break;
			case 0x40000000: CurrentPWM = PWMZ;break;
			default : break;
		}
		//printf("CurrentPWM:0x%x,",choose);
		/*应差值固定为 */
		CurrentDifference = Default_Difference;

		//printf("CurrentDifference:0x%x,",choose);
		/*读取应差值*/
		choose = Flashtemp % 0x01000000;
		CurrentThreshold = choose;
		//printf("CurrentThreshold:0x%x\r\n",choose);

}

/*******************************
*
*短路保护
*
*******************************/
void ShortCircuitProtection(void)
{
	uint8_t SCState;
	
	/*读取SC引脚的状态*/
	if(ShortCircuit!=1)
	{
		SCState = GPIO_ReadInputDataBit(SC_GPIO_Port ,SC_Pin);
		if(SCState == Bit_RESET)
		{
			/*拉低FB_SC*/
			ShortCircuit= 1;
		}
		else
		{
			ShortCircuit = 0;
			ConfirmShortCircuit = 0;
		}
	}
	if(ShortCircuit && ShortCircuitCounter>=5)
	{
		ConfirmShortCircuit=1;
		
		GPIO_WriteBit(OUT_GPIO_Port, OUT_Pin, Bit_RESET);/*马上拉低OUT*/
		ShortCircuitTimer = ShortCircuitLastTime;
	}
}

///**** Copyright (C)2017 HarryZeng. All Rights Reserved **** END OF FILE ****/
