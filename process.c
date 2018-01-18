///**
//  ********************************  STM32F0xx  *********************************
//  * @�ļ���     �� process.c
//  * @����       �� HarryZeng
//  * @��汾     �� V1.5.0
//  * @�ļ��汾   �� V1.0.0
//  * @����       �� 2017��04��21��
//  * @ժҪ       �� ���ݴ���
//  ******************************************************************************/
///*----------------------------------------------------------------------------
//  ������־:
//  2017-04-21 V1.0.0:��ʼ�汾
//  ----------------------------------------------------------------------------*/
///* ������ͷ�ļ� --------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "process.h"

/*----------------------------------�궨��-------------------------------------*/


/*------------------------------------��������---------------------------------------*/
void 	MARK_GetRegisterAState(void);
void 	CI_GetRegisterAState(void);
uint32_t 	Read_Value(PWM_Number PWM);
uint8_t  	Read_GOODBAD(void);
void  		SetOut(uint8_t OUT);
void  CI_Mode_SelfLearning(void);
void  MARK_Mode_SelfLearning(void);
void 	scan_key(void);
void 	printFlashTest(void);
void 	ShortCircuitProtection(void);
void 	WriteFlash(uint32_t addr,uint32_t data);
extern void DelaymsSet(int16_t ms);
/*------------------------------------ȫ�ֱ���---------------------------------------*/
uint32_t ADC_value = 0;
uint8_t 	ShortCircuit=0;
uint8_t 	ConfirmShortCircuit=0;
uint32_t 		ShortCircuitTimer=0;
uint32_t   ShortCircuitCounter = 0;
uint32_t   ShortCircuitLastTime = 0;
/*״̬����*/
uint8_t RegisterA;
uint8_t RegisterB = 0;
uint8_t OUT;

	uint32_t SX[4],SY[4],SZ[4];
	float SXA_B[2],SYA_B[2],SZA_B[2];

	float X=0,Y=0,Z=0,BIG=0;
	uint32_t SelfLADC=0;
	uint8_t SelfGetADCWell=0;
	uint32_t temppp;
	extern uint8_t DMAIndex;
	extern int16_t selfADCValue[12];

PWM_Number CurrentPWM = PWMX; //Ĭ�ϵ�ǰPWMͨ��ΪX
uint32_t S_Last,S_Current,S_History,S_FINAL;
int CurrentThreshold=500;
int CurrentDifference = Default_Difference;//Ĭ��Ӧ��ֵ

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
uint8_t FB_Flag=1;
uint32_t NXSET,NYSET,NZSET,NS_SET,NXYZ_SET;

float SX_RUN,SY_RUN,SZ_RUN,S_RUN_TOTAL;
int32_t CX_RUN,CY_RUN,CZ_RUN,NS_RUN,NXYZ_RUN;
int32_t CX,CY,CZ;

int32_t SCI,SMARK;
int32_t SCI_Min,SCI_Max;
int32_t DX_Data[6];
int16_t DX_Max = 0,DX_Min=1000;
uint8_t DX_Index = 0;
int DX=0;
/***********************************
*FLASH �ֽڶ���
*0x12000032
������32λ��,���4λ����PWMͨ����ѡ��PWMX->1��PWMY->2��PWMZ->4
						 ������4λ����Ӧ��ֵ��ѡ��20->1,50->2,80->4
						 ʣ�µ�24λ�򱣴�����ֵ
						 �磺0x12000032  ������Чͨ��:PWMX    Ӧ��ֵѡ:50   ��ֵ:50
************************************/
uint32_t FLASHData = 0x12000032;
/*----------------------------------��������----------------------------------------*/
/*****************
*
*�����ݴ�����
*
*****************/
void DataProcess(void)
{
	int First_ten_times;
	uint8_t  OUTPin_STATE;
	/*
		FALSH ��ȡ����
	*/
	printFlashTest();
	
	for(First_ten_times = 0;First_ten_times<10;First_ten_times++) /*���ϵ磬ǰʮ��PWMֻ�� RegisterA*/
	{
			//GetRegisterAState();
	}
	
	while(1)
	{
		/*��·�����ж�*/
		//ShortCircuitProtection();
		
		while(ConfirmShortCircuit)
		{
				GPIO_WriteBit(OUT_GPIO_Port, OUT_Pin, Bit_RESET); /*���ֶ�·����OUT��������*/
				if((ShortCircuitLastTime - ShortCircuitTimer)>=1000)
				{
						ConfirmShortCircuit = 0;
						ShortCircuitCounter = 0;
						ShortCircuit=0;
				}
		}
		
		if(KeyIndex<1)   /*С��1��KeyIndex =0 û������Ӧ����KeyIndex>=1ʱ���򰴼���SET����*/
		{
			//RegisterB = Read_GOODBAD();
			/*����FB��ƽ�ߵ��ж�RegisterA*/
			if(FB_Flag ==1)
				CI_GetRegisterAState();
			else if(FB_Flag==0)
				MARK_GetRegisterAState();
			/*ͬ������*/
			//OUT=RegisterA;
			/*���OUT*/
			SetOut(RegisterA);
		}
		
		/*����������ѧϰģʽ*/
		if(KeyTime>0)  
		{
			//printf("first enter ,key time:%d\r\n",KeyTime);
			OUTPin_STATE = GPIO_ReadInputDataBit(OUT_GPIO_Port,OUT_Pin); //��ȡOUT��ֵ,����дFLASHʱ������OUT�����ŵ�ƽ����
			GPIO_WriteBit(OUT_GPIO_Port, OUT_Pin, (BitAction)OUTPin_STATE);
			
			if(GPIO_ReadInputDataBit(FB_GPIO_Port,FB_Pin))
			{
				CI_Mode_SelfLearning();  //CI MODE
				FB_Flag = 1;
			}
			else
			{
				FB_Flag = 0;
				MARK_Mode_SelfLearning();//MARK MODE
			}
		}
	}
}

/********************
*
*�жϳ�CIģʽ��RegisterA״̬
*
**********************/
extern int16_t  RunTime; 
void CI_GetRegisterAState(void)
{
	uint8_t GetADCIndex=0,k;
	if(sample_finish) /*DMA�ж��У�ADCת����ɱ��*/
	{
		if(FB_Flag)  /*CI MODE*/
		{
			for(GetADCIndex=0,k=0;k<4;k++)
			{
				SX[k] = selfADCValue[GetADCIndex++];
				SY[k] = selfADCValue[GetADCIndex++];
				SZ[k] = selfADCValue[GetADCIndex++];	
			}
			SX_RUN = (SX[0]+SX[1]+SX[2]+SX[3])/4; //�ۼ���ƽ��
			SY_RUN = (SY[0]+SY[1]+SY[2]+SY[3])/4;
			SZ_RUN = (SZ[0]+SZ[1]+SZ[2]+SZ[3])/4;
		
			S_RUN_TOTAL = SX_RUN+SY_RUN+SZ_RUN;
			
			CX = 1024*SX_RUN/S_RUN_TOTAL;   //1->SXA,2->SXB
			CY = 1024*SY_RUN/S_RUN_TOTAL;   //1->SXA,2->SXB
			CZ = 1024*SZ_RUN/S_RUN_TOTAL;   //1->SXA,2->SXB
			
			if(S_RUN_TOTAL>SA_B[0])
				NS_RUN = S_RUN_TOTAL - SA_B[0];  /*NSR_RUN = S-SA ����ֵ*/
			else
				NS_RUN = SA_B[0] - S_RUN_TOTAL;
			
			if(CX>CXA_B[0])
				CX_RUN =	CX - CXA_B[0];  /*CX_RUN=CX-CXB�ľ���ֵ*/
			else
				CX_RUN =	CXA_B[0] - CX;
			
			if(CY > CYA_B[0])
				CY_RUN = 	CY - CYA_B[0];
			else
				CY_RUN = 	CYA_B[0] - CY;
			
			if(CZ > CZA_B[0] )
				CZ_RUN = 	CZ - CZA_B[0]; 
			else
				CZ_RUN = 	CZA_B[0] - CZ; 
			
			NXYZ_RUN = CX_RUN+CY_RUN+CZ_RUN;
			
			/************SCI**********/
			SCI = 1000 - (NS_RUN + NXYZ_RUN);  //2018-1-17  change
			if(SCI<=0)
				SCI = 0;
			else if(SCI>=1000)
				SCI  = 1000;
			/************DX**********/
			DX_Data[DX_Index] = SCI;
			if(DX_Data[DX_Index]>DX_Max)
				DX_Max = DX_Data[DX_Index];
			if(DX_Data[DX_Index] < DX_Min)
				DX_Min = DX_Data[DX_Index];
			DX_Index++;
			if(DX_Index>5)
			{
				DX_Index = 0;
				DX = (DX_Max - DX_Min)/4;
				DX_Max = 0;
				DX_Min = 1000;
			}

			/***********RegisterA***********/
			
			SCI_Max = CurrentThreshold + DX;
			SCI_Min = CurrentThreshold-100-DX ; 
			if(SCI_Min<10)
				 SCI_Min= 10;
			
			if(SCI >= SCI_Max)
				RegisterA = 1;
			else if(SCI <= SCI_Min)
				RegisterA = 0;
			
			RunTime = TIM_GetCounter(TIM14);
			
		}
	}
}

/********************
*
*�жϳ�MARKģʽ��RegisterA״̬
*
**********************/
void MARK_GetRegisterAState(void)
{
	uint8_t GetADCIndex=0,k;
	
	RegisterACounter++;// ���ڼ�¼����
	
	if(RegisterACounter<=1) //�����ǰ�ǵ�һ�μ���RegisterA������Ҫ�Ȼ�ȡһ��Signal�����������
	{
		S_Current = Read_Value(CurrentPWM);
		//printf("��һ�ν��� registor A\r\n");
		S_Last = S_Current;
	}
	
	S_Current = Read_Value(CurrentPWM);

	if(sample_finish) /*DMA�ж��У�ADCת����ɱ��*/
	{
		if(FB_Flag)  /*CI MODE*/
		{
			for(GetADCIndex=0,k=0;k<4;k++)
			{
				SX[k] = selfADCValue[GetADCIndex++];
				SY[k] = selfADCValue[GetADCIndex++];
				SZ[k] = selfADCValue[GetADCIndex++];	
			}
			SX_RUN = (SX[0]+SX[1]+SX[2]+SX[3])/4; //�ۼ���ƽ��
			SY_RUN = (SY[0]+SY[1]+SY[2]+SY[3])/4;
			SZ_RUN = (SZ[0]+SZ[1]+SZ[2]+SZ[3])/4;
		
			S_RUN_TOTAL = SX_RUN+SY_RUN+SZ_RUN;
			
			CX = 1024*SX_RUN/S_RUN_TOTAL;   //1->SXA,2->SXB
			CY = 1024*SY_RUN/S_RUN_TOTAL;   //1->SXA,2->SXB
			CZ = 1024*SZ_RUN/S_RUN_TOTAL;   //1->SXA,2->SXB
			
			
			if(S_RUN_TOTAL > SA_B[0])
				NS_RUN = S_RUN_TOTAL - SA_B[0];  /*NSR_RUN = S-SA ����ֵ*/
			else
				NS_RUN = SA_B[0] - S_RUN_TOTAL;
			
			if(CX > CXA_B[0])
				CX_RUN =	CX - CXA_B[0];  /*CX_RUN=CX-CXB�ľ���ֵ*/
			else
				CX_RUN =	CXA_B[0] - CX;
			
			if(CY > CYA_B[0])
				CY_RUN = 	CY - CYA_B[0];
			else
				CY_RUN = 	CYA_B[0] - CY;
			
			if(CZ > CZA_B[0])
				CZ_RUN = 	CZ - CZA_B[0];
			else
				CZ_RUN = 	CZA_B[0] - CZ;
			
			NXYZ_RUN = CX_RUN+CY_RUN+CZ_RUN;
			
			SMARK = 1000 - (NS_RUN + NXYZ_RUN);
			
			if(SMARK<=0)
				SMARK = 0;
			else if(SMARK>=1000)
				SMARK  = 1000;
			
		}		
			//sample_finish = 0;
	}
	
	if(S_FINAL>=CurrentThreshold)
			RegisterA = 1;
	else if(S_FINAL<=(CurrentThreshold - CurrentDifference))
			RegisterA = 0;
		
}

/************************
*
*����PWM1����ѡ��PWMͨ��������ADCֵ
*
*************************/
uint32_t Read_Value(PWM_Number PWM)
{
	/*������Ӧ��PWMͨ��*/
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
	return ADC_value;
}

/***************************************
*
*��ȡKG���뿪�ص�ֵ
*
**************************************/
uint8_t  Read_GOODBAD(void)
{
	
	uint8_t  GOODBAD_STATE;
	
	GOODBAD_STATE = GPIO_ReadInputDataBit(GOODBAD_GPIO_Port,GOODBAD_Pin); //��ȡKG��ֵ
	
	if(GOODBAD_STATE ==Bit_SET)
		return 1;
	else
		return 0;
}

/***************************************
*
*����OUT�������ƽ
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
//		while(sample_finish) /*DMA�ж��У�ADCת����ɱ��*/
//		{
//			*Self_ADC_Value = adc_dma_tab[0];
//			sample_finish = 0;
//			return 1;
//		}
//		return 0;
//}
/***************************************
*
*��ѧϰ����
*
**************************************/
void  MARK_Mode_SelfLearning(void)
{
		uint8_t selfADCIndex=0;
		uint8_t k=0;
		EnterSelfFlag = 1; /*������ѧϰ���λ*/
		DMAIndex=0;
	
		DelaymsSet(500);
	
		if(SelfGetADCWell)   /*��TIM1���ж��У�������3·��������*/
		{
			SelfGetADCWell = 0;
			for(selfADCIndex=0,k=0;k<4;k++)
			{
				SX[k] = selfADCValue[selfADCIndex++];
				SY[k] = selfADCValue[selfADCIndex++];
				SZ[k] = selfADCValue[selfADCIndex++];	
			}
			SXA_B[KeyIndex] = (SX[0]+SX[1]+SX[2]+SX[3])/4; //�ۼ���ƽ��
			SYA_B[KeyIndex] = (SY[0]+SY[1]+SY[2]+SY[3])/4;
			SZA_B[KeyIndex] = (SZ[0]+SZ[1]+SZ[2]+SZ[3])/4;
						
		}; /*�ȴ���ȡ����ADC�ɹ�*/
		
		KeyIndex++;  //��¼�ڼ��ΰ���   1->SXA,2->SXB
		if(KeyIndex>=2) //�ڶ��ΰ���
		{
			//printf("second enter ,key time:%d\r\n",KeyTime);
				KeyIndex = 0;
				/*�����С����ֵ*/
				/*----------PWMX�Աȴ�С--------*/
				if(SXA_B[0]>=SXA_B[1])
				{
						X = (SXA_B[0] - SXA_B[1]);
				}
				else
				{
						X = (SXA_B[1] - SXA_B[0]);
				}
				/*----------PWMY�Աȴ�С--------*/
				if(SYA_B[0]>=SYA_B[1])
				{
						Y = (SYA_B[0] - SYA_B[1]);
				}
				else
				{
						Y = (SYA_B[1] - SYA_B[0]);
				}
				/*----------PWMZ�Աȴ�С--------*/
				if(SZA_B[0]>=SZA_B[1])
				{
						Z = (SZA_B[0] - SZA_B[1]);
				}
				else
				{
						Z = (SZA_B[1] - SZA_B[0]);
				}
				/*�����ֵ,������ֵ���ж�PWMͨ��*/
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
					WriteFlash(0,FLASHData);																	//����FLASH
					//printf("Save Successfully\r\n");
					EnterSelfFlag = 0;
		}
			KeyTime = 0; //����������
}

/*CI_Mode_SelfLearning*/
void CI_Mode_SelfLearning(void)
{
		uint8_t selfADCIndex=0;
		uint8_t k=0;
		EnterSelfFlag = 1; /*������ѧϰ���λ*/
		DMAIndex=0;
	
		while(DMAIndex==0)
		{
			
		}
		if(SelfGetADCWell)
		{
			SelfGetADCWell = 0;
			for(selfADCIndex=0,k=0;k<4;k++)
			{
				SX[k] = selfADCValue[selfADCIndex++];
				SY[k] = selfADCValue[selfADCIndex++];
				SZ[k] = selfADCValue[selfADCIndex++];	
			}
			SXA_B[KeyIndex] = (SX[0]+SX[1]+SX[2]+SX[3])/4; //�ۼ���ƽ��
			SYA_B[KeyIndex] = (SY[0]+SY[1]+SY[2]+SY[3])/4;
			SZA_B[KeyIndex] = (SZ[0]+SZ[1]+SZ[2]+SZ[3])/4;
	 /*�ȴ���ȡ����ADC�ɹ�*/
		
		SA_B[KeyIndex]=SXA_B[KeyIndex]+SYA_B[KeyIndex]+SZA_B[KeyIndex];/*���SA*/
	
		CXA_B[KeyIndex] = 1024*SXA_B[KeyIndex]/SA_B[KeyIndex];   //1->SXA,2->SXB
		CYA_B[KeyIndex] = 1024*SYA_B[KeyIndex]/SA_B[KeyIndex];
		CZA_B[KeyIndex] = 1024*SZA_B[KeyIndex]/SA_B[KeyIndex];
		
		KeyIndex++;  //��¼�ڼ��ΰ���   1->SXA,2->SXB
		
		/****************************CXA,CYA,CZAҪ����FLash********************************/
		
			if(KeyIndex>=2) //�ڶ��ΰ���   //
			{
			//printf("second enter ,key time:%d\r\n",KeyTime);
				KeyIndex = 0;
				
				if(CXA_B[1]>CXA_B[0])
					NXSET =	CXA_B[1] - CXA_B[0];  /*NXSET=CXA-CXB�ľ���ֵ*/
				else
					NXSET =	CXA_B[0] - CXA_B[1];
				
				if(CYA_B[1]>CYA_B[0])
					NYSET = CYA_B[1] - CYA_B[0];
				else
					NYSET = CYA_B[0] - CYA_B[1];
				
				if(CZA_B[1]>CZA_B[0])
					NZSET = CZA_B[1] - CZA_B[0];
				else
					NZSET = CZA_B[0] - CZA_B[1];
				
				if(SA_B[1]>SA_B[0])
					NS_SET = SA_B[1] - SA_B[0];
				else
					NS_SET = SA_B[0] - SA_B[1];
				
				NXYZ_SET = NXSET+NYSET+NZSET;
				
				CurrentThreshold = 1000 - (NS_SET + NXYZ_SET)/2;
				
			if(CurrentThreshold<=200)
					CurrentThreshold = 200;
			else if(CurrentThreshold>=1000)
				CurrentThreshold = 1000;
//				WriteFlash(0,FLASHData);																	//����FLASH
//					//printf("Save Successfully\r\n");
//				EnterSelfFlag = 0;
			}
		}
			KeyTime = 0; //����������
}

/***************************************
*
*ɨ�谴��ʱ��
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
FLASH_Unlock(); //����FLASH��̲���������
FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);//�����־λ
FLASH_ErasePage(FLASH_START_ADDR); //����ָ����ַҳ
FLASH_ProgramWord(FLASH_START_ADDR+(addr*4),data); //��ָ��ҳ��0��ַ��ʼд
FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);//�����־λ
FLASH_Lock(); //����FLASH��̲���������
}

//FLASH��ȡ���ݲ���
uint32_t Flashtemp;
void printFlashTest(void)
{
		uint32_t choose = 0;
		Flashtemp = *(__IO uint32_t*)(FLASH_START_ADDR);
		//printf("addr:0x%x, data:0x%x\r\n", addr, temp);
		choose = Flashtemp & 0xF0000000;
		//DelaymsSet(500);
		/*��ȡPWMͨ��*/
		switch (choose)
		{
			case 0x10000000: CurrentPWM = PWMX;break;
			case 0x20000000: CurrentPWM = PWMY;break;
			case 0x40000000: CurrentPWM = PWMZ;break;
			default : break;
		}
		//printf("CurrentPWM:0x%x,",choose);
		/*Ӧ��ֵ�̶�Ϊ */
		CurrentDifference = Default_Difference;

		//printf("CurrentDifference:0x%x,",choose);
		/*��ȡӦ��ֵ*/
		choose = Flashtemp % 0x01000000;
		CurrentThreshold = choose;
		//printf("CurrentThreshold:0x%x\r\n",choose);

}

/*******************************
*
*��·����
*
*******************************/
void ShortCircuitProtection(void)
{
//	uint8_t SCState;
//	
//	/*��ȡSC���ŵ�״̬*/
//	if(ShortCircuit!=1)
//	{
//		SCState = GPIO_ReadInputDataBit(SC_GPIO_Port ,SC_Pin);
//		if(SCState == Bit_RESET)
//		{
//			/*����FB_SC*/
//			ShortCircuit= 1;
//		}
//		else
//		{
//			ShortCircuit = 0;
//			ConfirmShortCircuit = 0;
//		}
//	}
//	if(ShortCircuit && ShortCircuitCounter>=5)
//	{
//		ConfirmShortCircuit=1;
//		
//		GPIO_WriteBit(OUT_GPIO_Port, OUT_Pin, Bit_RESET);/*��������OUT*/
//		ShortCircuitTimer = ShortCircuitLastTime;
//	}
}

///**** Copyright (C)2017 HarryZeng. All Rights Reserved **** END OF FILE ****/
