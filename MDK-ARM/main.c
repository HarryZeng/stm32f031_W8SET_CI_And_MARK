#include  "stm32f0xx.h"
#include "stdio.h"
#include "process.h"

#define DMA_BUFFER_SIZE     1  


//  if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)Bit_RESET)
//  {
//    bitstatus = (uint8_t)Bit_SET;
//  }
//  else
//  {
//    bitstatus = (uint8_t)Bit_RESET;
//  }


//if (BitVal != Bit_RESET)
//  {
//    GPIOx->BSRR = GPIO_Pin;
//  }
//  else
//  {
//    GPIOx->BRR = GPIO_Pin ;
//  }

#define _Gpio_12_set  GPIO_WriteBit(GPIOA, GPIO_Pin_12, (BitAction)!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12))
#define _Gpio_7_set  GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7))

uint8_t sample_finish = 0;  
int16_t adc_dma_tab[6] = { 0 };  
uint8_t sample_index = 0;  
uint8_t TIM1step=0;

//?????  
int16_t sample_1[128] = { 0 };  
int16_t sample_2[128] = { 0 };  
int16_t sample_3[128] = { 0 };  
int16_t sample_4[128] = { 0 };  
int16_t sample_5[128] = { 0 };  
int16_t sample_6[128] = { 0 };  
  

RCC_ClocksTypeDef   SysClock;
extern void scan_key(void);
extern uint16_t scan_tick;
extern uint8_t EnterSelfFlag;
extern uint8_t 	ShortCircuitCounter;
extern uint8_t 	ShortCircuit;
void adc_config(void) ;
void user_adc_init(void)  ;
void adc_gpio_init(void);
void adc_dma_init(void) ;
void adc_timer_init(void)  ;
void USART_Config(void);
void GPIOA_Config(void);

void GPIO_DEINIT_ALL(void);
/*****************************************/
///////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  //加入以下代码,支持printf函数,而不需要选择use MicroLIB
  */
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USART1, (unsigned char) ch);
  return (ch);
}

void user_adc_init(void)  
{  
    adc_gpio_init();  
    adc_config();               // ??????????,????????????????????  
    adc_dma_init();             //  
    adc_timer_init();           //  
		GPIOA_Config();
		GPIO_DEINIT_ALL();
    //USART_Config();
}

void USART_Config(void)
{
        USART_InitTypeDef USART_InitStructure;
        /*???USART1??*/
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        USART_InitStructure.USART_BaudRate = 115200;//???????
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//?????
        USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//??????
        USART_InitStructure.USART_Parity = USART_Parity_No;//?????
        USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;//?????
        USART_Init(USART1, &USART_InitStructure);
        USART_Cmd(USART1, ENABLE);//???? 1
}
/*GPIOA????? */
void GPIOA_Config(void)
{
//        GPIO_InitTypeDef GPIO_InitStructure;
//        /*???GPIOA??*/
//        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//        GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
//        GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
//        /* ??PA9 ,PA10*/
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //??????
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
//        GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*?? 1 ????*/
void UART_send_byte(uint8_t byte)
{
        while(!((USART1->ISR)&(1<<7)));//?????
        USART1->TDR=byte; //??????
}
/*?????*/
void UART_Send(uint8_t *Buffer, uint32_t Length)
{
        while(Length != 0)
        {
                while(!((USART1->ISR)&(1<<7)));//?????
                USART1->TDR= *Buffer;
                Buffer++;
                Length--;
        }
}

uint8_t UART_Recive(void)
{
        while(!(USART1->ISR & (1<<5)));
        return(USART1->RDR);
}
  
void adc_config()  
{  
    ADC_InitTypeDef adc_init_structure;  

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);            //??ADC?? 
    
		ADC_DeInit(ADC1);                                               //??ADC  
    ADC_StructInit(&adc_init_structure);                            //???ADC???  
  
    adc_init_structure.ADC_ContinuousConvMode = DISABLE;            //????????  
    adc_init_structure.ADC_DataAlign = ADC_DataAlign_Right;         //???????  
    adc_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; //???????TIM2  
    adc_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//?????  
    adc_init_structure.ADC_Resolution = ADC_Resolution_12b;         //12????  
    adc_init_structure.ADC_ScanDirection = ADC_ScanDirection_Upward;//????0-18??  
		
    ADC_Init(ADC1, &adc_init_structure);  
		
    ADC_OverrunModeCmd(ADC1, ENABLE);                               //????????  
    ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_28_5Cycles);               //??????,????125nS  
    ADC_GetCalibrationFactor(ADC1);                                 //?????ADC  
    ADC_Cmd(ADC1, ENABLE);                                          //??ADC1  
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);         //??ADC1????  
  
    ADC_DMACmd(ADC1, ENABLE);                                       //??ADC_DMA  
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);           //??DMA?????????  
    ADC_StartOfConversion(ADC1);                                    //??????(??)  
}  
  
void adc_gpio_init()  
{  
    GPIO_InitTypeDef gpio_init_structure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);  
  
    GPIO_StructInit(&gpio_init_structure);  
    //GPIOA                                                         //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = (GPIO_Pin_0);  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_AN;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_UP;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure);  
}  
  
void adc_dma_init()  
{  
    DMA_InitTypeDef dma_init_structure;  
    NVIC_InitTypeDef nvic_init_structure;  
  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);              //??DMA??  
  
    nvic_init_structure.NVIC_IRQChannel = DMA1_Channel1_IRQn;       //??DMA1????  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //????  
    nvic_init_structure.NVIC_IRQChannelPriority = 0;                //?????0  
    NVIC_Init(&nvic_init_structure);  
  
    DMA_DeInit(DMA1_Channel1);                                      //复位DMA1_channel1  
    DMA_StructInit(&dma_init_structure);                            //初始化DMA结构体 
  
    dma_init_structure.DMA_BufferSize = DMA_BUFFER_SIZE;            //DMA缓存数组大小  
    dma_init_structure.DMA_DIR = DMA_DIR_PeripheralSRC;             //DMA方向:外设作为数据源 
    dma_init_structure.DMA_M2M = DISABLE;                           //内存到内存禁用
    dma_init_structure.DMA_MemoryBaseAddr = (uint32_t)&adc_dma_tab[0];//缓存数据数组起始地址
    dma_init_structure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//数据大小设置为Halfword  
    dma_init_structure.DMA_MemoryInc = DMA_MemoryInc_Enable;        //内存地址递增 
    dma_init_structure.DMA_Mode = DMA_Mode_Circular;                //DMA循环模式,完成后重新开始覆盖 
    dma_init_structure.DMA_PeripheralBaseAddr = (uint32_t) &(ADC1->DR);//取值的外设地址 
    dma_init_structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设取值大小设置为Halfword  
    dma_init_structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址递增禁用
    dma_init_structure.DMA_Priority = DMA_Priority_High;             //DMA优先级设置为高  
    DMA_Init(DMA1_Channel1, &dma_init_structure);  
  
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);                  //使能DMA中断
    DMA_ClearITPendingBit(DMA_IT_TC);                                //清除一次DMA中断标志  
    DMA_Cmd(DMA1_Channel1, ENABLE);                                  //使能DMA1  
}  
  
void adc_timer_init()  
{  
    TIM_TimeBaseInitTypeDef timer_init_structure;  
		TIM_OCInitTypeDef timer_OCinit_structure; 
    NVIC_InitTypeDef nvic_init_structure;  
  	GPIO_InitTypeDef gpio_init_structure;  
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);            //??TIM2??  //??GPIO??  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  
  
    GPIO_StructInit(&gpio_init_structure);  
    //GPIOA                                                         //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_2;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_AF;                   //????(??)??  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_NOPULL;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure);
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_2);
	
    nvic_init_structure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;                //使能TIM1中断通道  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //使能TIM1中断  
    nvic_init_structure.NVIC_IRQChannelPriority = 2;                //优先级为0  
		
    NVIC_Init(&nvic_init_structure);  
		
		
		nvic_init_structure.NVIC_IRQChannel = TIM2_IRQn;                //使能TIM2中断通道  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //使能TIM2中断  
    nvic_init_structure.NVIC_IRQChannelPriority = 0;                //优先级为0  
		
    NVIC_Init(&nvic_init_structure); 
		
		
		nvic_init_structure.NVIC_IRQChannel = TIM3_IRQn;                //使能TIM3中断通道  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //使能TIM3中断  
    nvic_init_structure.NVIC_IRQChannelPriority = 4;                //优先级为4
		
    NVIC_Init(&nvic_init_structure); 

		/*TIM3*/
		TIM_DeInit(TIM3);                                               //复位TIM3
    TIM_TimeBaseStructInit(&timer_init_structure);                  //初始化TIM结构体  
  
    timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;          //系统时钟,不分频,24M  
    timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //向上计数模式  
    timer_init_structure.TIM_Period = 2000;                          //每300 uS触发一次中断,??ADC  
    timer_init_structure.TIM_Prescaler = 15;                      //计数时钟分频,f=1M,systick=1 uS  
    timer_init_structure.TIM_RepetitionCounter = 0x00;              //发生0+1的update事件产生中断 
		
    TIM_TimeBaseInit(TIM3, &timer_init_structure);  
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                      //使能TIM3中断
    TIM_Cmd(TIM3, ENABLE);                                          //使能TIM3
	
		/*TIM2*/
    TIM_DeInit(TIM2);                                               //复位TIM2  
    TIM_TimeBaseStructInit(&timer_init_structure);                  //初始化TIM结构体  
  
    timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //向上计数模式  
    timer_init_structure.TIM_Period = 120;                          //每300 uS触发一次中断,??ADC  
    timer_init_structure.TIM_Prescaler = 7;                      //计数时钟分频,f=1M,systick=1 uS  
    timer_init_structure.TIM_RepetitionCounter = 0;              //发生0+1的update事件产生中断 
		
    TIM_TimeBaseInit(TIM2, &timer_init_structure);  
  
		timer_OCinit_structure.TIM_OCMode = TIM_OCMode_PWM1;
		timer_OCinit_structure.TIM_OutputState = TIM_OutputState_Enable;
		timer_OCinit_structure.TIM_Pulse = PWM1_HIGH;
		timer_OCinit_structure.TIM_OCPolarity = TIM_OCPolarity_High;
		
		
		TIM_OC3Init(TIM2,&timer_OCinit_structure);
		TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM2,ENABLE);
		
		
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                      //使能TIM2中断
		TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);           //选择TIM1的update为触发源  
		TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
		TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Gated);//触发模式只启动，门控制启停都可以控制
		TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);//主从模式MSM

		TIM_Cmd(TIM2, ENABLE);


/* TIM1 ??? ---------------------------------------------------
   TIM1 ????(TIM1CLK) ??? APB2 ?? (PCLK2)    
    => TIM1CLK = PCLK2 = SystemCoreClock
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock ?48 MHz 
   
   ??????? 4 ?PWM ???17.57 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   ??1??????? 50%
   ??2??????? 37.5%
   ??3??????? 25%
   ??4??????? 12.5%
   ????????????:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	*/  
    /*???????,???????????????*/
  //TimerPeriod = (SystemCoreClock / 17570 ) - 1;
  //TimerPeriod = (SystemCoreClock / DEF_PWMFRE ) - 1;
  //TimerPeriod = (SystemCoreClock / DEF_PWMFRE);
  /* TIM1 ???? */
  
  
  /* Time ??????*/
  timer_init_structure.TIM_Prescaler = 7;
  timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;  /* Time ????????????*/
  timer_init_structure.TIM_Period = 120;
  timer_init_structure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &timer_init_structure);

  /* ??1,2,3,4?PWM ???? */
  timer_OCinit_structure.TIM_OCMode = TIM_OCMode_PWM1;
  timer_OCinit_structure.TIM_OutputState = TIM_OutputState_Enable ;//TIM_OutputState_Enable; //PWM?????
  timer_OCinit_structure.TIM_OutputNState = TIM_OutputNState_Disable ;//TIM_OutputNState_Enable; //??PWM?????
  timer_OCinit_structure.TIM_OCPolarity = TIM_OCPolarity_High;  //PWM 1?????
  timer_OCinit_structure.TIM_OCNPolarity = TIM_OCNPolarity_Low; //PWM?? 0?????
  timer_OCinit_structure.TIM_OCIdleState = TIM_OCIdleState_Set;
  timer_OCinit_structure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  timer_OCinit_structure.TIM_Pulse = 3; //?????
  TIM_OC1Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	timer_OCinit_structure.TIM_Pulse = 3; //?????
  TIM_OC2Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	timer_OCinit_structure.TIM_Pulse = 3; //?????
  TIM_OC3Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);

  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);                      //使能TIM1中断
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	
  /* TIM1 ?????*/
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);							//选择TIM1的timer为触发源  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1Ref);							//选择TIM1的timer为触发源  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC2Ref);							//选择TIM1的timer为触发源  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC3Ref);							//选择TIM1的timer为触发源  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);     //清除update事件中断标志
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);//主从模式MSM  
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	//TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);
	
  TIM_Cmd(TIM1, ENABLE);
	
  /* TIM1 ????? */
  

}  

///*重设TIM1进入OnePule模式，用于控制脉冲个数*/

//void ChangeTIM1ToOnePulse(int Counter)
//{
//	TIM_TimeBaseInitTypeDef timer_init_structure;
//	TIM_Cmd(TIM1, DISABLE);
//	
//	timer_init_structure.TIM_RepetitionCounter = Counter;
//  TIM_TimeBaseInit(TIM1, &timer_init_structure);
//	
//	
//	TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);
//	TIM_Cmd(TIM1, DISABLE);
//}

    
/****************************??????****************************/  

void TIM2_IRQHandler()  
{  
    if(TIM_GetITStatus(TIM2, TIM_FLAG_Update))            //判断发生update事件中断  
    {  
				//_Gpio_7_set;
        TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);     //清除update事件中断标志
    }  
}  

void TIM1_BRK_UP_TRG_COM_IRQHandler()  
{  
    if(TIM_GetITStatus(TIM1, TIM_FLAG_Update))            //判断发生update事件中断  
    {
				if(ShortCircuit)
					ShortCircuitCounter++;
				else
					ShortCircuitCounter=0;
				
				if(EnterSelfFlag)
				{
					TIM1step++;
					if(TIM1step==1)
					{
						PWMX_ON;
						PWMY_OFF;
						PWMZ_OFF;
					}
					else if(TIM1step==2)
					{
						PWMX_OFF;
						PWMY_ON;
						PWMZ_OFF;
					}
					else if(TIM1step==3)
					{
						PWMX_OFF;
						PWMY_OFF;
						PWMZ_ON;
					}
				}
      TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);     //清除update事件中断标志
    }  
} 

extern uint32_t   ShortCircuitLastTime;
void TIM3_IRQHandler()
{
	  if(TIM_GetITStatus(TIM3, TIM_IT_Update))            //判断发生update事件中断  
    { 
			scan_tick++;
			ShortCircuitLastTime++;
			if(scan_tick>=20)  /*20ms*/
			{
				scan_tick = 0;
				scan_key();
			}
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);     //清除update事件中断标志
		}
}


extern uint8_t SelfGetADCWell;
extern uint8_t FB_Flag;
uint8_t ADCIndex=0;
uint8_t DMAIndex=0;

int16_t selfADCValue[12];

void DMA1_Channel1_IRQHandler()  
{  
//		if(DMA_GetITStatus(DMA_IT_HT))
//				;
    if(DMA_GetITStatus(DMA_IT_TC))                      //判断DMA传输完成中断  
    {   
       //sample_1[sample_index] = (adc_dma_tab[0]*3300)/4096;
			
				selfADCValue[ADCIndex++] = adc_dma_tab[0];
			
				if(ADCIndex>=12)
				{
					ADCIndex = 0;
					if(EnterSelfFlag&&(DMAIndex==0))//自学习模式
					{		
							DMAIndex=1;
							SelfGetADCWell=1;
					}
					else													//正常工作模式
					{
						sample_finish = 1; 
					}
					
				}				
				//sample_finish = 1;
    }  
    DMA_ClearITPendingBit(DMA_IT_TC);                   //清楚DMA中断标志位  
}  


void GPIO_INIT(void)
{
	
	    GPIO_InitTypeDef gpio_init_structure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  
  
    GPIO_StructInit(&gpio_init_structure);  
    //GPIOA                                                         //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = GPIO_Pin_12;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_NOPULL;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure);  
	
		gpio_init_structure.GPIO_Pin = GPIO_Pin_4; 
		gpio_init_structure.GPIO_Mode = GPIO_Mode_IN;                   //????(??)??  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
		gpio_init_structure.GPIO_PuPd= GPIO_PuPd_DOWN;                    //??
    GPIO_Init(GPIOA, &gpio_init_structure);  
	
	    //GPIOA                                                         //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_11;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_IN;                   //????(??)??  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_NOPULL;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure);
}

void GPIO_DEINIT_ALL(void)
{
	
	   GPIO_InitTypeDef gpio_init_structure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOF, ENABLE);  
  
    GPIO_StructInit(&gpio_init_structure);  
                                                        //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_DOWN;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure); 
	
    gpio_init_structure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_DOWN;                    //??  
    GPIO_Init(GPIOB, &gpio_init_structure);  

    gpio_init_structure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_DOWN;                    //??  
    GPIO_Init(GPIOF, &gpio_init_structure); 
}


void RCC_Configuration(void)
{
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_4);
	RCC_PLLCmd(ENABLE);
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div2);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET)
	{
		
	}
	
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	
	while(RCC_GetSYSCLKSource()!=0x08)
	{
	}
}

void DelaymsSet(int16_t ms)
{
		while(1)
		{
			ms--;
			if(ms<=0)
				break;
		}
}

int main(void)
{
	RCC_Configuration();
	GPIO_INIT();
	user_adc_init();
	RCC_GetClocksFreq(&SysClock);
	DelaymsSet(5000); 
	
	DataProcess();
	//while(1){}

}


