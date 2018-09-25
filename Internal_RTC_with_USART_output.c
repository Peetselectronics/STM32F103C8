//RTC and time USART oupput by peet's Electronics
//
//Includes
#include "stm32f10x_rtc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_pwr.h"
#include "misc.h"
#include "stdio.h"

__IO uint32_t  TimeVarLock;

void RTC_Configuration(void);
void USART_Configuration();
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void RTC_IRQHandler(void);
void clock_aan();
void Time_Display(uint32_t TimeVar);

int main(void)
{
    //Enable USART1 and GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_Configuration();
	USART_Configuration();
	NVIC_Configuration();
	clock_aan();
	//RTC_Configuration2();
    //Enable the USART1 Receive interrupt: this interrupt is generated when the
    // USART1 receive data register is not empty
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	const unsigned char menu[] = " Start!\r\n";
	UARTSend(menu, sizeof(menu));


    while(1)
    {
      	Time_Display(RTC_GetCounter());
    }
}

void TIMERS_init(){

TIM3->PSC = 23999;	// f timer = fclk / 24000 => 1kHz
TIM3->ARR = 0xFFFF;
TIM3->CR1 = TIM_CR1_CEN;
DBGMCU->CR = DBGMCU_CR_DBG_TIM3_STOP;
}

void Time_Display(uint32_t TimeVar)
{

 if(TimeVarLock != TimeVar){
    uint32_t THH = 0, TMM = 0, TSS = 0;

    //Reset RTC Counter when Time is 23:59:59
    if (RTC_GetCounter() == 0x0001517F)
    {
        RTC_SetCounter(0x0);
        //Wait until last write operation on RTC registers has finished
        RTC_WaitForLastTask();
    }

    //Compute  hours
    THH = TimeVar / 3600;
    // Compute minutes
    TMM = (TimeVar % 3600) / 60;
    // Compute seconds
    TSS = (TimeVar % 3600) % 60;

    char tijd[18];
	sprintf(tijd, "Tijd: %d:%d:%d\r\n", (int)THH, (int)TMM, (int)TSS);
	UARTSend(tijd, sizeof(tijd));
	TimeVarLock = TimeVar;
 }
}
void RTC_Configuration2(void)
{
    //Enable PWR and BKP clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);//Allow access to BKP Domain
    BKP_DeInit();//Reset Backup Domain
    RCC_LSEConfig(RCC_LSE_ON);   // Enable LSE
    //Wait till LSE is ready
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); //Select LSE as RTC Clock Source
    RCC_RTCCLKCmd(ENABLE); //Enable RTC Clock

}

void RTC_Configuration(void)
{
    //Enable PWR and BKP clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);//Allow access to BKP Domain
    BKP_DeInit();// Reset Backup Domain */
    RCC_LSEConfig(RCC_LSE_ON);   // Enable LSE
    BKP_TamperPinCmd(DISABLE);
    // Wait till LSE is ready
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // Select LSE as RTC Clock Source
    RCC_RTCCLKCmd(ENABLE); //Enable RTC Clock
    RTC_WaitForSynchro();//Wait for RTC registers synchronization
    RTC_WaitForLastTask();    // Wait until last write operation on RTC registers has finished
    RTC_ITConfig(RTC_IT_SEC, ENABLE);// Enable the RTC Second */
    RTC_WaitForLastTask();//Wait until last write operation on RTC registers has finished
    //Set RTC prescaler: set RTC period to 1sec
    RTC_SetPrescaler(32767); //RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
    //Wait until last write operation on RTC registers has finished
    RTC_WaitForLastTask();
}

void clock_aan(void){

	 if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
	    {
	        //Backup data register value is not correct or not yet programmed (when
	        //the first time the program is executed)
		 	const unsigned char str1[] = "\r\n\n RTC not yet configured....";
		 	UARTSend(str1, sizeof(str1));

	        //RTC Configuration
	        RTC_Configuration();
		 	const unsigned char str2[] = "\r\n RTC configured....";
		 	UARTSend(str2, sizeof(str2));

	        //Adjust time by values entred by the user on the hyperterminal
	        //Time_Adjust();

	        BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
	    }
	    else
	    {
	        //Check if the Power On Reset flag is set
	        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
	        {
			 	const unsigned char str3[] = "\r\n\n Power On Reset occurred....";
			 	UARTSend(str3, sizeof(str3));
	        }
	        //Check if the Pin Reset flag is set
	        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
	        {
			 	const unsigned char str4[] = "\r\n\n External Reset occurred....";
			 	UARTSend(str4, sizeof(str4));
	        }
		 	const unsigned char str5[] = "\r\n No need to configure RTC....";
		 	UARTSend(str5, sizeof(str5));
	        //Wait for RTC registers synchronization
	        RTC_WaitForSynchro();

	        //Enable the RTC Second
	        RTC_ITConfig(RTC_IT_SEC, ENABLE);
	        //Wait until last write operation on RTC registers has finished
	        RTC_WaitForLastTask();
	    }




    //Enable PWR and BKP clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    // Allow access to BKP Domain
    PWR_BackupAccessCmd(ENABLE);

    //Disable the Tamper Pin
    BKP_TamperPinCmd(DISABLE);
    //To output RTCCLK/64 on Tamper pin, the tamper functionality must be disabled

    //Enable RTC Clock Output on Tamper Pin
    //BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);


    //Clear reset flags
    RCC_ClearFlag();

}
void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE); //Enable USART1
}


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //Configure USART1 Tx (PA.09) as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Configure USART1 Rx (PA.10) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Initialize LED which connected on PB8, Enable the Clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //Configure one bit for preemption priority
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    //Enable the RTC Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //Enable the USARTx Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART1, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        //Loop until the end of transmission
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}

void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        //Clear the RTC Second interrupt
        RTC_ClearITPendingBit(RTC_IT_SEC);

        //Toggle LED
        //Blue pill PC13 led
        GPIOC->ODR ^= GPIO_Pin_13;

        //Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }
}
