// STM32F103C8T6

#include "stm32f10x_conf.h"
#include "stm32f10x.h"

#define ON 1
#define OFF 0
#define LED_PORT              GPIOC
#define LED_PIN               GPIO_Pin_13

//#define CAN1_ReMap // Закоментировать, если нет ремапинга портов
#ifndef CAN1_ReMap
    #define CAN1_GPIO_PORT        GPIOA
    #define CAN1_RX_SOURCE        GPIO_Pin_11              // RX-порт
    #define CAN1_TX_SOURCE        GPIO_Pin_12              // TX-порт
    #define CAN1_Periph        RCC_APB2Periph_GPIOA        // Порт перифирии
#else
    #define CAN1_GPIO_PORT        GPIOB
    #define CAN1_RX_SOURCE        GPIO_Pin_8               // RX-порт
    #define CAN1_TX_SOURCE        GPIO_Pin_9               // TX-порт
    #define CAN1_Periph        RCC_APB2Periph_GPIOB        // Порт перифирии
#endif

//#define Mode                   CAN_Mode_Silent_LoopBack
#define Mode                   CAN_Mode_Normal
//#define Mode                   CAN_Mode_LoopBack

CanTxMsg canTX;
CanRxMsg RxMessage;

void pause (uint32_t tick){
  while (tick--){}
}

void initLED(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

void statLED(uint8_t a){
  if (a==1) GPIO_ResetBits(LED_PORT,LED_PIN);
  if (a==0) GPIO_SetBits(LED_PORT,LED_PIN);}

void initCAN(){
        GPIO_InitTypeDef GPIO_InitStructure;

        /* CAN GPIOs configuration */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);        // включаем тактирование AFIO
        RCC_APB2PeriphClockCmd(CAN1_Periph, ENABLE);                // включаем тактирование порта

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);        // включаем тактирование CAN-шины

        // Настраиваем CAN RX pin
        GPIO_InitStructure.GPIO_Pin   = CAN1_RX_SOURCE;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

        // Настраиваем CAN TX pin
        GPIO_InitStructure.GPIO_Pin   = CAN1_TX_SOURCE;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

#ifdef CAN1_ReMap
        GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);          // Переносим Can1 на PB8, PB9
#endif
}

void CAN(uint16_t a){
        // Инициализация шины
        CAN_InitTypeDef CAN_InitStructure;
switch(a){
        case 1000:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
        CAN_InitStructure.CAN_Prescaler = 3;
		break ;
        case 800:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
        CAN_InitStructure.CAN_Prescaler = 3;
		break ;
        case 500:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
        CAN_InitStructure.CAN_Prescaler = 6;
		break ;
        case 250:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
        CAN_InitStructure.CAN_Prescaler = 9;
		break ;
        case 125:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
        CAN_InitStructure.CAN_Prescaler = 18;
		break ;
        case 100:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
        CAN_InitStructure.CAN_Prescaler = 30;
		break ;
        case 50:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
        CAN_InitStructure.CAN_Prescaler = 45;
		break ;
        case 20:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
        CAN_InitStructure.CAN_Prescaler = 150;
		break ;
        case 10:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
        CAN_InitStructure.CAN_Prescaler = 225;
		break ;
}
        // CAN cell init
        CAN_InitStructure.CAN_TTCM = DISABLE;
        CAN_InitStructure.CAN_ABOM = DISABLE;
        CAN_InitStructure.CAN_AWUM = DISABLE;
        CAN_InitStructure.CAN_NART = ENABLE;
        CAN_InitStructure.CAN_RFLM = DISABLE;
        CAN_InitStructure.CAN_TXFP = DISABLE;
        CAN_InitStructure.CAN_Mode = Mode;
        CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
        CAN_Init(CAN1, &CAN_InitStructure);
}

void initFILTER(void){
        // CAN filter init
        CAN_FilterInitTypeDef CAN_FilterInitStructure;
        CAN_FilterInitStructure.CAN_FilterNumber = 1;
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
        CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
        CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
        CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
        CAN_FilterInit(&CAN_FilterInitStructure);
}

void initINT(void){
        // CAN FIFO0 message pending interrupt enable
        CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

        // NVIC Configuration
        // Enable CAN1 RX0 interrupt IRQ channel
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}

void USB_LP_CAN1_RX0_IRQHandler(void){


        if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET){         // Проверим почтовый ящик
            CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        statLED(1);
        pause (100000);
        statLED(0);
      }
      }



void main(void)
{
        initLED();
        initCAN();
        CAN (800);
        initFILTER();
        initINT();
        statLED(0);
 while(1){

        canTX.StdId=0x1ff;
        canTX.DLC=8;
        canTX.Data[0]=1;
        canTX.Data[1]=2;
        canTX.Data[2]=3;
        canTX.Data[3]=4;
        canTX.Data[4]=5;
        canTX.Data[5]=6;
        canTX.Data[6]=7;
        canTX.Data[7]=8;
        canTX.IDE = CAN_Id_Standard;
        canTX.RTR = CAN_RTR_DATA;
        CAN_Transmit(CAN1, &canTX);
        pause (50000);

}
}


