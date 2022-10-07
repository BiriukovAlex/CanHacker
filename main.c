// STM32F103C8T6

#include "stm32f10x_conf.h"
#include "stm32f10x.h"

#define ON 1
#define OFF 0
#define LED_PORT              GPIOC
#define LED_PIN               GPIO_Pin_13

#define CAN1_ReMap // Тут сокрыты комменты на раша language 
#ifndef CAN1_ReMap
    #define CAN1_GPIO_PORT        GPIOA
    #define CAN1_RX_SOURCE        GPIO_Pin_11              // RX-ïîðò
    #define CAN1_TX_SOURCE        GPIO_Pin_12              // TX-ïîðò
    #define CAN1_Periph        RCC_APB2Periph_GPIOA        // Ïîðò ïåðèôèðèè
#else
    #define CAN1_GPIO_PORT        GPIOB
    #define CAN1_RX_SOURCE        GPIO_Pin_8               // RX-ïîðò
    #define CAN1_TX_SOURCE        GPIO_Pin_9               // TX-ïîðò
    #define CAN1_Periph        RCC_APB2Periph_GPIOB        // Ïîðò ïåðèôèðèè
#endif

//#define Mode                   CAN_Mode_Silent_LoopBack
#define Mode                   CAN_Mode_Normal
//#define Mode                   CAN_Mode_LoopBack

#define dtread 30
#define dtwrite 1024
#define str 10
char    string[str];
char    datawrite[dtwrite];
uint8_t dataread[dtread];
uint16_t serial_tx_buffer_tail;
uint16_t serial_tx_buffer_head;
uint8_t readcount;
uint8_t FLAG;                       //1-åñòü ïàêåò ïàêåò
                                    //2-interface_state
                                    //4-listen only
                                    //8-time stamp
uint32_t filterID;
uint32_t filterMASK;

CanTxMsg canTX;
CanRxMsg RxMessage;

void pause (uint32_t tick){
  while(tick--){}}

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

void initUART(){
    GPIO_InitTypeDef PORT;    //Ñòðóêòóðà ñîäåðæàùàÿ íàñòðîéêè ïîðòà
    USART_InitTypeDef USART;  //Ñòðóêòóðà ñîäåðæàùàÿ íàñòðîéêè USART
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);

    GPIO_StructInit(&PORT);
    PORT.GPIO_Mode = GPIO_Mode_AF_PP;
    PORT.GPIO_Pin = GPIO_Pin_9; //  Tx
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &PORT);

    PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    PORT.GPIO_Pin = GPIO_Pin_10; //  Rx
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &PORT);

    //Íàñòðîéêà USART
    USART_StructInit(&USART);
    USART.USART_BaudRate = 115200;   //Ñêîðîñòü îáìåíà
    USART.USART_WordLength = USART_WordLength_8b; //Äëèíà ñëîâà 8 áèò
    USART.USART_StopBits = USART_StopBits_1; //1 ñòîï-áèò
    USART.USART_Parity = USART_Parity_No ; //Áåç ïðîâåðêè ÷åòíîñòè
    USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Áåç àïïàðàòíîãî êîíòðîëÿ
    USART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Âêëþ÷åí ïåðåäàò÷èê è ïðèåìíèê USART
    USART_Init(USART1, &USART);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);  //Âêëþ÷àåì UART
  // defaults to 8-bit, no parity, 1 stop bit
}

void serial_write(char data) {
  // Calculate next head
  uint16_t next_head = serial_tx_buffer_head + 1;
  if (next_head == dtwrite) { next_head = 0; }
  while (next_head == serial_tx_buffer_tail) {}
  // Store data and advance head
  datawrite[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);}

//+++++ ÏÎÑÛËÊÀ ÊÎÌÀÍÄÛ ÏÎ USART +++++
void USART_COMM(char *string){
  for (char a=0; a<str; a++){
       if (string[a]==0) break;
       serial_write(string[a]);}
}

//+++++ Ïðåðûâàíèå USART +++++
void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_TC)){
    USART_ClearITPendingBit(USART1, USART_IT_TC);
    uint16_t tail = serial_tx_buffer_tail;
    USART1->DR = datawrite[tail];
    tail++;
    if (tail == dtwrite ) {tail=0;}
    serial_tx_buffer_tail = tail;
    if (tail == serial_tx_buffer_head) {USART_ITConfig(USART1, USART_IT_TC, DISABLE); }}

	if (USART1->SR & USART_SR_RXNE) {
	dataread[readcount]=USART1->DR;
	if (dataread[readcount]==0x0d) FLAG|=1;
	if (readcount<dtread)readcount++;}}

void initCAN(){
        GPIO_InitTypeDef GPIO_InitStructure;

        /* CAN GPIOs configuration */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);        // âêëþ÷àåì òàêòèðîâàíèå AFIO
        RCC_APB2PeriphClockCmd(CAN1_Periph, ENABLE);                // âêëþ÷àåì òàêòèðîâàíèå ïîðòà

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);        // âêëþ÷àåì òàêòèðîâàíèå CAN-øèíû

        // Íàñòðàèâàåì CAN RX pin
        GPIO_InitStructure.GPIO_Pin   = CAN1_RX_SOURCE;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

        // Íàñòðàèâàåì CAN TX pin
        GPIO_InitStructure.GPIO_Pin   = CAN1_TX_SOURCE;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

#ifdef CAN1_ReMap
        GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);          // Ïåðåíîñèì Can1 íà PB8, PB9
#endif
}

void RCC_Config(void){
// Äëÿ íàñòðîéêè CAN â ìàêñèìàëüíîì ðåæèìå ðàáîòû íà ñêîðîñòè äî 1Mb íàì íåîáõîäèìî
// Íàñòðîèòü ÷àñòîòó ïåðåôèðèè APB1 íà 16 MHz

      RCC_ClocksTypeDef RCC_Clocks;
      ErrorStatus HSEStartUpStatus;

      // Ñáðîñèì íàñòðîéêè òàêòèðîâàíèÿ ñèñòåìû
     RCC_DeInit();                                                // RCC system reset

     // Âêëþ÷èì âíåøíèé êâàðö, êàê èñòî÷íèê ñèãíàëà
     RCC_HSEConfig(RCC_HSE_ON);                                   // Enable HSE

     HSEStartUpStatus = RCC_WaitForHSEStartUp();                  // Ïîäîæäåì âêëþ÷åíèÿ HSE

     if (HSEStartUpStatus == SUCCESS){                             // Åñëè âêëþ÷èëñÿ êâàðö

     RCC_HCLKConfig(RCC_SYSCLK_Div1);                         // HCLK = SYSCLK     (64MHz)
     RCC_PCLK1Config(RCC_HCLK_Div4);                          // PCLK1 = HCLK / 8  (16MHz)
     RCC_PCLK2Config(RCC_HCLK_Div1);                          // PCLK2 = HCLK      (64MHz)
     RCC_ADCCLKConfig(RCC_PCLK2_Div2);                        // ADC CLK

     RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_8);     // PLLCLK = 8MHz * 8 = 64 MHz
     RCC_PLLCmd(ENABLE);                                      // Âêëþ÷àåì PLL

     while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}   // Æäåì âêëþ÷åíèÿ PLL

     RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);               // Âûáèðàåì PLL êàê èñòî÷íèê
                                                              // ñèñòåìíîãî òàêòèðîâàíèÿ
     while (RCC_GetSYSCLKSource() != 0x08) {}                 // Æäåì, ïîêà íå óñòàíîâèòñÿ PLL,
                                                              // êàê èñòî÷íèê ñèñòåìíîãî òàêòèðîâàíèÿ
}

    RCC_GetClocksFreq (&RCC_Clocks);
}


void CAN(uint16_t a){
        // Èíèöèàëèçàöèÿ øèíû
        // SamplePoint 87.5
        CAN_InitTypeDef CAN_InitStructure;
switch(a){
        case 1000:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 1;
		break ;
        case 800:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
        CAN_InitStructure.CAN_Prescaler = 2;
		break ;
        case 500:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 2;
		break ;
        case 250:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 4;
		break ;
        case 125:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 8;
		break ;
        case 100:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 10;
		break ;
        case 50:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 20;
		break ;
        case 20:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 50;
		break ;
        case 10:
        CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 100;
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
		USART_COMM("\r");
}

void TIM(uint8_t a){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 64000;
    TIMER_InitStructure.TIM_Period = 60000;
    TIM_TimeBaseInit(TIM1, &TIMER_InitStructure);
    if (a==ON) TIM_Cmd(TIM1, ENABLE);
    if (a==OFF) TIM_Cmd(TIM1, DISABLE);
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

void _initFILTER(uint32_t filterMASK,uint32_t filterID){
        // CAN filter init
        CAN_FilterInitTypeDef CAN_FilterInitStructure;
        CAN_FilterInitStructure.CAN_FilterNumber = 1;
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
        CAN_FilterInitStructure.CAN_FilterIdHigh = (uint16_t) filterID<<16;
        CAN_FilterInitStructure.CAN_FilterIdLow = (uint16_t) filterID ;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (uint16_t) filterMASK<<16;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = (uint16_t) filterMASK;
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

uint8_t halfbyte_to_hexascii(uint8_t _halfbyte){
 _halfbyte &= 0x0F ;
 if(_halfbyte >= 10) return('A' + _halfbyte - 10) ;
	else               return('0' + _halfbyte) ;
}

void USB_LP_CAN1_RX0_IRQHandler(void){
        statLED(1);
        uint16_t stamp=TIM_GetCounter(TIM1);

        if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)         // Ïðîâåðèì ïî÷òîâûé ÿùèê
    {
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    if (RxMessage.RTR==CAN_RTR_Remote){
      if (RxMessage.IDE==CAN_Id_Standard){
      serial_write('r');
      serial_write(halfbyte_to_hexascii((RxMessage.StdId)>>8));
      serial_write(halfbyte_to_hexascii((RxMessage.StdId)>>4));
      serial_write(halfbyte_to_hexascii(RxMessage.StdId));
      serial_write(halfbyte_to_hexascii(RxMessage.DLC));}

      if (RxMessage.IDE==CAN_Id_Extended){
      serial_write('R');
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>28));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>24));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>20));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>16));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>12));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>8));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>4));
      serial_write(halfbyte_to_hexascii(RxMessage.ExtId));
      serial_write(halfbyte_to_hexascii(RxMessage.DLC));}}

      if (RxMessage.RTR==CAN_RTR_DATA){
      if (RxMessage.IDE==CAN_Id_Standard){
      serial_write('t');
      serial_write(halfbyte_to_hexascii((RxMessage.StdId)>>8));
      serial_write(halfbyte_to_hexascii((RxMessage.StdId)>>4));
      serial_write(halfbyte_to_hexascii(RxMessage.StdId));}

      if (RxMessage.IDE==CAN_Id_Extended){
      serial_write('T');
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>28));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>24));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>20));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>16));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>12));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>8));
      serial_write(halfbyte_to_hexascii((RxMessage.ExtId)>>4));
      serial_write(halfbyte_to_hexascii(RxMessage.ExtId));}

      serial_write(halfbyte_to_hexascii(RxMessage.DLC));
      for (uint8_t a=0; a<RxMessage.DLC; a++){
      serial_write(halfbyte_to_hexascii(RxMessage.Data[a]>>4));
      serial_write(halfbyte_to_hexascii(RxMessage.Data[a]));}}

      if ((FLAG&8)==8){
      serial_write(halfbyte_to_hexascii(stamp>>12));
      serial_write(halfbyte_to_hexascii(stamp>>8));
      serial_write(halfbyte_to_hexascii(stamp>>4));
      serial_write(halfbyte_to_hexascii(stamp));}

      serial_write(0x0d);
      }
      statLED(0);}

uint8_t hexascii_to_halfbyte(uint8_t _ascii){
 if((_ascii >= '0') && (_ascii <= '9')) return(_ascii - '0') ;
 if((_ascii >= 'a') && (_ascii <= 'f')) return(_ascii - 'W') ;
 if((_ascii >= 'A') && (_ascii <= 'F')) return(_ascii - '7') ;
 return(0xFF);
}

void main(void){
        RCC_Config();
        initLED();
        initUART();
        initCAN();
        initFILTER();
        initINT();
        statLED(0);

  while(1){   
    if ((FLAG&1)==1){
      switch(dataread[0]){

		case 'V':
		    USART_COMM("V0101\r");
		break ;

		case 'v':
		    USART_COMM("vSTM32\r");
		break ;

        case 'L':
        FLAG|=4;                   //Listen Only
		USART_COMM("\r");
		break ;

        case 'O':
        FLAG|=2;                   //interface_state = 1
        FLAG&=~4;
		USART_COMM("\r");
		break ;

		case 'C':
		FLAG&=~2;                 //interface_state	= 0
		USART_COMM("\r");
		break ;

        case 'Z':
		switch(dataread[1]){
		case	'1':							//Stamp ON
		FLAG|=8;
                TIM(ON);
                USART_COMM("\r");
		break ;

		case	'0':							//Stamp OFF
		FLAG&=~8;
                TIM(OFF);
                USART_COMM("\r");
		break ;

                default :
		FLAG&=~8;
                USART_COMM("\r");
		break ;
		}
        break;

        case 'S':
				switch(dataread[1]){
						case	'0':							//CAN = 10Kbls
							CAN(10);
						break ;

						case	'1':							//CAN = 20Kbls
							CAN(20);
						break ;

						case	'2':							//CAN = 50Kbls
							CAN(50);
						break ;

						case	'3':							//CAN = 100Kbls
							CAN(100);
						break ;

						case	'4':							//CAN = 125Kbls
							CAN(125);
						break ;

						case	'5':							//CAN = 250Kbls
							CAN(250);
						break ;

						case	'6':							//CAN = 500Kbls
							CAN(500);
						break ;

						case	'7':							//CAN = 800Kbls
							CAN(800);
						break ;

						case	'8':							//CAN = 1000Kbls
							CAN(1000);
						break ;
						default :
							CAN(500);
						break ;}
        break;

        case 't':
        canTX.StdId=hexascii_to_halfbyte(dataread[1])<<8|hexascii_to_halfbyte(dataread[2])<<4|hexascii_to_halfbyte(dataread[3]);
        canTX.DLC=hexascii_to_halfbyte(dataread[4]);
        canTX.Data[0]=hexascii_to_halfbyte(dataread[5])<<4|hexascii_to_halfbyte(dataread[6]);
        canTX.Data[1]=hexascii_to_halfbyte(dataread[7])<<4|hexascii_to_halfbyte(dataread[8]);
        canTX.Data[2]=hexascii_to_halfbyte(dataread[9])<<4|hexascii_to_halfbyte(dataread[10]);
        canTX.Data[3]=hexascii_to_halfbyte(dataread[11])<<4|hexascii_to_halfbyte(dataread[12]);
        canTX.Data[4]=hexascii_to_halfbyte(dataread[13])<<4|hexascii_to_halfbyte(dataread[14]);
        canTX.Data[5]=hexascii_to_halfbyte(dataread[15])<<4|hexascii_to_halfbyte(dataread[16]);
        canTX.Data[6]=hexascii_to_halfbyte(dataread[17])<<4|hexascii_to_halfbyte(dataread[18]);
        canTX.Data[7]=hexascii_to_halfbyte(dataread[19])<<4|hexascii_to_halfbyte(dataread[20]);
        canTX.IDE = CAN_Id_Standard;
        canTX.RTR = CAN_RTR_DATA;
        CAN_Transmit(CAN1, &canTX);
		USART_COMM("\r");
        break;

        case 'r':
        canTX.StdId=hexascii_to_halfbyte(dataread[1])<<8|hexascii_to_halfbyte(dataread[2])<<4|hexascii_to_halfbyte(dataread[3]);
        canTX.DLC=hexascii_to_halfbyte(dataread[4]);
        canTX.IDE = CAN_Id_Standard;
        canTX.RTR = CAN_RTR_Remote;
        CAN_Transmit(CAN1, &canTX);
		USART_COMM("\r");
        break;

        case 'R':
        canTX.ExtId=hexascii_to_halfbyte(dataread[1])<<28|hexascii_to_halfbyte(dataread[2])<<24|hexascii_to_halfbyte(dataread[3])<<20|hexascii_to_halfbyte(dataread[4])<<16|hexascii_to_halfbyte(dataread[5])<<12|hexascii_to_halfbyte(dataread[6])<<8|hexascii_to_halfbyte(dataread[7])<<4|hexascii_to_halfbyte(dataread[8]);
        canTX.DLC=hexascii_to_halfbyte(dataread[9]);
        canTX.IDE = CAN_Id_Extended;
        canTX.RTR = CAN_RTR_Remote;
        CAN_Transmit(CAN1, &canTX);
		USART_COMM("\r");
        break;

        case 'T':
        canTX.ExtId=hexascii_to_halfbyte(dataread[1])<<28|hexascii_to_halfbyte(dataread[2])<<24|hexascii_to_halfbyte(dataread[3])<<20|hexascii_to_halfbyte(dataread[4])<<16|hexascii_to_halfbyte(dataread[5])<<12|hexascii_to_halfbyte(dataread[6])<<8|hexascii_to_halfbyte(dataread[7])<<4|hexascii_to_halfbyte(dataread[8]);
        canTX.DLC=hexascii_to_halfbyte(dataread[9]);
        canTX.Data[0]=hexascii_to_halfbyte(dataread[10])<<4|hexascii_to_halfbyte(dataread[11]);
        canTX.Data[1]=hexascii_to_halfbyte(dataread[12])<<4|hexascii_to_halfbyte(dataread[13]);
        canTX.Data[2]=hexascii_to_halfbyte(dataread[14])<<4|hexascii_to_halfbyte(dataread[15]);
        canTX.Data[3]=hexascii_to_halfbyte(dataread[16])<<4|hexascii_to_halfbyte(dataread[17]);
        canTX.Data[4]=hexascii_to_halfbyte(dataread[18])<<4|hexascii_to_halfbyte(dataread[19]);
        canTX.Data[5]=hexascii_to_halfbyte(dataread[20])<<4|hexascii_to_halfbyte(dataread[21]);
        canTX.Data[6]=hexascii_to_halfbyte(dataread[22])<<4|hexascii_to_halfbyte(dataread[23]);
        canTX.Data[7]=hexascii_to_halfbyte(dataread[24])<<4|hexascii_to_halfbyte(dataread[25]);
        canTX.IDE = CAN_Id_Extended;
        canTX.RTR = CAN_RTR_DATA;
        CAN_Transmit(CAN1, &canTX);
		USART_COMM("\r");
        break;

        case 'm':
        filterMASK=hexascii_to_halfbyte(dataread[1])<<28|hexascii_to_halfbyte(dataread[2])<<24|hexascii_to_halfbyte(dataread[3])<<20|hexascii_to_halfbyte(dataread[4])<<16|hexascii_to_halfbyte(dataread[5])<<12|hexascii_to_halfbyte(dataread[6])<<8|hexascii_to_halfbyte(dataread[7])<<4|hexascii_to_halfbyte(dataread[8]);
        if (filterMASK==0xffffffff) initFILTER();
		USART_COMM("\r");
        break;

        case 'M':
        filterID=hexascii_to_halfbyte(dataread[1])<<28|hexascii_to_halfbyte(dataread[2])<<24|hexascii_to_halfbyte(dataread[3])<<20|hexascii_to_halfbyte(dataread[4])<<16|hexascii_to_halfbyte(dataread[5])<<12|hexascii_to_halfbyte(dataread[6])<<8|hexascii_to_halfbyte(dataread[7])<<4|hexascii_to_halfbyte(dataread[8]);
		USART_COMM("\r");
        _initFILTER(filterMASK,filterID);
        break;

        default :
		USART_COMM("\r");
		break ;}
      readcount=0;
      FLAG&=~1;}
    }}
