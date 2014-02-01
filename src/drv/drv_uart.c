#include "board.h"
/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////
// UART1 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define UART1_TX_PIN        GPIO_Pin_6
#define UART1_RX_PIN        GPIO_Pin_7
#define UART1_GPIO          GPIOB
#define UART1_TX_PINSOURCE  GPIO_PinSource6
#define UART1_RX_PINSOURCE  GPIO_PinSource7

#define UART1_BUFFER_SIZE   2048

// Receive buffer, circular DMA
volatile uint8_t rx1Buffer[UART1_BUFFER_SIZE];
uint32_t rx1DMAPos = 0;

volatile uint8_t tx1Buffer[UART1_BUFFER_SIZE];
volatile uint16_t tx1BufferTail = 0;
volatile uint16_t tx1BufferHead = 0;

volatile uint8_t  tx1DmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// UART1 Transmit via DMA
///////////////////////////////////////////////////////////////////////////////

static void uart1TxDMA(void)
{
	if ((tx1DmaEnabled == true) || (tx1BufferHead == tx1BufferTail))  // Ignore call if already active or no new data in buffer
        return;

    DMA2_Stream7->M0AR = (uint32_t)&tx1Buffer[tx1BufferTail];

    if (tx1BufferHead > tx1BufferTail)
    {
	    DMA_SetCurrDataCounter(DMA2_Stream7, tx1BufferHead - tx1BufferTail);
	    tx1BufferTail = tx1BufferHead;
    }
    else
    {
	    DMA_SetCurrDataCounter(DMA2_Stream7, UART1_BUFFER_SIZE - tx1BufferTail);
	    tx1BufferTail = 0;
    }

    tx1DmaEnabled = true;

    DMA_Cmd(DMA2_Stream7, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// UART1 TX Complete Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void DMA2_Stream7_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);

    tx1DmaEnabled = false;

    uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Initialization
///////////////////////////////////////////////////////////////////////////////

enum { expandEvr = 0 };

/*void telemetryListenerCB(evr_t e)
{
    if (expandEvr)
        telemetryPrintF("EVR-%s %8.3fs %s (%04x)\n", evrToSeverityStr(e.evr), (float)e.time/1000., evrToStr(e.evr), e.reason);
    else
        telemetryPrintF("EVR:%08x %04x %04x\n", e.time, e.evr, e.reason);
}*/

///////////////////////////////////////

void telemetryInit(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    USART_StructInit(&USART_InitStructure);
    DMA_StructInit(&DMA_InitStructure);

    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,   ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = UART1_TX_PIN | UART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PINSOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PINSOURCE, GPIO_AF_USART1);

    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = baudrate;
  //USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  //USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  //USART_InitStructure.USART_Parity              = USART_Parity_No;
  //USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  //USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART_InitStructure);

    // Receive DMA into a circular buffer

    DMA_DeInit(DMA2_Stream5);

    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx1Buffer;
  //DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = UART1_BUFFER_SIZE;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  //DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  //DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream5, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    rx1DMAPos = DMA_GetCurrDataCounter(DMA2_Stream5);

    // Transmit DMA
    DMA_DeInit(DMA2_Stream7);

  //DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
  //DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)tx1Buffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  //DMA_InitStructure.DMA_BufferSize         = UART_BUFFER_SIZE;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  //DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  //DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  //DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  //DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    DMA_SetCurrDataCounter(DMA2_Stream7, 0);

    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART1, ENABLE);

    //evrRegisterListener(telemetryListenerCB);
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Available
///////////////////////////////////////////////////////////////////////////////

uint16_t telemetryAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA2_Stream5) != rx1DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Read
///////////////////////////////////////////////////////////////////////////////

uint8_t telemetryRead(void)
{
    uint8_t ch;

    ch = rx1Buffer[UART1_BUFFER_SIZE - rx1DMAPos];
    // go back around the buffer
    if (--rx1DMAPos == 0)
	    rx1DMAPos = UART1_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t telemetryReadPoll(void)
{
    while (!telemetryAvailable()); // wait for some bytes
    return telemetryRead();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Write
///////////////////////////////////////////////////////////////////////////////

void telemetryWrite(uint8_t ch)
{
    tx1Buffer[tx1BufferHead] = ch;
    tx1BufferHead = (tx1BufferHead + 1) % UART1_BUFFER_SIZE;

    uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Print
///////////////////////////////////////////////////////////////////////////////

void telemetryPrint(char *str)
{
    while (*str)
    {
    	tx1Buffer[tx1BufferHead] = *str++;
    	tx1BufferHead = (tx1BufferHead + 1) % UART1_BUFFER_SIZE;
    }

	uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Print Formatted - Print formatted string to Telemetry Port
// From Ala42
///////////////////////////////////////////////////////////////////////////////

/*void telemetryPrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	telemetryPrint(buf);
	va_end(vlist);
}*/

///////////////////////////////////////////////////////////////////////////////		
		

/* -------------------------- UART2  ----------------------------- */
uartReceiveCallbackPtr uart2Callback = NULL;
#define UART2_BUFFER_SIZE    1024

volatile uint8_t tx2Buffer[UART2_BUFFER_SIZE];
uint32_t tx2BufferTail = 0;
uint32_t tx2BufferHead = 0;
bool uart2RxOnly = false;

static void uart2Open(uint32_t speed)
{
    USART_InitTypeDef USART_InitStructure;

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | (uart2RxOnly ? 0 : USART_Mode_Tx);
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
}

void uart2Init(uint32_t speed, uartReceiveCallbackPtr func, bool rxOnly)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    uart2RxOnly = rxOnly;

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2_TX    PD5
    // USART2_RX    PD6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    if (!rxOnly)
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6,GPIO_AF_USART2);

    uart2Open(speed);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    if (!rxOnly)
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    USART_Cmd(USART2, ENABLE);
		uart2Callback = func;
}

void uart2ChangeBaud(uint32_t speed)
{
    uart2Open(speed);
}

void uart2Write(uint8_t ch)
{
    if (uart2RxOnly)
        return;
				
		tx2Buffer[tx2BufferHead] = ch;
    tx2BufferHead = (tx2BufferHead + 1) % UART2_BUFFER_SIZE;

    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
		//USART_ITConfig(USART2, USART_IT_TC, ENABLE);
}

/*void uart2Write_(uint8_t ch){

		// wait until data register is empty
		while( !(USART2->SR & 0x00000040) ); 
		USART_SendData(USART2, ch);
				
}*/

bool isUart2TransmitEmpty(void)
{
		if(tx2BufferTail != tx2BufferHead) {
			//if (USART_GetITStatus(USART2, USART_IT_TC) == SET) {
				USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
			//}	
			return false;
		}
		return true; //tx2BufferTail == tx2BufferHead;
}

void USART2_IRQHandler(void)
{
    uint16_t SR = USART2->SR;

    //if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
		if (SR & USART_IT_RXNE) {
        if (uart2Callback)
            uart2Callback(USART_ReceiveData(USART2));
    }
    
		//while(USART_GetITStatus(USART2, USART_IT_TXE) == RESET); 
		
		//if (USART_GetITStatus(USART2, USART_IT_TXE) == SET) {
		if (SR & USART_FLAG_TXE) {
        if (tx2BufferTail != tx2BufferHead) {
            
						USART2->DR = tx2Buffer[tx2BufferTail];
            tx2BufferTail = (tx2BufferTail + 1) % UART2_BUFFER_SIZE;
        } else {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
		
		/*if (USART_GetITStatus(USART2, USART_IT_TC) != RESET)
    {
        // Очищаем флаг прерывания 
        USART_ClearITPendingBit(USART2, USART_IT_TC);
 
        // Отправляем байт данных
        USART_SendData(USART2, tx2Buffer[tx2BufferTail]);
 
        // Увеличиваем счетчик отправленных байт
        tx2BufferTail = (tx2BufferTail + 1) % UART2_BUFFER_SIZE;
 
    }*/
}


/* -------------------------- UART3  ----------------------------- */

uartReceiveCallbackPtr uart3Callback = NULL;
#define UART3_BUFFER_SIZE    256

volatile uint8_t tx3Buffer[UART3_BUFFER_SIZE];
uint32_t tx3BufferTail = 0;
uint32_t tx3BufferHead = 0;
bool uart3RxOnly = false;

static void uart3Open(uint32_t speed)
{
    USART_InitTypeDef USART_InitStructure;

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | (uart3RxOnly ? 0 : USART_Mode_Tx);
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
}

void uart3Init(uint32_t speed, uartReceiveCallbackPtr func, bool rxOnly)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    uart3RxOnly = rxOnly;

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART3_TX    PD8
    // USART3_RX    PD9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    if (!rxOnly)
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9,GPIO_AF_USART2);

    uart3Open(speed);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    if (!rxOnly)
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    USART_Cmd(USART3, ENABLE);
		uart3Callback = func;
}

void uart3ChangeBaud(uint32_t speed)
{
    uart3Open(speed);
}

void uart3Write(uint8_t ch)
{
    if (uart3RxOnly)
        return;

    tx3Buffer[tx3BufferHead] = ch;
    tx3BufferHead = (tx3BufferHead + 1) % UART3_BUFFER_SIZE;

    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

bool isUart3TransmitEmpty(void)
{
    return tx3BufferTail == tx3BufferHead;
}

void USART3_IRQHandler(void)
{
    uint16_t SR = USART3->SR;

    if (SR & USART_IT_RXNE) {
        if (uart3Callback)
            uart3Callback(USART_ReceiveData(USART3));
    }
    if (SR & USART_FLAG_TXE) {
        if (tx3BufferTail != tx3BufferHead) {
            USART3->DR = tx3Buffer[tx3BufferTail];
            tx3BufferTail = (tx3BufferTail + 1) % UART3_BUFFER_SIZE;
        } else {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}

