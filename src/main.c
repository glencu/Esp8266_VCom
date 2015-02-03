/**
 ******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    20-September-2012
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F3-Discovery_Demo
 * @{
 */

#define LINEMAX 254 // Maximal allowed/expected line length

volatile char line_buffer[LINEMAX + 1]; // Holding buffer with space for terminating NUL
//volatile int line_valid = 0;
int __errno;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
__IO uint32_t TimingDelay = 0;
__IO uint32_t USBConnectTimeOut = 100;
__IO uint32_t UserButtonPressed = 0;
__IO uint8_t DataReady = 0;

/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t Receive_Buffer[64];
extern __IO uint32_t Receive_length;
uint8_t Send_Buffer[64];
uint32_t packet_sent = 1;
uint32_t packet_receive = 1;

void Delay(__IO uint32_t nTime);
void USART1_IRQHandler(void);


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void) {

	/* SysTick end of count event each 10ms */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 100000);


    /* Configure the USB */
    initUSB();

    initializeUart();


    serial_prints(USART1 , "AT+GMR\r\n");
   	while(1)
   	{
   		Delay(30000);

   		Delay(30000);
   	}
}

void initializeUart()
{

	GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Use the alternative pin functions
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // Setup pull-up resistors
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitTypeDef USART_InitStructure;

	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);

	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;             // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;         // this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // the USART2 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);                       // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART2 peripheral
	USART_Cmd(USART1, ENABLE);
}


void serial_prints (USART_TypeDef* USARTx, volatile char *buffer)
{
    /* transmit till NULL character is encountered */
    while(*buffer)
    {
      USART_SendData(USARTx, *buffer++);
      while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);

      Delay(5);
    }
}

void USART1_IRQHandler(void)
{
  static char rx_buffer[LINEMAX];   // Local holding buffer to build line
  static int rx_index = 0;

  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // Received character?
  {
    char rx =  USART_ReceiveData(USART1);

    if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
    {
        if (rx_index != 0) // Line has some content?
        {
            memcpy((void *)line_buffer, rx_buffer, rx_index); // Copy to static line buffer from dynamic receive buffer
        }
    }
    else
    {
        if (rx_index == LINEMAX) // If overflows pull back to start
            rx_index = 0;

          rx_buffer[rx_index++] = rx; // Copy to buffer and increment
    }
  }
}

void initializeEsp8266pin()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // Use the alternative pin functions
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // Setup pull-up resistors
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
}

/**
 * @brief  Configure the USB.
 * @param  None
 * @retval None
 */
void initUSB(void) {
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();

	USB_Init();

	while ((bDeviceState != CONFIGURED) && (USBConnectTimeOut != 0)) {
	}
}


void Delay(__IO uint32_t nTime) {
	TimingDelay = nTime;

	while (TimingDelay != 0)
		;
}

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void) {
	if (TimingDelay != 0x00) {
		TimingDelay--;
	}
}

