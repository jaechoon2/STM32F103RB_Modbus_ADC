/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "modbus.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    if (ch == '\n') {
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 1, 0xFFFF);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);

    return ch;
}

#define SLAVE_ADDRESS 0x01
#define EXPECTED_PACKET_LENGTH 8 //8  // 실제 패킷 길이에 맞게 설정
#define MODBUS_BUFFER_SIZE 256

uint8_t ModbusReceiveBuffer[MODBUS_BUFFER_SIZE];
uint8_t UART1_RxBuffer;
volatile uint16_t ModbusReceiveIndex = 0;
volatile uint16_t uwADCxConvertedValue[11];
volatile uint16_t uwADCxConvertedVals;

uint16_t ModbusCRC(uint8_t *buf, int len);
void ProcessModbusPacket(uint8_t *buf, int len);
uint16_t ReadHoldingRegister(uint16_t address);

#define RS485_TX_RE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define RS485_RX_DE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)


#define MODBUS_MAX_LEN  256
#define MODBUS_SLAVE_ID 0x01

uint8_t tx_buf[MODBUS_MAX_LEN];
uint8_t rx_buf[MODBUS_MAX_LEN];

uint8_t tx_data[] = "RS485 DMA Packet\r\n";
uint8_t rx_buffer[64];

uint8_t rx_data[64];
uint8_t rx_buffer[64];

/* RS485 제어 핀 매크로 */
//#define RS485_TX_ENABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
//#define RS485_RX_ENABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)

void RS485_TX_ENABLE() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
}

void RS485_RX_ENABLE() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {

    	printf("HAL_UART_RxCpltCallback  \r\n");
        // 수신된 데이터를 버퍼에 저장
        ModbusReceiveBuffer[ModbusReceiveIndex++] = UART1_RxBuffer;

        // 패킷 종료 조건 확인 (예: 일정 시간 내에 데이터가 수신되지 않으면 패킷 종료로 간주)
        if (ModbusReceiveIndex >= EXPECTED_PACKET_LENGTH) {
            // 패킷 처리 함수 호출
            ProcessModbusPacket(ModbusReceiveBuffer, ModbusReceiveIndex);
            ModbusReceiveIndex = 0;  // 인덱스 초기화
        }

        // 다음 바이트 수신 준비
        HAL_UART_Receive_IT(&huart1, &UART1_RxBuffer, 1);

    }else if (huart->Instance == USART2) {
    	// 수신된 데이터를 버퍼에 저장
		ModbusReceiveBuffer[ModbusReceiveIndex++] = UART1_RxBuffer;

		// 패킷 종료 조건 확인 (예: 일정 시간 내에 데이터가 수신되지 않으면 패킷 종료로 간주)
		if (ModbusReceiveIndex >= EXPECTED_PACKET_LENGTH) {
			// 패킷 처리 함수 호출
			ProcessModbusPacket(ModbusReceiveBuffer, ModbusReceiveIndex);
			ModbusReceiveIndex = 0;  // 인덱스 초기화
		}

		HAL_UART_Receive_IT(&huart2, &UART1_RxBuffer, 1);
    }
}

uint16_t ModbusCRC(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 1) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void ProcessModbusPacket(uint8_t *buf, int len) {
    // CRC 확인
    uint16_t crc = ModbusCRC(buf, len - 2);
//    if (crc != (buf[len - 2] | (buf[len - 1] << 8))) {
//        // CRC 오류 처리
//        return;
//    }

    // 슬레이브 주소 확인
    uint8_t slaveAddress = buf[0];
    if (slaveAddress != SLAVE_ADDRESS) {
        // 슬레이브 주소 불일치
        return;
    }

    // 함수 코드 확인 및 처리
    uint8_t functionCode = buf[1];
    switch (functionCode) {
        case 0x03:  // Read Holding Registers
            // 시작 주소 및 레지스터 수 읽기
            uint16_t startAddress = (buf[2] << 8) | buf[3];
            uint16_t numRegisters = (buf[4] << 8) | buf[5];

            // 응답 패킷 준비
            uint8_t response[5 + 2 * numRegisters];
            response[0] = slaveAddress;
            response[1] = functionCode;
            response[2] = numRegisters * 2;  // 바이트 수
            for (int i = 0; i < numRegisters; i++) {
            	uint16_t regValue = ReadHoldingRegister(startAddress + i);
                response[3 + i * 2] = regValue >> 8;
                response[4 + i * 2] = regValue & 0xFF;
            }

            // CRC 추가
            crc = ModbusCRC(response, 3 + 2 * numRegisters);
            response[3 + 2 * numRegisters] = crc & 0xFF;
            response[4 + 2 * numRegisters] = crc >> 8;

            // 응답 전송
            HAL_UART_Transmit(&huart1, response, sizeof(response), HAL_MAX_DELAY);
            break;

        // 다른 함수 코드 처리
    }
}

void SendResponsePacket(void)
{
	uint16_t crc = 0xff;
	uint16_t numRegisters = 10;

	// 응답 패킷 준비
	uint8_t response[5 + 2 * numRegisters];
	response[0] = 0x01;//slaveAddress;
	response[1] = 0x10;//functionCode;
	response[2] = numRegisters * 2;  // 바이트 수
	for (int i = 0; i < numRegisters; i++) {
		uint16_t regValue = uwADCxConvertedVals;
	    response[3 + i * 2] = regValue >> 8;
	    response[4 + i * 2] = regValue & 0xFF;
	}

	// CRC 추가
	crc = ModbusCRC(response, 3 + 2 * numRegisters);
	response[3 + 2 * numRegisters] = crc & 0xFF;
	response[4 + 2 * numRegisters] = crc >> 8;

	// 응답 전송
	HAL_UART_Transmit(&huart1, response, sizeof(response), HAL_MAX_DELAY);
}

uint16_t ReadHoldingRegister(uint16_t address) {
    // 실제 레지스터 읽기 로직을 여기에 추가
    // 예제에서는 address에 따라 임의의 값을 반환

	uint16_t regVal = 0;
	regVal = uwADCxConvertedValue[address];
    return regVal;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &UART1_RxBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Relay Off
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Relay Off
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Relay Off
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Relay Off

  uint16_t len = modbus_build_request(0x01, 0x0000, 0x0001, 1, tx_buf);

  RS485_TX_ENABLE();
  HAL_UART_Transmit(&huart1, tx_buf, len, 100);
  HAL_Delay(1000);

  /* USER CODE BEGIN WHILE */

  /* ### - 1 - Initialize ADC peripheral(CubeMX ?��?�� ?��?��) ##################### */

  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start (&hadc1) != HAL_OK)
  {
	  Error_Handler ();
  }

  if (HAL_ADC_Start_IT (&hadc1) != HAL_OK)
  {
     Error_Handler ();
  }


  /* ### - 3 - Channel configuration (MX_ADC1_Init() 처리) ################### */
  while (1)
  {
	  //	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  //	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  //
	  //	  HAL_UART_Receive(&huart1, rx_buf, len, 100);
	  //	  HAL_Delay(500);

//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//
//	  HAL_UART_Transmit(&huart1, tx_buf, len, 100);
//	  HAL_Delay(500);
//
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

	 /* ### - 4 - Start the conversion process ################################*/
	 if (HAL_ADC_Start (&hadc1) != HAL_OK)
	  {
		/* Start Conversation Error */
		Error_Handler ();
	  }

	 for (uint8_t i = 0; i < 10; i++)
	  {
            /* ### - 5 - Wait for the end of conversion ############################*/
            HAL_ADC_PollForConversion (&hadc1, 100);

            /* Check if the continuous conversion of regular channel is finished */
            if ((HAL_ADC_GetState (&hadc1) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
              {
                /* ### - 6 - Get the converted value of regular channel ##############*/
                uwADCxConvertedValue[i] = HAL_ADC_GetValue (&hadc1);
              }
          }

        /* ### - 7 - Stop the conversion process #################################*/
        HAL_ADC_Stop (&hadc1);
        //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        // 500ms 마다 LED 토글
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // PA5 핀에 연결된 LED 제어 (핀 이름은 실제 설정에 맞게 변경)
        HAL_Delay(500); // 500ms 지연


        // 버튼(B1, 예를 들어 PC13)이 눌렸는지 확인 (Pull-up 저항 사용 가정, 누르면 LOW)
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
		{
			printf("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\n", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7] );
			//HAL_UART_Transmit(&huart2, (uint8_t *)"Blue Button Pressed..\r\n", 23, HAL_MAX_DELAY);
			//RS485_TX_ENABLE();
			HAL_UART_Transmit(&huart2, &UART1_RxBuffer, 10, 100);
			//HAL_Delay(500);

			// 버튼 눌렸을 때 처리 (Debouncing은 추가 구현 필요)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED 켜기
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Relay Off

		}else {

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Relay On
		}

        HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RE_Pin|DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RE_Pin DE_Pin */
  GPIO_InitStruct.Pin = RE_Pin|DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *AdcHandle)
{
  /* ### - 6 - Get the converted value of regular channel ##############*/
  uwADCxConvertedVals = HAL_ADC_GetValue (AdcHandle);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
