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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "modbus.h" // modbus.h 포함 확인
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

/* USER CODE BEGIN PV */
#define SLAVE_ADDRESS 0x01
#define MODBUS_BUFFER_SIZE 256

uint8_t ModbusReceiveBuffer[MODBUS_BUFFER_SIZE];
uint8_t UART1_RxBuffer; // 1바이트 수신용 버퍼
volatile uint16_t ModbusReceiveIndex = 0;
volatile bool ModbusFrameReceived = false; // 프레임 수신 완료 플래그

// ADC 관련 변수
volatile uint16_t uwADCxConvertedValue[10]; // ADC 값 저장 (예: 10개)
volatile uint16_t uwADCxConvertedVals; // 단일 변환 값 (필요시 사용)

// 릴레이 GPIO 핀 정의 (가독성 향상)
#define RELAY1_PORT GPIOC
#define RELAY1_PIN GPIO_PIN_6
#define RELAY2_PORT GPIOC
#define RELAY2_PIN GPIO_PIN_7
#define RELAY3_PORT GPIOC
#define RELAY3_PIN GPIO_PIN_8
#define RELAY4_PORT GPIOC
#define RELAY4_PIN GPIO_PIN_9

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t ModbusCRC(uint8_t *buf, int len);
void ProcessModbusPacket(uint8_t *buf, int len);
uint16_t ReadHoldingRegister(uint16_t address);
// WriteSingleRegister 함수의 반환 타입을 void로 변경
void WriteSingleRegister(uint16_t address, uint16_t value);

#define MODBUS_MAX_LEN  256
#define MODBUS_SLAVE_ID 0x01

uint8_t tx_buf[MODBUS_MAX_LEN];
uint8_t rx_buf[MODBUS_MAX_LEN];

uint8_t tx_data[] = "RS485 DMA Packet\r\n";
uint8_t rx_buffer[64];

uint8_t rx_data[64];
uint8_t rx_buffer[64];
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
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r", 1, 0xFFFF);
    }
    HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 0xFFFF);

    return ch;
}
// ... (printf 관련 코드 유지)

// --- RS485 제어 매크로 ---
// DE와 /RE 핀이 어떻게 연결되었는지 확인 필요.
// 일반적으로 DE와 /RE를 묶어서 하나의 핀으로 제어하는 경우가 많음 (예: PB13)
// 여기서는 DE(PB13), /RE(PB14) 개별 제어로 가정
void RS485_TX_ENABLE() {
    // 전송 시작 전 DE High, /RE High (트랜시버에 따라 다를 수 있음, 데이터시트 확인!)
    HAL_GPIO_WritePin(GPIOB, DE_Pin, GPIO_PIN_RESET);     // DE High
    HAL_GPIO_WritePin(GPIOB, RE_Pin, GPIO_PIN_SET);     // /RE High (보통 DE와 반대거나 같음)
    // 짧은 지연시간 (필요시)
    // for(volatile int i=0; i<10; i++); // 매우 짧은 지연, 클럭 속도에 따라 조절
}

void RS485_RX_ENABLE() {
    // 전송 완료 후 또는 평상시 DE Low, /RE Low
    // for(volatile int i=0; i<10; i++); // 전송 완료 보장 위한 짧은 지연 (필요시)
    HAL_GPIO_WritePin(GPIOB, DE_Pin, GPIO_PIN_SET);   // DE Low
    HAL_GPIO_WritePin(GPIOB, RE_Pin, GPIO_PIN_RESET);   // /RE Low
}


// --- UART 수신 콜백 (Modbus T3.5 타이머 연동) ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 수신 버퍼 오버플로우 방지
        if (ModbusReceiveIndex < MODBUS_BUFFER_SIZE) {
            ModbusReceiveBuffer[ModbusReceiveIndex++] = UART1_RxBuffer;

            // T3.5 타이머 리셋 및 시작 (문자 수신 시마다 재시작)
            __HAL_TIM_SET_COUNTER(&htim2, 0); // 카운터 초기화
            HAL_TIM_Base_Start_IT(&htim2);    // 타이머 시작 (One Pulse Mode 권장)
        } else {
            // 버퍼 오버플로우 처리 (예: 인덱스 초기화)
            ModbusReceiveIndex = 0;
            // 에러 로그 등
        }

        // 다음 1 바이트 수신 준비
        HAL_UART_Receive_IT(&huart1, &UART1_RxBuffer, 1);

    }
    // else if (huart->Instance == USART2) {
    //     // USART2 수신 처리 (printf용이면 보통 불필요)
    // }
}

// --- T3.5 타이머 인터럽트 콜백 ---
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // T3.5 시간 동안 추가 수신이 없었음 -> 프레임 종료
        HAL_TIM_Base_Stop_IT(&htim2); // 타이머 중지

        if (ModbusReceiveIndex > 0) { // 수신된 데이터가 있을 경우만
             ModbusFrameReceived = true; // 메인 루프에서 처리하도록 플래그 설정
        }
    }
}

// --- CRC 계산 함수 (기존 코드 유지) ---
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
    // Modbus RTU는 LSB first, MSB last
    // return crc; // 이대로 반환하면 LSB가 먼저 나옴
     return (crc >> 8) | (crc << 8); // SWAP bytes for correct comparison/transmission
}

// --- Modbus 패킷 처리 함수 ---
void ProcessModbusPacket(uint8_t *buf, int len) {
    // 1. 최소 길이 확인 (SlaveID + FuncCode + CRC = 4 bytes)
    if (len < 4) {
        return; // 너무 짧은 패킷
    }

    // 2. 슬레이브 주소 확인
    uint8_t slaveAddress = buf[0];
    if (slaveAddress != SLAVE_ADDRESS) {
        return; // 내 주소가 아님
    }

    // 3. CRC 확인 (수정: CRC 체크 활성화 및 바이트 순서 고려)
    uint16_t received_crc = (buf[len - 1] << 8) | buf[len - 2]; // MSB first in calculation
    uint16_t calculated_crc = ModbusCRC(buf, len - 2);
    // calculated_crc = (calculated_crc >> 8) | (calculated_crc << 8); // SWAP bytes

    if (received_crc != calculated_crc) {
         // CRC 오류 처리 (예: 에러 로그, 무시)
         printf("CRC Error! Received: 0x%04X, Calculated: 0x4X\r\n", received_crc, calculated_crc);
         return;
    }


    // 4. 함수 코드 확인 및 처리
    uint8_t functionCode = buf[1];
    uint8_t response[MODBUS_BUFFER_SIZE]; // 응답 버퍼
    int responseLength = 0;
    uint16_t startAddress, numRegisters, regValue, writeValue, crc;

    switch (functionCode) {
        case 0x03: // Read Holding Registers
            if (len != 8) return; // FC03 요청 길이는 8바이트
            startAddress = (buf[2] << 8) | buf[3];
            numRegisters = (buf[4] << 8) | buf[5];

            // 주소 및 개수 유효성 검사 (예: ADC 배열 범위 확인)
            if ((startAddress + numRegisters) > (sizeof(uwADCxConvertedValue) / sizeof(uwADCxConvertedValue[0]))) {
                // 예외 처리: Illegal Data Address
                response[0] = slaveAddress;
                response[1] = functionCode | 0x80; // 예외 응답 코드
                response[2] = 0x02; // 예외 코드: Illegal Data Address
                responseLength = 3;
            } else {
                // 정상 응답 준비
                response[0] = slaveAddress;
                response[1] = functionCode;
                response[2] = numRegisters * 2; // 바이트 수
                responseLength = 3;
                for (int i = 0; i < numRegisters; i++) {
                    regValue = ReadHoldingRegister(startAddress + i);
                    response[responseLength++] = regValue >> 8;   // MSB
                    response[responseLength++] = regValue & 0xFF; // LSB
                }
            }
            break;

        case 0x06: // Write Single Register
            if (len != 8) return; // FC06 요청 길이는 8바이트
            startAddress = (buf[2] << 8) | buf[3]; // 레지스터 주소 (릴레이 번호)
            writeValue = (buf[4] << 8) | buf[5];   // 쓸 값 (ON/OFF)

            // 주소 유효성 검사 (1~4번 릴레이 가정)
            if (startAddress < 1 || startAddress > 4) {
                 // 예외 처리: Illegal Data Address
                response[0] = slaveAddress;
                response[1] = functionCode | 0x80;
                response[2] = 0x02;
                responseLength = 3;
            } else {
                 // 레지스터 쓰기 (실제 릴레이 제어)
                 WriteSingleRegister(startAddress, writeValue);

                 // 정상 응답 (요청을 그대로 반향)
                 for(int i=0; i<len-2; i++) { // CRC 제외하고 복사
                     response[i] = buf[i];
                 }
                 responseLength = len - 2; // CRC 제외 길이
            }
            break;

        // --- 다른 필요한 Function Code 처리 추가 ---
        case 0x10: // Write Multiple Registers
             // ... 구현 ...
             break;

        default:
            // 지원하지 않는 함수 코드 예외 처리
            response[0] = slaveAddress;
            response[1] = functionCode | 0x80;
            response[2] = 0x01; // 예외 코드: Illegal Function
            responseLength = 3;
            break;
    }

    // 5. 응답 전송 (예외 또는 정상 응답)
    if (responseLength > 0) {
        crc = ModbusCRC(response, responseLength);
        response[responseLength++] = crc & 0xFF;       // CRC LSB
        response[responseLength++] = (crc >> 8) & 0xFF; // CRC MSB (ModbusCRC 함수 수정으로 인해 순서 변경됨)

        RS485_TX_ENABLE(); // 송신 모드 설정
        HAL_Delay(1); // DE 핀 안정화 시간 (필요시, 매우 짧게)
        HAL_UART_Transmit(&huart1, response, responseLength, HAL_MAX_DELAY); // 블로킹 방식 전송

        // HAL_UART_Transmit_IT 또는 HAL_UART_Transmit_DMA 사용 시 주의:
        // 전송 완료 콜백(HAL_UART_TxCpltCallback)에서 RS485_RX_ENABLE() 호출 필요

        // 전송 완료 후 수신 모드 설정 (블로킹 함수 뒤에 호출)
        // 전송 버퍼가 비워지는 것을 기다리는 약간의 지연이 필요할 수 있음
        while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY); // 간단한 대기 (더 정확한 방법 고려)
        RS485_RX_ENABLE();
    }
}

// --- Holding Register 읽기 함수 ---
uint16_t ReadHoldingRegister(uint16_t address) {
    // 주소 유효성 검사 (0부터 시작하는 인덱스 가정)
    if (address < (sizeof(uwADCxConvertedValue) / sizeof(uwADCxConvertedValue[0]))) {
        return uwADCxConvertedValue[address];
    }
    // 다른 레지스터 영역 처리 (필요시)
    return 0; // 잘못된 주소면 0 반환
}

// --- Single Register 쓰기 함수 (릴레이 제어) ---
void WriteSingleRegister(uint16_t address, uint16_t value) {
    GPIO_PinState pinState = (value == 0xFF00) ? GPIO_PIN_RESET : GPIO_PIN_SET; // 0xFF00: ON(LOW), 0x0000: OFF(HIGH) - 릴레이 특성에 맞게 조절

    switch (address) {
        case 1: // 릴레이 1 주소
            HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, pinState);
            break;
        case 2: // 릴레이 2 주소
            HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, pinState);
            break;
        case 3: // 릴레이 3 주소
            HAL_GPIO_WritePin(RELAY3_PORT, RELAY3_PIN, pinState);
            break;
        case 4: // 릴레이 4 주소
            HAL_GPIO_WritePin(RELAY4_PORT, RELAY4_PIN, pinState);
            break;
        default:
            // 잘못된 주소
            break;
    }
    // 다른 레지스터 영역 처리 (필요시)
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
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // 초기 릴레이 상태 설정 (OFF)
  HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY3_PORT, RELAY3_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY4_PORT, RELAY4_PIN, GPIO_PIN_SET);

  RS485_RX_ENABLE(); // 초기 상태는 수신 모드

  // Modbus 수신 시작 (첫 1 바이트)
  HAL_UART_Receive_IT(&huart1, &UART1_RxBuffer, 1);

  // ADC 설정
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
      Error_Handler();
  }
  // ADC 인터럽트 방식 사용 시 시작 (콜백에서 값 읽기)
  // if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) { Error_Handler(); }
  // 또는 폴링 방식 사용 시 아래 루프 내에서 처리

   uint16_t len = modbus_build_request(0x01, 0x0000, 0x0001, 1, tx_buf);
   RS485_TX_ENABLE();
   HAL_UART_Transmit(&huart1, tx_buf, len, 100);
   HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // --- Modbus 프레임 처리 ---
    if (ModbusFrameReceived) {
        ProcessModbusPacket(ModbusReceiveBuffer, ModbusReceiveIndex);
        ModbusReceiveIndex = 0;      // 인덱스 초기화
        ModbusFrameReceived = false; // 플래그 리셋
        // 수신 재시작 (ProcessModbusPacket 에서 송신 후 수신 모드로 전환하므로 불필요할 수 있음)
        // 하지만 안전을 위해 추가 고려: HAL_UART_Receive_IT(&huart1, &UART1_RxBuffer, 1);
    }


    // --- ADC 값 읽기 (폴링 방식 예시) ---
    // ADC 인터럽트 방식을 사용하면 이 부분은 필요 없음
//	 HAL_ADC_Start(&hadc1);
//	 for (uint8_t i = 0; i < (sizeof(uwADCxConvertedValue) / sizeof(uwADCxConvertedValue[0])); i++) {
//		 // ADC 채널 변경 로직 필요 시 추가 (CubeMX에서 Scan 모드 설정 시 자동)
//		 HAL_ADC_PollForConversion(&hadc1, 10); // 타임아웃 10ms
//		 if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
//			 uwADCxConvertedValue[i] = HAL_ADC_GetValue(&hadc1);
//		 } else {
//			 // ADC 읽기 오류 처리
//		 }
//	 }
//	 HAL_ADC_Stop(&hadc1);

	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // PA5 핀에 연결된 LED 제어 (핀 이름은 실제 설정에 맞게 변경)
	 HAL_Delay(500); // 500ms 지연

	 // 버튼(B1, 예를 들어 PC13)이 눌렸는지 확인 (Pull-up 저항 사용 가정, 누르면 LOW)
	 if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	 {
		 printf("Blue Button Pressed...\r\n");

		// 버튼 눌렸을 때 처리 (Debouncing은 추가 구현 필요)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED 켜기
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // Relay Off

	 }else {

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Relay On
	 }


    // --- 기타 작업 ---
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // LD2 토글 (하트비트)
    HAL_Delay(500); // 메인 루프 지연 (다른 작업에 따라 조절)

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

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
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
