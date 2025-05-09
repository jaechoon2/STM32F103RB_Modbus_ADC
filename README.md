# STM32F103RB_Modbus_ADC

# STM32F103RB Modbus RTU + ADC Monitoring Project

---

## 📋 프로젝트 개요

이 프로젝트는 STM32F103RB MCU 기반으로  
**ADC 값을 측정하고 Modbus RTU 프로토콜로 외부 시스템에 제공하는** 시스템입니다.

- **ADC1** 채널 0~3 측정 (4개 채널)
- **Modbus RTU** Slave 동작
- **UART1 (9600bps, 8N1)** 시리얼 통신
- **ADC 값 평균 필터(EMA) 적용** (노이즈 제거)
- **PC용 Qt Modbus Master 프로그램과 연동 가능**

---

## 📦 주요 기능

| 기능 | 설명 |
|:---|:---|
| ADC 측정 | ADC1 채널 0~3, 12bit 정밀도 |
| Modbus RTU 응답 | Function Code `0x03 (Read Holding Registers)` 지원 |
| ADC 평균 필터 적용 | Exponential Moving Average (EMA) 방식 |
| UART 통신 | 9600 bps, 8 data bits, No parity, 1 stop bit |
| CubeMX 프로젝트 포함 | `.ioc` 파일 포함, 설정 변경 가능 |

---

## 🛠 하드웨어 환경

| 항목 | 사양 |
|:---|:---|
| MCU | STM32F103RBT6 |
| Clock | HSE 8MHz → System Clock 72MHz 설정 |
| ADC 입력 채널 | ADC1_IN0, ADC1_IN1, ADC1_IN2, ADC1_IN3 |
| UART | USART1 사용 (PA9: TX, PA10: RX) |
| 통신 포맷 | Modbus RTU 표준 (RS-485 권장) |

---

## 🖥 소프트웨어 개발환경

| 항목 | 사용 도구 |
|:---|:---|
| IDE | STM32CubeIDE 1.15.0 이상 |
| SDK | STM32CubeF1 |
| Toolchain | arm-none-eabi-gcc |

---

## 🔥 Modbus 데이터 맵 (Holding Registers)

| 주소 (0x) | 설명 |
|:---|:---|
| 0000 | ADC 채널 0 값 (Filtered) |
| 0001 | ADC 채널 1 값 (Filtered) |
| 0002 | ADC 채널 2 값 (Filtered) |
| 0003 | ADC 채널 3 값 (Filtered) |

---

## 📜 통신 설정 예시 (PC용 Qt 프로그램 기준)

- Slave ID: `1`
- Function: `0x03 Read Holding Registers`
- Start Address: `0x0000`
- Number of Registers: `4`
- Baudrate: `9600`
- Parity: `None`
- Stop Bits: `1`

---

## 🚀 빌드 및 실행 방법

1. STM32CubeIDE에서 본 프로젝트 열기
2. `Project -> Build Project` (Ctrl+B)
3. 보드에 다운로드 (디버거 연결 후 Run)
4. PC 프로그램에서 Modbus Master로 접속하여 값 읽기

---

## 📚 참고 자료

- [Modbus RTU 프로토콜 스펙 (modbus.org)](https://www.modbus.org/specs.php)
- [STMicroelectronics STM32F1 제품군](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series.html)

---

## 🙏 라이센스

본 프로젝트는 자유롭게 수정 및 사용 가능합니다.  
(단, 상업적 사용 시 별도 문의 부탁드립니다.)

---



## 🔄 시스템 데이터 흐름도

```plaintext
+-------------+       UART(Modbus RTU)       +----------------+
| STM32F103RB |  ==========================> | Qt PC Program   |
|   (Slave)   | <==========================   | (Master Mode)   |
|             |    ADC 값 요청 (0x03)          |                |
|  - ADC 측정 |    ADC 값 응답 (4개 채널)       |                |
+-------------+                                +----------------+
