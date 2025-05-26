#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

// 상수 정의
#define FLASH_DATA_SECTOR 1              // 데이터 저장용 SPI 플래시 섹터 (섹터 1)
#define FLASH_METADATA_ADDR ((uint32_t)0x00000000) // 메타데이터 저장 주소 (섹터 0)
#define FLASH_DATA_ADDR ((uint32_t)0x00001000)     // 데이터 저장 시작 주소 (섹터 1)
#define DATA_10S_SIZE sizeof(DataRecord10s)        // 10초 데이터 레코드 크기 (18바이트)
#define DATA_1MIN_SIZE sizeof(DataRecord1min)      // 1분 데이터 레코드 크기 (14바이트)
#define UART_RX_BUFFER_SIZE 32                    // UART 수신 버퍼 크기
#define SPI_RX_BUFFER_SIZE 256                    // SPI 읽기 버퍼 크기 (페이지 단위)
#define BUTTON_DEBOUNCE_MS 50                     // 버튼 디바운싱 시간 (50ms)
#define BUTTON_LONG_PRESS_MS 2000                 // 긴 누름 판단 기준 시간 (2초)
#define INIT_FLAG 0xA5A5A5A5                      // 초기화 플래그 값

// 전역 변수 선언
ADC_HandleTypeDef hadc1, hadc2;                   // ADC1(전압), ADC2(전류) 핸들
DMA_HandleTypeDef hdma_adc1, hdma_adc2;           // ADC DMA 핸들
RTC_HandleTypeDef hrtc;                           // RTC 핸들
UART_HandleTypeDef huart1;                        // UART1 핸들
SPI_HandleTypeDef hspi1;                          // SPI1 핸들
TIM_HandleTypeDef htim2;                          // TIM2 핸들 (버튼 디바운싱/누름 시간)
uint32_t voltage_buffer[128], current_buffer[128]; // ADC 데이터 버퍼
float battery_voltage, battery_current, battery_power; // 배터리 전압(V), 전류(mA), 전력(mW)
float voltage_sum, current_sum, power_sum;         // 1분 평균 계산용 합계
uint32_t sample_count_1min;                       // 1분 샘플 카운트
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];      // UART 수신 버퍼
uint8_t spi_rx_buffer[SPI_RX_BUFFER_SIZE];        // SPI 읽기 버퍼
uint8_t uart_rx_index = 0;                        // UART 수신 버퍼 인덱스
volatile uint32_t button_press_time = 0;          // 버튼 누름 시작 시간
volatile uint8_t button_state = 0;                // 버튼 상태 (0: 대기, 1: 누름, 2: 긴 누름)

// 데이터 구조체 정의
typedef struct {
    uint32_t timestamp;  // RTC 초 단위 타임스탬프
    float voltage;       // 전압 (V)
    float current;       // 전류 (mA)
    float power;         // 전력 (mW)
    uint16_t crc;        // CRC16 체크섬
} DataRecord10s;

typedef struct {
    float avg_voltage;   // 평균 전압 (V)
    float avg_current;   // 평균 전류 (mA)
    float avg_power;     // 평균 전력 (mW)
    uint16_t crc;        // CRC16 체크섬
} DataRecord1min;

typedef struct {
    uint32_t last_write_addr; // 마지막 데이터 기록 주소
    uint32_t sector_index;    // 현재 섹터 인덱스
    uint32_t init_flag;       // 초기화 상태 플래그
} FlashMetadata;

// 함수 선언
void SystemClock_Config(void);
void RTC_Init(void);
void ADC1_Init(void);
void ADC2_Init(void);
void UART1_Init(void);
void SPI1_Init(void);
void TIM2_Init(void);
void GPIO_Button_Init(void);
void LED_Flash(uint8_t pattern);
void Calculate_Voltage_Current(void);
void Write_SPI_Flash(uint32_t addr, void* data, uint32_t size);
void Read_SPI_Flash(uint32_t addr, void* data, uint32_t size);
void Erase_SPI_Flash_Sector(uint32_t sector);
void Erase_SPI_Flash_Chip(void);
void Reset_Data_Log(void);
void Read_Flash_Metadata(FlashMetadata* metadata);
void Write_Flash_Metadata(FlashMetadata* metadata);
void Process_UART_Data(void);
int Validate_DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute);
void Set_RTC(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute);
void Analyze_Battery_Status(void);
uint16_t Calculate_CRC16(void* data, uint32_t size);

/**
 * @brief 메인 함수
 * @details 시스템 초기화, ADC/UART/SPI/TIM/GPIO 설정 후 데이터 기록 및 버튼/UART 명령 처리
 */
int main(void) {
    HAL_Init(); // HAL 라이브러리 초기화
    SystemClock_Config(); // 시스템 클럭 설정
    RTC_Init(); // RTC 초기화
    ADC1_Init(); // ADC1(전압) 초기화
    ADC2_Init(); // ADC2(전류) 초기화
    UART1_Init(); // UART1 초기화
    SPI1_Init(); // SPI1 초기화
    TIM2_Init(); // TIM2 초기화 (버튼 디바운싱)
    GPIO_Button_Init(); // 버튼 및 LED GPIO 초기화

    // 메타데이터 읽기
    FlashMetadata metadata;
    Read_Flash_Metadata(&metadata);
    uint32_t current_write_addr = metadata.last_write_addr;
    // 메타데이터 유효성 확인 및 초기화
    if (current_write_addr == 0xFFFFFFFF || current_write_addr >= FLASH_DATA_ADDR + 252 * 1024 || metadata.init_flag != INIT_FLAG) {
        current_write_addr = FLASH_DATA_ADDR;
        Erase_SPI_Flash_Sector(FLASH_DATA_SECTOR); // 데이터 섹터 초기화
        metadata.last_write_addr = current_write_addr;
        metadata.sector_index = FLASH_DATA_SECTOR;
        metadata.init_flag = INIT_FLAG;
        Write_Flash_Metadata(&metadata);
    }

    // ADC DMA 시작
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)voltage_buffer, 128);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)current_buffer, 128);
    // UART 수신 인터럽트 시작
    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    uint32_t last_10s = 0, last_1min = 0;
    voltage_sum = 0;
    current_sum = 0;
    power_sum = 0;
    sample_count_1min = 0;

    // 메인 루프
    while (1) {
        // 현재 시간 가져오기
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        uint32_t current_time = sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds;

        // 배터리 데이터 계산
        Calculate_Voltage_Current();
        voltage_sum += battery_voltage;
        current_sum += battery_current;
        power_sum += battery_power;
        sample_count_1min++;

        // 10초마다 데이터 기록
        if (current_time >= last_10s + 10) {
            DataRecord10s record = {
                .timestamp = current_time,
                .voltage = battery_voltage,
                .current = battery_current,
                .power = battery_power,
                .crc = Calculate_CRC16(&record, DATA_10S_SIZE - 2)
            };
            Write_SPI_Flash(current_write_addr, &record, DATA_10S_SIZE);
            current_write_addr += DATA_10S_SIZE;
            last_10s = current_time;

            // 디버깅 출력
            printf("10s: Time=%lus, V=%.2fV, I=%.2fmA, P=%.2fmW\n",
                   record.timestamp, record.voltage, record.current, record.power);
        }

        // 1분마다 평균 데이터 기록
        if (current_time >= last_1min + 60) {
            DataRecord1min record = {
                .avg_voltage = voltage_sum / sample_count_1min,
                .avg_current = current_sum / sample_count_1min,
                .avg_power = power_sum / sample_count_1min,
                .crc = Calculate_CRC16(&record, DATA_1MIN_SIZE - 2)
            };
            Write_SPI_Flash(current_write_addr, &record, DATA_1MIN_SIZE);
            current_write_addr += DATA_1MIN_SIZE;
            last_1min = current_time;

            // 디버깅 출력
            printf("1min: Avg V=%.2fV, Avg I=%.2fmA, Avg P=%.2fmW\n",
                   record.avg_voltage, record.avg_current, record.avg_power);

            // 평균 데이터 초기화
            voltage_sum = 0;
            current_sum = 0;
            power_sum = 0;
            sample_count_1min = 0;

            // 데이터 섹터 끝 도달 시 초기화
            if (current_write_addr >= FLASH_DATA_ADDR + 252 * 1024 - DATA_10S_SIZE) {
                current_write_addr = FLASH_DATA_ADDR;
                Erase_SPI_Flash_Sector(FLASH_DATA_SECTOR);
            }

            // 메타데이터 업데이트
            metadata.last_write_addr = current_write_addr;
            metadata.sector_index = FLASH_DATA_SECTOR;
            metadata.init_flag = INIT_FLAG;
            Write_Flash_Metadata(&metadata);
        }

        HAL_Delay(100); // 100ms 대기
    }
}

/**
 * @brief 시스템 클럭 설정
 * @details HSE(8MHz)와 PLL로 400MHz 시스템 클럭 설정, RTC 클럭 활성화
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // HSE 및 LSE 활성화, PLL 설정
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RTC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // 시스템 클럭, AHB/APB 설정
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

    __HAL_RCC_RTC_ENABLE(); // RTC 클럭 활성화
}

/**
 * @brief RTC 초기화
 * @details LSE(32.768kHz)로 RTC 설정, 기본 시간(2025/05/26 00:00:00) 설정
 */
void RTC_Init(void) {
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    HAL_RTC_Init(&hrtc);

    // 초기 시간 설정
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    sTime.Hours = 0;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sDate.Year = 25;
    sDate.Month = 5;
    sDate.Date = 26;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

/**
 * @brief UART1 초기화
 * @details 115200 baud, PA9(TX), PA10(RX), 인터럽트 활성화
 */
void UART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE(); // USART1 클럭 활성화

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    // UART GPIO 설정
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // UART 인터럽트 활성화
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief SPI1 초기화
 * @details W25Q128용 SPI1 설정, PA5(SCK), PA6(MISO), PA7(MOSI), PB0(CS)
 */
void SPI1_Init(void) {
    __HAL_RCC_SPI1_CLK_ENABLE(); // SPI1 클럭 활성화

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);

    // SPI GPIO 설정
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // CS 핀 설정
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // CS 비활성화
}

/**
 * @brief TIM2 초기화
 * @details 버튼 디바운싱 및 긴 누름 감지를 위한 1ms 틱 타이머
 */
void TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE(); // TIM2 클럭 활성화

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (SystemCoreClock / 1000) - 1; // 1ms 틱
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    // 타이머 인터럽트 활성화
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief 버튼 및 LED GPIO 초기화
 * @details PB1(버튼, EXTI1, 풀업), PC13(LED, 오픈 드레인)
 */
void GPIO_Button_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 버튼: PB1, 내부 풀업, EXTI1
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // LED: PC13, 오픈 드레인, 액티브 로우
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED 끄기

    // EXTI 인터럽트 활성화
    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/**
 * @brief LED 점멸
 * @param pattern 1: 빠른 점멸(진행 중), 2: 느린 점멸(완료)
 */
void LED_Flash(uint8_t pattern) {
    if (pattern == 1) { // 빠른 점멸 (100ms 간격, 4회)
        for (int i = 0; i < 4; i++) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED 켜기
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED 끄기
            HAL_Delay(100);
        }
    } else if (pattern == 2) { // 느린 점멸 (500ms 간격, 2회)
        for (int i = 0; i < 2; i++) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_Delay(500);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_Delay(500);
        }
    }
}

/**
 * @brief 배터리 전압/전류/전력 계산
 * @details ADC 데이터 평균화 후 전압(V), 전류(mA), 전력(mW) 계산
 */
void Calculate_Voltage_Current(void) {
    uint32_t voltage_sum = 0, current_sum = 0;
    for (int i = 0; i < 128; i++) {
        voltage_sum += voltage_buffer[i];
        current_sum += current_buffer[i];
    }
    uint32_t voltage_avg = voltage_sum / 128;
    uint32_t current_avg = current_sum / 128;

    battery_voltage = (voltage_avg * 3.3 / 65535.0) / 0.6; // 전압(V)
    battery_current = (current_avg * 3.3 / 65535.0) / 0.1 * 1000; // 전류(mA)
    battery_power = battery_voltage * (battery_current / 1000) * 1000; // 전력(mW)
}

/**
 * @brief SPI 플래시 쓰기
 * @param addr 쓰기 시작 주소
 * @param data 데이터 포인터
 * @param size 데이터 크기
 */
void Write_SPI_Flash(uint32_t addr, void* data, uint32_t size) {
    uint8_t cmd[4];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // CS 활성화

    // 쓰기 활성화
    cmd[0] = 0x06; // WREN
    HAL_SPI_Transmit(&hspi1, cmd, 1, 100);

    // 페이지 프로그램
    cmd[0] = 0x02; // Page Program
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)data, size, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // CS 비활성화

    // 쓰기 완료 대기
    do {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        cmd[0] = 0x05; // RDSR
        HAL_SPI_Transmit(&hspi1, cmd, 1, 100);
        HAL_SPI_Receive(&hspi1, cmd, 1, 100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    } while (cmd[0] & 0x01); // WIP 비트 확인
}

/**
 * @brief SPI 플래시 읽기
 * @param addr 읽기 시작 주소
 * @param data 데이터 저장 포인터
 * @param size 읽기 크기
 */
void Read_SPI_Flash(uint32_t addr, void* data, uint32_t size) {
    uint8_t cmd[4];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    cmd[0] = 0x03; // Read Data
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Receive(&hspi1, (uint8_t*)data, size, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
 * @brief SPI 플래시 섹터 지우기
 * @param sector 지울 섹터 번호
 */
void Erase_SPI_Flash_Sector(uint32_t sector) {
    uint8_t cmd[4];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    cmd[0] = 0x06; // WREN
    HAL_SPI_Transmit(&hspi1, cmd, 1, 100);

    cmd[0] = 0x20; // Sector Erase
    cmd[1] = (sector * 4096 >> 16) & 0xFF;
    cmd[2] = (sector * 4096 >> 8) & 0xFF;
    cmd[3] = 0;
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // 지우기 완료 대기
    do {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        cmd[0] = 0x05; // RDSR
        HAL_SPI_Transmit(&hspi1, cmd, 1, 100);
        HAL_SPI_Receive(&hspi1, cmd, 1, 100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    } while (cmd[0] & 0x01);
}

/**
 * @brief 전체 SPI 플래시 지우기
 * @details W25Q128 칩 전체 지우기, 완료 후 메타데이터 초기화
 */
void Erase_SPI_Flash_Chip(void) {
    uint8_t cmd[1];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    cmd[0] = 0x06; // WREN
    HAL_SPI_Transmit(&hspi1, cmd, 1, 100);

    cmd[0] = 0xC7; // Chip Erase
    HAL_SPI_Transmit(&hspi1, cmd, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    LED_Flash(1); // 진행 중 빠른 점멸
    // 지우기 완료 대기
    do {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        cmd[0] = 0x05; // RDSR
        HAL_SPI_Transmit(&hspi1, cmd, 1, 100);
        HAL_SPI_Receive(&hspi1, cmd, 1, 100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    } while (cmd[0] & 0x01);

    LED_Flash(2); // 완료 시 느린 점멸
    HAL_UART_Transmit(&huart1, (uint8_t*)"SPI Flash chip erased\n", 22, 100);

    // 메타데이터 초기화
    FlashMetadata metadata = {
        .last_write_addr = FLASH_DATA_ADDR,
        .sector_index = FLASH_DATA_SECTOR,
        .init_flag = INIT_FLAG
    };
    Write_Flash_Metadata(&metadata);
}

/**
 * @brief 데이터 로그 초기화
 * @details 데이터 섹터 지우고 기록 포인터 재설정
 */
void Reset_Data_Log(void) {
    LED_Flash(1); // 진행 중 빠른 점멸
    Erase_SPI_Flash_Sector(FLASH_DATA_SECTOR);
    FlashMetadata metadata = {
        .last_write_addr = FLASH_DATA_ADDR,
        .sector_index = FLASH_DATA_SECTOR,
        .init_flag = INIT_FLAG
    };
    Write_Flash_Metadata(&metadata);
    LED_Flash(2); // 완료 시 느린 점멸
    HAL_UART_Transmit(&huart1, (uint8_t*)"Data log reset\n", 15, 100);
}

/**
 * @brief 메타데이터 읽기
 * @param metadata 메타데이터 구조체 포인터
 */
void Read_Flash_Metadata(FlashMetadata* metadata) {
    Read_SPI_Flash(FLASH_METADATA_ADDR, metadata, sizeof(FlashMetadata));
}

/**
 * @brief 메타데이터 쓰기
 * @param metadata 메타데이터 구조체 포인터
 */
void Write_Flash_Metadata(FlashMetadata* metadata) {
    Erase_SPI_Flash_Sector(0);
    Write_SPI_Flash(FLASH_METADATA_ADDR, metadata, sizeof(FlashMetadata));
}

/**
 * @brief CRC16 계산
 * @param data 데이터 포인터
 * @param size 데이터 크기
 * @return CRC16 값
 */
uint16_t Calculate_CRC16(void* data, uint32_t size) {
    uint16_t crc = 0xFFFF;
    uint8_t* buffer = (uint8_t*)data;
    for (uint32_t i = 0; i < size; i++) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // CRC-16-IBM
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief UART 데이터 처리
 * @details "SET_TIME" 및 "GET_BATTERY_STATUS" 명령 처리
 */
void Process_UART_Data(void) {
    if (uart_rx_buffer[uart_rx_index] == '\n' || uart_rx_buffer[uart_rx_index] == '\r') {
        uart_rx_buffer[uart_rx_index] = '\0';
        if (strncmp((char*)uart_rx_buffer, "SET_TIME ", 9) == 0) {
            uint16_t year;
            uint8_t month, day, hour, minute;
            // 날짜/시간 파싱
            if (sscanf((char*)uart_rx_buffer, "SET_TIME %hu/%hhu/%hhu %hhu:%hhu",
                       &year, &month, &day, &hour, &minute) == 5) {
                if (Validate_DateTime(year, month, day, hour, minute)) {
                    Set_RTC(year, month, day, hour, minute);
                    HAL_UART_Transmit(&huart1, (uint8_t*)"RTC set successfully\n", 21, 100);
                } else {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid date/time\n", 18, 100);
                }
            } else {
                HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid format\n", 15, 100);
            }
        } else if (strcmp((char*)uart_rx_buffer, "GET_BATTERY_STATUS") == 0) {
            Analyze_Battery_Status(); // 배터리 상태 분석
        } else {
            HAL_UART_Transmit(&huart1, (uint8_t*)"Unknown command\n", 16, 100);
        }
        uart_rx_index = 0;
        memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
    } else {
        uart_rx_index++;
        if (uart_rx_index >= UART_RX_BUFFER_SIZE) {
            uart_rx_index = 0;
            memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
            HAL_UART_Transmit(&huart1, (uint8_t*)"Buffer overflow\n", 16, 100);
        }
    }
    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);
}

/**
 * @brief 날짜/시간 유효성 검증
 * @param year 연도 (2000~2099)
 * @param month 월 (1~12)
 * @param day 일 (1~31)
 * @param hour 시 (0~23)
 * @param minute 분 (0~59)
 * @return 1: 유효, 0: 무효
 */
int Validate_DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute) {
    if (year < 2000 || year > 2099 || month < 1 || month > 12 ||
        day < 1 || day > 31 || hour > 23 || minute > 59) {
        return 0;
    }

    uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2) {
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
            days_in_month[1] = 29; // 윤년 처리
        }
    }
    if (day > days_in_month[month - 1]) {
        return 0;
    }

    return 1;
}

/**
 * @brief RTC 시간 설정
 * @param year 연도
 * @param month 월
 * @param day 일
 * @param hour 시
 * @param minute 분
 */
void Set_RTC(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = hour;
    sTime.Minutes = minute;
    sTime.Seconds = 0;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    sDate.Year = year - 2000;
    sDate.Month = month;
    sDate.Date = day;
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

/**
 * @brief 배터리 상태 분석
 * @details SPI 플래시 데이터 읽고 총 에너지, 평균 전압/전류/전력 계산
 */
void Analyze_Battery_Status(void) {
    FlashMetadata metadata;
    Read_Flash_Metadata(&metadata);
    uint32_t current_addr = FLASH_DATA_ADDR;
    uint32_t end_addr = metadata.last_write_addr;
    if (end_addr < FLASH_DATA_ADDR) end_addr = FLASH_DATA_ADDR;

    float total_energy_wh = 0;
    float voltage_sum = 0, current_sum = 0, power_sum = 0;
    uint32_t valid_records = 0;
    uint32_t start_time = 0xFFFFFFFF, end_time = 0;
    uint32_t error_count = 0;

    // 데이터 순차적 읽기
    while (current_addr < end_addr) {
        Read_SPI_Flash(current_addr, spi_rx_buffer, SPI_RX_BUFFER_SIZE);
        for (uint32_t i = 0; i < SPI_RX_BUFFER_SIZE && current_addr < end_addr; ) {
            if (current_addr + DATA_10S_SIZE <= end_addr && i + DATA_10S_SIZE <= SPI_RX_BUFFER_SIZE) {
                DataRecord10s* record = (DataRecord10s*)(spi_rx_buffer + i);
                uint16_t calc_crc = Calculate_CRC16(record, DATA_10S_SIZE - 2);
                if (calc_crc == record->crc && record->timestamp != 0xFFFFFFFF) {
                    voltage_sum += record->voltage;
                    current_sum += record->current;
                    power_sum += record->power;
                    total_energy_wh += (record->power / 1000.0) * (10.0 / 3600.0); // mW to Wh
                    if (record->timestamp < start_time) start_time = record->timestamp;
                    if (record->timestamp > end_time) end_time = record->timestamp;
                    valid_records++;
                } else {
                    error_count++;
                }
                i += DATA_10S_SIZE;
                current_addr += DATA_10S_SIZE;
            } else if (current_addr + DATA_1MIN_SIZE <= end_addr && i + DATA_1MIN_SIZE <= SPI_RX_BUFFER_SIZE) {
                DataRecord1min* record = (DataRecord1min*)(spi_rx_buffer + i);
                uint16_t calc_crc = Calculate_CRC16(record, DATA_1MIN_SIZE - 2);
                if (calc_crc == record->crc) {
                    voltage_sum += record->avg_voltage;
                    current_sum += record->avg_current;
                    power_sum += record->avg_power;
                    total_energy_wh += (record->avg_power / 1000.0) * (60.0 / 3600.0);
                    valid_records++;
                } else {
                    error_count++;
                }
                i += DATA_1MIN_SIZE;
                current_addr += DATA_1MIN_SIZE;
            } else {
                break;
            }
        }
    }

    // 결과 출력
    char output[128];
    if (valid_records > 0) {
        uint32_t start_hour = start_time / 3600;
        uint32_t start_min = (start_time % 3600) / 60;
        uint32_t start_sec = start_time % 60;
        uint32_t end_hour = end_time / 3600;
        uint32_t end_min = (end_time % 3600) / 60;
        uint32_t end_sec = end_time % 60;

        snprintf(output, sizeof(output),
                 "Battery Status: %04lu/%02lu/%02lu %02lu:%02lu:%02lu to %04lu/%02lu/%02lu %02lu:%02lu:%02lu, "
                 "Total Energy=%.2fWh, Avg V=%.2fV, Avg I=%.2fmA, Avg P=%.2fmW, Errors=%lu\n",
                 2000 + (start_time / (3600 * 24 * 365)) % 100, (start_time / (3600 * 24)) % 12 + 1, (start_time / (3600 * 24)) % 30 + 1,
                 start_hour, start_min, start_sec,
                 2000 + (end_time / (3600 * 24 * 365)) % 100, (end_time / (3600 * 24)) % 12 + 1, (end_time / (3600 * 24)) % 30 + 1,
                 end_hour, end_min, end_sec,
                 total_energy_wh, voltage_sum / valid_records, current_sum / valid_records, power_sum / valid_records,
                 error_count);
    } else {
        snprintf(output, sizeof(output), "No valid data found\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), 100);
}

/**
 * @brief UART 수신 콜백
 * @param huart UART 핸들
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        Process_UART_Data();
    }
}

/**
 * @brief UART 에러 콜백
 * @param huart UART 핸들
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_rx_index = 0;
        memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
        HAL_UART_Transmit(&huart1, (uint8_t*)"UART error\n", 12, 100);
        HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);
    }
}

/**
 * @brief 버튼 EXTI 콜백
 * @param GPIO_Pin 인터럽트 발생 핀
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1 && button_state == 0) {
        button_state = 1;
        button_press_time = HAL_GetTick();
        HAL_TIM_Base_Start(&htim2); // 타이머 시작
    }
}

/**
 * @brief 타이머 주기 콜백
 * @param htim 타이머 핸들
 * @details 버튼 디바운싱 및 긴 누름 감지
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        static uint32_t last_check = 0;
        uint32_t current_time = HAL_GetTick();

        if (button_state == 1 && (current_time - button_press_time) >= BUTTON_DEBOUNCE_MS) {
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
                // 버튼 누름 유지
                if ((current_time - button_press_time) >= BUTTON_LONG_PRESS_MS) {
                    button_state = 2;
                    Erase_SPI_Flash_Chip(); // 긴 누름: 전체 초기화
                    HAL_TIM_Base_DeInit(&htim2);
                }
            } else {
                // 버튼 떼짐
                if (button_state == 1 && (current_time - button_press_time) < BUTTON_LONG_PRESS_MS) {
                    Reset_Data_Log(); // 짧은 누름: 데이터 로그 초기화
                }
                button_state = 0;
                HAL_TIM_Base_DeInit(&htim2);
            }
        }
    }
}

/**
 * @brief ADC 변환 완료 콜백
 * @param hadc ADC 핸들
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == &hadc1 || hadc == &hadc2) {
        // ADC 변환 완료
    }
}