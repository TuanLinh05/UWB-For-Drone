/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c (TAG)
  * @brief          : TAG SS-TWR Ranging — Interrupt-driven State Machine @ 50Hz
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "dw1000_hw.h"
#include "tag_ranging.h"
#include "uart_tx.h"
#include "telemetry.h"
#include <string.h>
#include <stdio.h>

/* USER CODE BEGIN PV */

/* DW1000 device ID (read at startup, visible in Live Expressions) */
volatile uint32_t device_id = 0;

/* Per-anchor distances — declared in tag_ranging.c, extern here for debugger */
extern volatile int32_t distance_a1_mm;
extern volatile int32_t distance_a2_mm;
extern volatile int32_t distance_a3_mm;
extern volatile int32_t distance_a4_mm;

extern volatile int32_t distance_a1_filtered_mm;
extern volatile int32_t distance_a2_filtered_mm;
extern volatile int32_t distance_a3_filtered_mm;
extern volatile int32_t distance_a4_filtered_mm;

/* Debug counters (Live Expressions) */
volatile uint32_t ranging_count = 0;
volatile uint32_t timeout_count = 0;

/* Phase 2: kết quả readback PHY (0 = khớp). Live Expressions / boot log. */
volatile uint32_t dw1000_config_mismatch = 0;

/* Phase 3: nhiệt độ die DW1000 (raw SAR byte). Live Expressions. */
volatile uint8_t dw1000_temp_raw = 0;

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* Phase 2: Independent Watchdog — tự reset nếu vòng lặp treo (an toàn bay).
 * Dùng thanh ghi trực tiếp để không phụ thuộc module HAL_IWDG (không có trong
 * Drivers của project). LSI ~40kHz / 32 = 1250Hz, reload 250 → ~200ms. */
#define USE_IWDG        1
#define IWDG_REFRESH()  (IWDG->KR = 0x0000AAAAU)

/* USER CODE END PV */

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
#if USE_IWDG
static void MX_IWDG_Init(void);
#endif

/* printf được chuyển hướng qua ring buffer non-blocking (UART_TX).
   Không còn HAL_MAX_DELAY trên đường chạy — sửa F-05. */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  uint8_t c = (uint8_t)ch;
  UART_TX_Write(&c, 1);
  return ch;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  UART_TX_Write((const uint8_t *)ptr, (uint16_t)len);
  return len;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* Bật UART TX non-blocking TRƯỚC mọi printf (banner cũng đi qua ring). */
  UART_TX_Init(&huart1);

  /* Phase 2: đồng hồ µs (DWT) cho timeout chính xác. */
  MCU_TimerInit();

#if TELEM_ASCII
  /* In binary mode USART1 is a strict framed telemetry stream.  Human-readable
     boot logs on the same UART can contain 0xAA 0x55 by chance and make the
     ESP32 parser report a false CRC error.  Keep logs only for the explicit
     ASCII bring-up mode. */
  printf("\r\n================================\r\n");
  printf("  UWB TAG (ADDR = 0x%04X)\r\n", TAG_ADDR);
  printf("  IRQ State-Machine @ 50Hz\r\n");
  printf("================================\r\n");
#endif

  /* ---- Boot blink: 5 quick flashes ----------------------------------- */
  /* LED GPIO is initialized inside DW1000_GPIO_Init() → DW1000_Init().  */
  /* We do an early blink here using direct register writes to confirm   */
  /* the MCU is alive before DW1000 init.                                */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  {
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    for (int i = 0; i < 5; i++) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); HAL_Delay(80);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   HAL_Delay(80);
    }
  }

  /* ---- DW1000 Init --------------------------------------------------- */
  if (Tag_Init() != 0)
  {
    /* Fatal: DW1000 not responding. Fast blink forever. */
    device_id = 0xDEAD;
    while (1) { HAL_GPIO_TogglePin(LED_PORT, LED_PIN); HAL_Delay(200); }
  }

  device_id = DW1000_ReadDeviceID();
#if TELEM_ASCII
  printf("DW1000 SPI runtime: %u MHz\r\n", (unsigned)dw1000_spi_mhz);
#endif

  /* ---- Phase 2: readback verify PHY config --------------------------- */
  dw1000_config_mismatch = DW1000_VerifyConfig();
#if TELEM_ASCII
  printf("PHY verify: 0x%02lX %s\r\n", (unsigned long)dw1000_config_mismatch,
         dw1000_config_mismatch == 0 ? "(OK)" : "(MISMATCH!)");
#endif

  /* ---- Phase 3: đọc nhiệt độ die DW1000 lúc radio còn idle ----------- */
  dw1000_temp_raw = DW1000_ReadTemperatureRaw();
#if TELEM_ASCII
  printf("DW1000 temp raw: %u (~%d C)\r\n",
         dw1000_temp_raw, (int)((dw1000_temp_raw - 0x76) * 1.14f + 23.0f));
#endif

  /* ---- Arm EXTI on PA3 (DW1000 IRQ pin) ------------------------------ *
   * Tag_Init() calls DW1000_Configure() which sets SYS_MASK, then        *
   * DW1000_ClearAllStatus() which drives IRQ pin LOW.                    *
   * Only now is it safe to arm the rising-edge EXTI.                     */
  DW1000_EnableIRQ();

  /* Publish the active calibration/ranging profile. It is also repeated with
   * stats below so clients that connect after boot still receive it. */
  Telem_SendInfo();

  /* ---- Phase 2: bật watchdog SAU khi init xong (tránh reset sớm) ------ */
#if USE_IWDG
  MX_IWDG_Init();
#endif

  /* ---- Main loop: purely non-blocking -------------------------------- *
   * - Mỗi chu kỳ A1→A2→A3→A4 hoàn tất → 1 gói telemetry.                *
   * - Sample mới nhận biết qua cờ tag_cycle_ready (không so sánh giá trị *
   *   khoảng cách — sửa F-10).                                           *
   * - ~1Hz gửi 1 gói thống kê (counters + tần số thực đo được).         *
   * - Toàn bộ output qua UART ring buffer, không hàm blocking.          */
    uint32_t last_stats_tick  = HAL_GetTick();
    uint32_t last_ops_count   = 0;
    uint32_t last_cycle_count = 0;

    while (1)
    {
        /* Tag_Task() drives the full A1→A2→A3→A4 state machine. */
        Tag_Task();

        /* Phase 2 (F): IRQ của DW1000 là tín hiệu MỨC. Nếu còn HIGH mà EXTI
         * cạnh lên đã lỡ event thứ hai, ép xử lý tiếp ở vòng sau. */
        if (DW1000_IrqLineActive() && !dw1000_irq_flag)
            dw1000_irq_flag = 1;

#if USE_IWDG
        IWDG_REFRESH();   /* kick watchdog mỗi vòng */
#endif

        /* Một chu kỳ vừa hoàn tất → xuất một gói range (~50Hz). */
        if (tag_cycle_ready)
        {
            tag_cycle_ready = 0;

            TagCycleSnapshot_t snap;
            Tag_GetSnapshot(&snap);
            Telem_SendRangeCycle(&snap);
        }

        /* Sync counters cho Live Expressions */
        const DW1000_RangingResult *r = Tag_GetResult();
        ranging_count = r->ranging_count;
        timeout_count = r->timeout_count;

        /* Gói thống kê ~1Hz — đo tần số chu kỳ/phép đo thực tế. */
        uint32_t now = HAL_GetTick();
        if (now - last_stats_tick >= 1000)
        {
            uint16_t ops_hz = (uint16_t)(response_ok_count - last_ops_count);
            uint16_t cyc_hz = (uint16_t)(tag_cycle_count  - last_cycle_count);
            Telem_SendInfo();
            Telem_SendStats(cyc_hz, ops_hz);

            last_ops_count   = response_ok_count;
            last_cycle_count = tag_cycle_count;
            last_stats_tick  = now;
        }
    }
}

/* ========================================================================== */
/*                     CLOCK + PERIPHERAL INIT                                 */
/* ========================================================================== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;  /* 64 MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

#if USE_IWDG
/**
 * @brief  Independent Watchdog ~200ms qua thanh ghi (không dùng HAL_IWDG).
 *         Phải gọi IWDG_REFRESH() trong <200ms nếu không MCU sẽ reset.
 */
static void MX_IWDG_Init(void)
{
  DBGMCU->CR |= (1UL << 8);   /* DBG_IWDG_STOP: dừng watchdog khi debugger halt */

  IWDG->KR  = 0x00005555U;    /* mở khoá ghi PR/RLR */
  IWDG->PR  = 0x03U;          /* prescaler /32 */
  IWDG->RLR = 250U;           /* reload → ~200ms */
  IWDG->KR  = 0x0000AAAAU;    /* nạp lại bộ đếm */
  IWDG->KR  = 0x0000CCCCU;    /* khởi động watchdog (tự bật LSI) */
}
#endif

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
