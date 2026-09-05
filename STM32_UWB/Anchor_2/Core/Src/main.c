/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c (Anchor)
  * @brief          : SS-TWR Ranging Anchor Node
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "dw1000_hw.h"
#include "anchor_ranging.h"
#include <stdio.h>

/* USER CODE BEGIN PV */
volatile DW1000_RangingResult ranging_result;
volatile uint32_t device_id = 0;

/* Phase 2: readback PHY (0 = khớp) — Live Expressions. */
volatile uint32_t dw1000_config_mismatch = 0;

SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart1;

/* Phase 2: Independent Watchdog qua thanh ghi (không phụ thuộc module HAL_IWDG).
 * LSI ~40kHz / 32 = 1250Hz, reload 250 → ~200ms. */
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

/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  // HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_SPI1_Init();
  // MX_USART1_UART_Init();

  /* Phase 2: đồng hồ µs (DWT). */
  MCU_TimerInit();

  // printf("\r\n================================\r\n");
  // printf("  UWB ANCHOR (ADDR = 0x%04X)\r\n", ANCHOR_ADDR);
  // printf("  IRQ State-Machine @ 50Hz\r\n");
  // printf("================================\r\n");

  /* ---- Boot blink: 3 quick flashes to confirm startup ---- */
  for (int i = 0; i < 3; i++)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); HAL_Delay(120);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   HAL_Delay(120);
  }

  /* ---- DW1000 Init ---------------------------------------- */
  if (Anchor_Init() != 0)
  {
    // printf("[ERROR] DW1000 Init FAILED! Halting.\r\n");
    device_id = 0xDEAD;
    /* Fast blink = fatal error */
    while (1) { HAL_GPIO_TogglePin(LED_PORT, LED_PIN); HAL_Delay(200); }
  }

  device_id = DW1000_ReadDeviceID();
  // printf("[OK] DW1000 ID = 0x%08lX\r\n", device_id);

  /* ---- Phase 2: readback verify PHY config --------------- */
  dw1000_config_mismatch = DW1000_VerifyConfig();

  /* ---- Arm EXTI on PA3 (DW1000 IRQ) ---------------------- *
   * Order is critical:                                         *
   *   Anchor_Init() calls DW1000_Configure() which sets        *
   *   SYS_MASK, then DW1000_ClearAllStatus() lowers IRQ pin.  *
   *   Only NOW is it safe to arm the EXTI.                     */
  DW1000_EnableIRQ();

  /* ---- Start listening for POLL frames -------------------- */
  Anchor_StartListening();
  // printf("[OK] Listening... (IRQ-driven state machine running)\r\n");

  /* ---- Phase 2: bật watchdog sau khi init xong ----------- */
#if USE_IWDG
  MX_IWDG_Init();
#endif

  /* ---- Main loop: purely non-blocking --------------------- */
  while (1)
  {
    /* Anchor_Task() returns immediately if dw1000_irq_flag == 0.
     * When DW1000 fires an IRQ, the flag is set and Task() handles it. */
    Anchor_Task();

    /* Phase 2 (F): IRQ mức — nếu còn HIGH mà lỡ cạnh, ép xử lý vòng sau. */
    if (DW1000_IrqLineActive() && !dw1000_irq_flag)
        dw1000_irq_flag = 1;

#if USE_IWDG
    IWDG_REFRESH();
#endif

    /* Mirror result to the watchable volatile for Live Expressions */
    const DW1000_RangingResult *r = Anchor_GetResult();
    ranging_result.ranging_count  = r->ranging_count;
    ranging_result.timeout_count  = r->timeout_count;
    ranging_result.last_status    = r->last_status;
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;  /* 64MHz */
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
 */
static void MX_IWDG_Init(void)
{
  DBGMCU->CR |= (1UL << 8);   /* DBG_IWDG_STOP: dừng watchdog khi debugger halt */

  IWDG->KR  = 0x00005555U;    /* mở khoá ghi PR/RLR */
  IWDG->PR  = 0x03U;          /* prescaler /32 */
  IWDG->RLR = 250U;           /* reload → ~200ms */
  IWDG->KR  = 0x0000AAAAU;    /* nạp lại bộ đếm */
  IWDG->KR  = 0x0000CCCCU;    /* khởi động watchdog */
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
