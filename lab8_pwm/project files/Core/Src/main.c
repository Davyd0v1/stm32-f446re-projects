/* main.c — PWM brightness regulator (Variant 2)
   - UART работает без прерываний (опрос в главном цикле)
   - Остальная логика без изменений
*/

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdlib.h>

/* --- Peripheral handles --- */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* --- Config --- */
#ifndef PWM_RES_BITS
#define PWM_RES_BITS 10
#endif
#if (PWM_RES_BITS != 8) && (PWM_RES_BITS != 10)
#error "PWM_RES_BITS must be 8 or 10"
#endif

#define TIMER_CLK_HZ 50000000UL
#define PWM_ARR ((1U << PWM_RES_BITS) - 1U)
#define RX_BUF_LEN 128
#define DEBOUNCE_MS 30
#define BEEP_MS 80

/* --- State --- */
static char rx_line[RX_BUF_LEN];
static uint16_t rx_idx = 0;        // уже не volatile, т.к. не из прерывания

volatile uint8_t pwm_enabled = 0;
volatile uint8_t duty_percent = 50; /* 0..100 */
volatile uint32_t pwm_freq = 500;   /* Hz */
volatile uint32_t pwm_prescaler = 0;
volatile uint32_t pwm_arr = PWM_ARR;
volatile uint32_t pwm_ccr = 0;

/* Fade structure */
typedef struct {
  uint8_t active;
  int start_pct;
  int end_pct;
  uint32_t duration_ms;
  uint32_t elapsed_ms;
} fade_t;
static fade_t fade = {0};

/* Debounce */
typedef struct { uint8_t stable; uint8_t prev; uint8_t cnt; } db_t;
static db_t db1 = {1,1,0}, db2 = {1,1,0}, db3 = {1,1,0};

/* Blink & beep */
static uint32_t blink_counter_ms = 0;
static uint8_t beep_active = 0;
static uint32_t beep_time_ms = 0;

/* --- Prototypes (CubeMX) --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* --- Helpers --- */
static void uart_printf(const char *fmt, ...)
{
  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

/* compute prescaler for target freq (ARR = pwm_arr) */
static int compute_prescaler_for_freq(uint32_t freq, uint32_t *out_presc)
{
  if (freq == 0 || pwm_arr == 0) return 0;
  uint64_t denom = (uint64_t)(pwm_arr + 1) * (uint64_t)freq;
  if (denom == 0) return 0;
  uint64_t presc_real = (TIMER_CLK_HZ + denom - 1) / denom; /* ceil */
  if (presc_real == 0) presc_real = 1;
  uint64_t presc = presc_real - 1;
  if (presc > 0xFFFF) return 0;
  *out_presc = (uint32_t)presc;
  return 1;
}

/* Apply PWM frequency by re-init TIM3 (ARR = pwm_arr) */
static void apply_pwm_frequency(uint32_t freq)
{
  if (freq < 100) freq = 100;
  if (freq > 1000) freq = 1000;
  uint32_t presc;
  if (!compute_prescaler_for_freq(freq, &presc)) {
    uart_printf("ERR: cannot set freq %lu (prescaler overflow)\r\n", (unsigned long)freq);
    return;
  }

  /* stop PWM */
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

  /* reconfigure TIM3 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (uint16_t)presc;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = pwm_arr;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    uart_printf("ERR: HAL_TIM_PWM_Init failed\r\n");
    while (1);
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  uint32_t ccr = (uint32_t)duty_percent * (pwm_arr + 1) / 100;
  sConfigOC.Pulse = ccr;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    uart_printf("ERR: HAL_TIM_PWM_ConfigChannel failed\r\n");
    while (1);
  }

  if (pwm_enabled) HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  pwm_prescaler = presc;
  pwm_freq = freq;
  pwm_ccr = ccr;

  uart_printf("OK: FREQ %lu Hz, RES %u-bit\r\n", (unsigned long)freq, PWM_RES_BITS);
}

/* Set duty percent and update compare */
static void set_duty_percent(uint8_t pct)
{
  if (pct > 100) pct = 100;
  duty_percent = pct;
  uint32_t ccr = (uint32_t)duty_percent * (pwm_arr + 1) / 100;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
  pwm_ccr = ccr;

  /* LED3 PA7 on at 0 or 100 */
  if (duty_percent == 0 || duty_percent == 100)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  uart_printf("OK: DUTY %u%%\r\n", duty_percent);
}

/* Start/Stop PWM */
static void pwm_start_stop(uint8_t start)
{
  pwm_enabled = start ? 1 : 0;
  if (pwm_enabled) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    uart_printf("PWM STARTED\r\n");
  } else {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    uart_printf("PWM STOPPED\r\n");
  }
}

/* Start short beep on piezo */
static void start_beep(uint32_t ms)
{
  uint32_t pulse = (htim1.Init.Period + 1) / 2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  beep_active = 1;
  beep_time_ms = ms;
}

/* --- TIM2 1ms callback: debounce/buttons/fade/blink/beep --- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    /* Button1 PC13: PullUp, pressed when LOW (RESET) */
    uint8_t s1 = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    if (s1 != db1.prev) { db1.cnt = 0; db1.prev = s1; }
    else {
      if (db1.cnt < DEBOUNCE_MS) db1.cnt++;
      if (db1.cnt == DEBOUNCE_MS && db1.stable != s1) {
        db1.stable = s1;
        if (s1 == 0) { pwm_start_stop(!pwm_enabled); start_beep(BEEP_MS); }
      }
    }

    /* Button2 PA0: configured PullUp -> pressed when LOW */
    uint8_t s2 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0 : 1;
    if (s2 != db2.prev) { db2.cnt = 0; db2.prev = s2; }
    else {
      if (db2.cnt < DEBOUNCE_MS) db2.cnt++;
      if (db2.cnt == DEBOUNCE_MS && db2.stable != s2) {
        db2.stable = s2;
        if (s2 == 0) { /* pressed */
          int newd = duty_percent + 10;
          if (newd > 100) newd = 100;
          set_duty_percent((uint8_t)newd);
          start_beep(BEEP_MS);
        }
      }
    }

    /* Button3 PA1: PullUp -> pressed when LOW */
    uint8_t s3 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) ? 0 : 1;
    if (s3 != db3.prev) { db3.cnt = 0; db3.prev = s3; }
    else {
      if (db3.cnt < DEBOUNCE_MS) db3.cnt++;
      if (db3.cnt == DEBOUNCE_MS && db3.stable != s3) {
        db3.stable = s3;
        if (s3 == 0) {
          int newd = duty_percent - 10;
          if (newd < 0) newd = 0;
          set_duty_percent((uint8_t)newd);
          start_beep(BEEP_MS);
        }
      }
    }

    /* Fade handling */
    if (fade.active) {
      fade.elapsed_ms++;
      if (fade.elapsed_ms >= fade.duration_ms) {
        fade.active = 0;
        set_duty_percent((uint8_t)fade.end_pct);
        uart_printf("FADE done -> %d%%\r\n", fade.end_pct);
      } else {
        float t = (float)fade.elapsed_ms / (float)fade.duration_ms;
        int cur = (int)(fade.start_pct + t * (fade.end_pct - fade.start_pct));
        if (cur < 0) cur = 0;
        if (cur > 100) cur = 100;
        set_duty_percent((uint8_t)cur);
      }
    }

    /* Beep timeout */
    if (beep_active) {
      if (beep_time_ms > 0) {
        beep_time_ms--;
        if (beep_time_ms == 0) {
          HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
          beep_active = 0;
        }
      }
    }

    /* LED2 (PA5) blink 1Hz when PWM active */
    if (pwm_enabled) {
      blink_counter_ms++;
      if (blink_counter_ms >= 500) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        blink_counter_ms = 0;
      }
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      blink_counter_ms = 0;
    }
  }
}

/* --- UART обработка без прерываний (вызывается из main) --- */
/* --- UART обработка без прерываний (RXNE polling, ПОЛНАЯ версия) --- */
static void uart_process(void)
{
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
  {
    uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
    char c = (char)ch;

    /* Эхо символа */
    HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY);

    if (c == '\r' || c == '\n')
    {
      if (rx_idx > 0)
      {
        rx_line[rx_idx] = '\0';

        for (uint16_t i = 0; i < rx_idx; ++i)
          rx_line[i] = (char)toupper((unsigned char)rx_line[i]);

        /* ================= КОМАНДЫ ================= */

        if (strncmp(rx_line, "DUTY", 4) == 0) {
          int val;
          if (sscanf(rx_line, "DUTY %d", &val) == 1) {
            if (val < 0) val = 0;
            if (val > 100) val = 100;
            set_duty_percent((uint8_t)val);
          } else uart_printf("ERR: DUTY <0-100>\r\n");
        }

        else if (strncmp(rx_line, "FREQ", 4) == 0) {
          int val;
          if (sscanf(rx_line, "FREQ %d", &val) == 1) {
            if (val < 100) val = 100;
            if (val > 1000) val = 1000;
            apply_pwm_frequency((uint32_t)val);
          } else uart_printf("ERR: FREQ <100-1000>\r\n");
        }

        else if (strncmp(rx_line, "FADE", 4) == 0) {
          int a,b,s;
          if (sscanf(rx_line, "FADE %d-%d %d", &a, &b, &s) == 3 && s >= 0) {
            if (a < 0) a = 0; if (a > 100) a = 100;
            if (b < 0) b = 0; if (b > 100) b = 100;
            fade.active = 1;
            fade.start_pct = a;
            fade.end_pct = b;
            fade.duration_ms = (uint32_t)s * 1000U;
            fade.elapsed_ms = 0;
            set_duty_percent((uint8_t)a);
            uart_printf("OK: FADE %d->%d over %d s\r\n", a, b, s);
          } else uart_printf("ERR: FADE a-b seconds\r\n");
        }

        else if (strcmp(rx_line, "START") == 0) {
          pwm_start_stop(1);
        }

        else if (strcmp(rx_line, "STOP") == 0) {
          pwm_start_stop(0);
        }

        else if (strcmp(rx_line, "STATUS") == 0) {
          uart_printf("STATUS: PWM=%s, DUTY=%u%%, FREQ=%luHz, RES=%u-bit\r\n",
                      pwm_enabled ? "ON" : "OFF",
                      duty_percent,
                      (unsigned long)pwm_freq,
                      PWM_RES_BITS);
        }

        else if (strcmp(rx_line, "RESET") == 0) {
          duty_percent = 50;
          fade.active = 0;
          apply_pwm_frequency(500);
          set_duty_percent(50);
          pwm_start_stop(0);
          uart_printf("RESET to defaults\r\n");
        }

        else if (strcmp(rx_line, "PARAM") == 0) {
          uart_printf("PARAM: TIMER_CLK=%lu, PRESC=%lu, ARR=%lu, CCR=%lu\r\n",
                      (unsigned long)TIMER_CLK_HZ,
                      (unsigned long)pwm_prescaler,
                      (unsigned long)pwm_arr,
                      (unsigned long)pwm_ccr);
        }

        else if (strcmp(rx_line, "HELP") == 0) {
          uart_printf("Available commands:\r\n");
          uart_printf("DUTY <0-100>\r\n");
          uart_printf("FREQ <100-1000>\r\n");
          uart_printf("FADE a-b seconds\r\n");
          uart_printf("START\r\n");
          uart_printf("STOP\r\n");
          uart_printf("STATUS\r\n");
          uart_printf("RESET\r\n");
          uart_printf("PARAM\r\n");
          uart_printf("HELP\r\n");
        }

        else {
          uart_printf("ERR: unknown cmd\r\n");
        }
      }

      rx_idx = 0;
      uart_printf("> ");
    }
    else
    {
      if (rx_idx < RX_BUF_LEN - 1)
        rx_line[rx_idx++] = c;
      else
        rx_idx = 0;
    }
  }
}

/* --- main --- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* runtime init */
  pwm_arr = htim3.Init.Period;
  pwm_prescaler = htim3.Init.Prescaler;
  pwm_freq = 500;

  set_duty_percent(duty_percent);
  pwm_start_stop(0);

  /* start TIM2 base (1ms ticks) */
  HAL_TIM_Base_Start_IT(&htim2);

  /* init debouncers from current pin levels to avoid false triggers */
  db1.prev = db1.stable = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) ? 1 : 0;
  db2.prev = db2.stable = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;
  db3.prev = db3.stable = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) ? 1 : 0;

  uart_printf("Boot: PWM controller ready. RES=%u-bit\r\n", PWM_RES_BITS);
  uart_printf("HELP - all comands\r\n");

  while (1) {
    // Обработка UART (без прерываний)
    uart_process();

    // Можно добавить небольшой delay для снижения нагрузки, но тогда команды
    // будут обрабатываться с задержкой. Лучше не ставить, или поставить маленький.
    // HAL_Delay(1); // опционально
  }
}

/* -----------------------
   Ниже без изменений, кроме удалённого HAL_UART_RxCpltCallback
   и переноса инициализации пинов USART2 в MX_GPIO_Init.
   ----------------------- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* TIM1 init (piezo) */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim1);
}

/* TIM2 init (1 ms base) */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  /* enable TIM2 IRQ (CubeMX usually does in stm32xx_it.c too) */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* TIM3 init (PWM) */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_ARR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (pwm_arr + 1) / 2; /* initial 50% */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim3);

  pwm_arr = htim3.Init.Period;
  pwm_prescaler = htim3.Init.Prescaler;
}

/* USART2 init (без прерываний, только аппаратная инициализация) */
static void MX_USART2_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }

  /* Прерывания не включаем */
}

/* GPIO init */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* initial LED outputs off */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /* B1 (PC13) user button EXTI falling, PullUp */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* PA0 button 2: set Pull-Up (pressed = LOW) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA1 button 3: PullUp */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA5 PA7 outputs for LED blink and limit */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA2 (USART2_TX), PA3 (USART2_RX) as alternate function AF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* Error handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

/* End of file */
