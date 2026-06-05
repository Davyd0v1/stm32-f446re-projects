/* main.c — Variant 2 (Binary counter), UART linefix + HELP + SET cmds, column output */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* Hardware mapping (Nucleo-style) */
#define LATCH_PORT GPIOB
#define LATCH_PIN  GPIO_PIN_0

#define BUZZER_PORT GPIOB
#define BUZZER_PIN  GPIO_PIN_5

#define BTN_START_PORT GPIOC
#define BTN_START_PIN  GPIO_PIN_13

#define BTN_RESET_PORT GPIOB
#define BTN_RESET_PIN  GPIO_PIN_3

#define BTN_INC_PORT GPIOB
#define BTN_INC_PIN  GPIO_PIN_4

#define LED_STEP_PORT GPIOA
#define LED_STEP_PIN  GPIO_PIN_8

#define LED_AUTO_PORT GPIOA
#define LED_AUTO_PIN  GPIO_PIN_9

#define LED_OVF_PORT  GPIOA
#define LED_OVF_PIN   GPIO_PIN_10

#define UART_RX_BUF_LEN 128

/* Peripherals */
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* State */
static volatile uint8_t counter = 0;
static volatile bool running = false;
static volatile bool auto_mode = false;
static volatile bool overflow_flag = false;
static volatile uint32_t speed_ms = 500;
static uint32_t last_tick = 0;

/* UART polling buffer */
static char uart_line[UART_RX_BUF_LEN];
static uint16_t uart_idx = 0;

/* Debounce previous */
static uint8_t prev_start = 1;
static uint8_t prev_reset = 1;
static uint8_t prev_inc = 1;

/* bit mapping mirror (0 = normal, 1 = reverse) */
static uint8_t bit_mirror = 0;

/* prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

/* helpers */
static uint8_t reverse_bits(uint8_t b)
{
  b = (b & 0xF0)>>4 | (b & 0x0F)<<4;
  b = (b & 0xCC)>>2 | (b & 0x33)<<2;
  b = (b & 0xAA)>>1 | (b & 0x55)<<1;
  return b;
}

static void HC595_Write(uint8_t data)
{
  uint8_t out = data;
  if (bit_mirror) out = reverse_bits(data);
  HAL_GPIO_WritePin(LATCH_PORT, LATCH_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &out, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(LATCH_PORT, LATCH_PIN, GPIO_PIN_SET);
}

static void HC595_TestSteps(uint32_t d, uint8_t clear_between)
{
  for (uint8_t i=0;i<8;i++){
    HC595_Write(1u<<i);
    HAL_Delay(d);
    if (clear_between) { HC595_Write(0); HAL_Delay(60); }
  }
  HC595_Write(0);
}

static void update_outputs(void)
{
  HC595_Write(counter);
  HAL_GPIO_WritePin(LED_AUTO_PORT, LED_AUTO_PIN, auto_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_OVF_PORT, LED_OVF_PIN, overflow_flag ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void send_uart(const char *s)
{
  if (!s) return;
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

/* Trim leading spaces */
static char* lskip(char *s) {
  while (*s == ' ' || *s == '\t') ++s;
  return s;
}

/* Process a full ASCII command line (case-insensitive-ish) */
/* Outputs are in column: each field / line ends with \r\n */
static void process_uart_command(const char *cmd)
{
  if (!cmd) return;
  char tmp[UART_RX_BUF_LEN];
  strncpy(tmp, cmd, sizeof(tmp)-1);
  tmp[sizeof(tmp)-1] = '\0';
  /* lowercase in-place */
  for (char *p = tmp; *p; ++p) if (*p >= 'A' && *p <= 'Z') *p = *p - 'A' + 'a';

  /* HELP: list commands column-wise */
  if (strcmp(tmp, "help") == 0) {
    send_uart("Available commands:\r\n");
    send_uart("HELP           - show this help\r\n");
    send_uart("STATUS         - show current state\r\n");
    send_uart("RESET          - set outputs to 0\r\n");
    send_uart("SET 0xNN       - set value (hex)\r\n");
    send_uart("SET N          - set value (dec 0-255)\r\n");
    send_uart("INC            - increment by 1\r\n");
    send_uart("DEC            - decrement by 1\r\n");
    send_uart("COUNT START    - start automatic count\r\n");
    send_uart("COUNT STOP     - stop automatic count\r\n");
    send_uart("SPEED <ms>     - set speed in ms\r\n");
    send_uart("PATTERN        - run LED test pattern\r\n");
    send_uart("\r\n");
    return;
  }

  /* STATUS: print in column */
  if (strcmp(tmp, "status") == 0) {
    char out[128];
    snprintf(out, sizeof(out), "CNT: %u\r\n", (unsigned)counter); send_uart(out);
    snprintf(out, sizeof(out), "AUTO: %u\r\n", (unsigned)auto_mode); send_uart(out);
    snprintf(out, sizeof(out), "RUN: %u\r\n", (unsigned)running); send_uart(out);
    snprintf(out, sizeof(out), "SPD(ms): %lu\r\n", (unsigned long)speed_ms); send_uart(out);
    snprintf(out, sizeof(out), "MAP: %u\r\n", (unsigned)bit_mirror); send_uart(out);
    snprintf(out, sizeof(out), "OVF: %u\r\n", (unsigned)overflow_flag); send_uart(out);
    send_uart("\r\n");
    return;
  }

  /* RESET */
  if (strcmp(tmp, "reset") == 0) {
    counter = 0; overflow_flag = false; update_outputs();
    send_uart("OK\r\n");
    return;
  }

  /* SET handling: "set 0xnn" or "set n" */
  if (strncmp(tmp, "set", 3) == 0) {
    char *arg = lskip(tmp + 3);
    if (*arg == '\0') {
      send_uart("ERR: SET requires argument\r\n");
      return;
    }
    /* hex form allowed: 0xNN */
    uint32_t val = 0;
    if (arg[0] == '0' && (arg[1] == 'x' || arg[1] == 'X')) {
      char *endptr = NULL;
      val = (uint32_t)strtoul(arg, &endptr, 16);
      if (endptr == arg) { send_uart("ERR: invalid hex\r\n"); return; }
    } else {
      /* decimal */
      char *endptr = NULL;
      val = (uint32_t)strtoul(arg, &endptr, 10);
      if (endptr == arg) { send_uart("ERR: invalid decimal\r\n"); return; }
    }
    if (val > 255) { send_uart("ERR: value out of range (0-255)\r\n"); return; }
    counter = (uint8_t)val;
    overflow_flag = false;
    update_outputs();
    char out[64];
    snprintf(out, sizeof(out), "OK: SET %u (0x%02X)\r\n", (unsigned)counter, (unsigned)counter);
    send_uart(out);
    return;
  }

  /* INC */
  if (strcmp(tmp, "inc") == 0) {
    uint8_t prev = counter; counter++; overflow_flag = (prev == 255 && counter == 0);
    HAL_GPIO_TogglePin(LED_STEP_PORT, LED_STEP_PIN);
    update_outputs();
    send_uart("OK: INC\r\n");
    return;
  }

  /* DEC */
  if (strcmp(tmp, "dec") == 0) {
    uint8_t prev = counter; counter--; overflow_flag = (prev == 0 && counter == 255);
    HAL_GPIO_TogglePin(LED_STEP_PORT, LED_STEP_PIN);
    update_outputs();
    send_uart("OK: DEC\r\n");
    return;
  }

  /* COUNT START / STOP */
  if (strstr(tmp, "count start") != NULL) {
    auto_mode = true; running = true; send_uart("OK: COUNT START\r\n"); return;
  }
  if (strstr(tmp, "count stop") != NULL) {
    running = false; auto_mode = false; send_uart("OK: COUNT STOP\r\n"); return;
  }

  /* SPEED */
  if (strncmp(tmp, "speed ", 6) == 0) {
    char *arg = lskip(tmp + 6);
    if (*arg == '\0') { send_uart("ERR: SPEED requires ms value\r\n"); return; }
    long v = strtol(arg, NULL, 10);
    if (v <= 0) { send_uart("ERR: invalid speed\r\n"); return; }
    speed_ms = (uint32_t)v;
    char out[64]; snprintf(out,sizeof(out),"OK: SPEED %lu ms\r\n",(unsigned long)speed_ms); send_uart(out);
    return;
  }

  /* MAP NORMAL / REVERSE */
  if (strncmp(tmp, "map normal", 10) == 0) { bit_mirror = 0; send_uart("OK: MAP NORMAL\r\n"); return; }
  if (strncmp(tmp, "map reverse", 11) == 0) { bit_mirror = 1; send_uart("OK: MAP REVERSE\r\n"); return; }

  /* PATTERN */
  if (strncmp(tmp, "pattern", 7) == 0) {
    send_uart("PATTERN:\r\n");
    HC595_TestSteps(150, 0);
    HC595_TestSteps(100, 1);
    update_outputs();
    return;
  }

  send_uart("ERR: unknown command. Try HELP\r\n");
}

/* ---------- MAIN ---------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* initial */
  counter = 0; running = false; auto_mode = false; overflow_flag = false;
  last_tick = HAL_GetTick();
  HAL_GPIO_WritePin(LATCH_PORT, LATCH_PIN, GPIO_PIN_SET);
  HC595_Write(0);

  /* small boot pattern (help mapping) */
  HC595_TestSteps(100, 0);
  HC595_TestSteps(80, 1);
  update_outputs();

  send_uart("Binary counter ready. Type HELP\r\n");

  /* main loop: UART polling with short timeout */
  for (;;)
  {
    uint32_t now = HAL_GetTick();

    /* UART polling read one byte with small timeout (5 ms) */
    uint8_t ch;
    if (HAL_UART_Receive(&huart2, &ch, 1, 5) == HAL_OK) {

      /* BACKSPACE handling (DEL=127 or BS=8) */
      if (ch == 8 || ch == 127) {
        if (uart_idx > 0) {
          uart_idx--;
          /* erase on terminal: BS SP BS */
          const char bs_seq[] = "\b \b";
          HAL_UART_Transmit(&huart2, (uint8_t*)bs_seq, 3, HAL_MAX_DELAY);
        }
        continue;
      }

      /* End-of-line handling: for CR or LF send CR+LF and process */
      if (ch == '\r' || ch == '\n') {
        const char crlf[] = "\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)crlf, 2, HAL_MAX_DELAY); /* echo newline properly */

        if (uart_idx > 0) {
          uart_line[uart_idx] = 0;
          process_uart_command(uart_line);
          uart_idx = 0;
        } else {
          /* empty line -> just print prompt-like newline */
        }
        continue;
      }

      /* printable char: echo and store */
      HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY);
      /* quick visual */
      HAL_GPIO_WritePin(LED_STEP_PORT, LED_STEP_PIN, GPIO_PIN_SET);
      HAL_Delay(12);
      HAL_GPIO_WritePin(LED_STEP_PORT, LED_STEP_PIN, GPIO_PIN_RESET);

      if (uart_idx < (UART_RX_BUF_LEN - 2)) uart_line[uart_idx++] = (char)ch;
      continue;
    }

    /* AUTO counting */
    if (running && (now - last_tick) >= speed_ms) {
      last_tick = now;
      uint8_t prev = counter;
      counter++;
      if (prev == 255 && counter == 0) {
        overflow_flag = true;
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
        HAL_Delay(20);
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
      } else overflow_flag = false;
      HAL_GPIO_TogglePin(LED_STEP_PORT, LED_STEP_PIN);
      update_outputs();
    }

    /* Buttons polling (debounce) */
    uint8_t s = (uint8_t)HAL_GPIO_ReadPin(BTN_START_PORT, BTN_START_PIN);
    if (prev_start == 1 && s == 0) {
      HAL_Delay(20);
      while (HAL_GPIO_ReadPin(BTN_START_PORT, BTN_START_PIN) == GPIO_PIN_RESET) HAL_Delay(5);
      running = !running;
      if (running) auto_mode = true;
      char t[64]; snprintf(t,sizeof(t),"BTN START: running=%u\r\n",(unsigned)running);
      send_uart(t);
    }
    prev_start = s;

    uint8_t r = (uint8_t)HAL_GPIO_ReadPin(BTN_RESET_PORT, BTN_RESET_PIN);
    if (prev_reset == 1 && r == 0) {
      HAL_Delay(20);
      while(HAL_GPIO_ReadPin(BTN_RESET_PORT, BTN_RESET_PIN) == GPIO_PIN_RESET) HAL_Delay(5);
      counter = 0; overflow_flag = false; update_outputs();
      send_uart("BTN RESET\r\n");
    }
    prev_reset = r;

    uint8_t i = (uint8_t)HAL_GPIO_ReadPin(BTN_INC_PORT, BTN_INC_PIN);
    if (prev_inc == 1 && i == 0) {
      HAL_Delay(20);
      while(HAL_GPIO_ReadPin(BTN_INC_PORT, BTN_INC_PIN) == GPIO_PIN_RESET) HAL_Delay(5);
      uint8_t prevv = counter; counter++; overflow_flag = (prevv == 255 && counter == 0);
      HAL_GPIO_TogglePin(LED_STEP_PORT, LED_STEP_PIN);
      update_outputs();
      send_uart("BTN INC\r\n");
    }
    prev_inc = i;

    /* small sleep to reduce CPU usage */
    HAL_Delay(2);
  }
}

/* ----------------- MX / HAL functions ----------------- */

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
  __HAL_RCC_SPI1_CLK_ENABLE();
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

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
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* PB0 latch, PB5 buzzer */
  HAL_GPIO_WritePin(GPIOB, LATCH_PIN|BUZZER_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LATCH_PIN|BUZZER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PA8 PA9 PA10 LEDs */
  HAL_GPIO_WritePin(GPIOA, LED_STEP_PIN|LED_AUTO_PIN|LED_OVF_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_STEP_PIN|LED_AUTO_PIN|LED_OVF_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Buttons: PC13, PB3, PB4 inputs pull-up */
  GPIO_InitStruct.Pin = BTN_START_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_START_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BTN_RESET_PIN|BTN_INC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI1 AF pins: PA5 SCK, PA7 MOSI */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 AF pins: PA2 TX, PA3 RX */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LED_OVF_PORT, LED_OVF_PIN);
    HAL_Delay(200);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  char buf[128];
  snprintf(buf, sizeof(buf), "Assert failed: %s:%lu\r\n", file ? (char*)file : "?", (unsigned long)line);
  send_uart(buf);
}
#endif
