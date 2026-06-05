/* main.c
   Nucleo-F446RE: UART -> LCD1602 via PCF8574 (I2C PB8/PB9)
   - I2C1 PB8 = SCL, PB9 = SDA (AF4)
   - USART2 PA2 = TX, PA3 = RX (AF7) @115200
   - PCF8574 mapping: P4..P7 = D4..D7, P0 = RS, P2 = EN, P3 = BL
   - LCD_ADDR = 0x27<<1 (change to 0x3F<<1 if required)
*/

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdint.h>

/* ----------------- HAL handles ----------------- */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* ----------------- Config ----------------- */
#define LCD_ADDR    (0x27 << 1)
#define RX_BUF_SZ   128
#define DEBOUNCE_MS 150

/* pins for simple LEDs / piezo if needed (not required for LCD) */
#define LED_PORT GPIOA
#define LED_G    GPIO_PIN_5

/* UART rx buffer (polled) */
static char rx_buf[RX_BUF_SZ];
static uint16_t rx_idx = 0;
static volatile uint8_t rx_line_ready = 0;

/* app state */
static volatile uint8_t mode_counter = 0;
static volatile uint32_t counter_val = 0;
static volatile uint32_t beep_end_tick = 0;

/* debounce */
static uint32_t last_btn_pc13 = 0;
static uint32_t last_btn_pa0  = 0;
static uint32_t last_btn_pa1  = 0;

/* forward prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

/* ---------- UART helper ---------- */
static void uart_printf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)strlen(buf), HAL_MAX_DELAY);
}

/* ---------- LCD (PCF8574) low-level (mapping variant common) ---------- */
/* mapping: P4..P7 = D4..D7, P0 = RS, P1 = RW (unused), P2 = EN, P3 = BL */
#define M_RS  0x01
#define M_RW  0x02
#define M_EN  0x04
#define M_BL  0x08

static HAL_StatusTypeDef pcf_write_byte(uint8_t b)
{
  uint8_t tx = b | M_BL; /* keep backlight on unless explicitly off */
  return HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &tx, 1, 200);
}

static void lcd_pulse_enable(uint8_t data)
{
  /* E = 1 */
  (void)pcf_write_byte(data | M_EN);
  HAL_Delay(1);
  /* E = 0 */
  (void)pcf_write_byte(data & ~M_EN);
  HAL_Delay(1);
}

/* nibble passed in high 4 bits (0xF0) */
static void lcd_send4(uint8_t nibble_high, uint8_t rs)
{
  uint8_t out = (nibble_high & 0xF0) | (rs ? M_RS : 0x00);
  (void)pcf_write_byte(out);
  lcd_pulse_enable(out);
}

static void lcd_send_byte(uint8_t byte, uint8_t rs)
{
  uint8_t high = byte & 0xF0;
  uint8_t low  = (uint8_t)((byte << 4) & 0xF0);
  lcd_send4(high, rs);
  lcd_send4(low,  rs);
  HAL_Delay(1);
}

static void LCD_Command(uint8_t cmd) { lcd_send_byte(cmd, 0); HAL_Delay(2); }
static void LCD_Char(uint8_t c)      { lcd_send_byte(c, 1); }

/* Init sequence for HD44780 via PCF8574 */
static void LCD_Init(void)
{
  HAL_Delay(50);
  lcd_send4(0x30, 0); HAL_Delay(5);
  lcd_send4(0x30, 0); HAL_Delay(5);
  lcd_send4(0x30, 0); HAL_Delay(2);
  lcd_send4(0x20, 0); HAL_Delay(2);

  LCD_Command(0x28); /* 4-bit, 2 lines */
  LCD_Command(0x08); /* display off */
  LCD_Command(0x01); HAL_Delay(2); /* clear */
  LCD_Command(0x06); /* entry mode */
  LCD_Command(0x0C); /* display on, cursor off */
}

static void LCD_SetCursor(uint8_t row, uint8_t col)
{
  uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
  LCD_Command(addr);
}

static void LCD_PrintPadded(const char *s, uint8_t width)
{
  uint8_t i = 0;
  while (*s && i < width) { LCD_Char((uint8_t)*s++); i++; }
  for (; i < width; ++i) LCD_Char(' ');
}

/* ---------- I2C helpers (diagnostics) ---------- */
static void I2C_Scan_Aggressive(void)
{
    uart_printf("I2C scan (aggressive) start...\r\n");
    for (uint8_t addr = 1; addr < 127; ++addr) {
        int found = 0;
        for (int attempt = 0; attempt < 3; ++attempt) {
            HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, 50);
            if (st == HAL_OK) { found = 1; break; }
            HAL_Delay(5);
        }
        if (found) uart_printf("Found 0x%02X\r\n", addr);
    }
    const uint8_t test_addrs[] = {0x27, 0x3F};
    for (int i=0;i<2;i++) {
        uint8_t a = test_addrs[i];
        uart_printf("Explicit check 0x%02X: ", a);
        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 5, 300);
        if (st == HAL_OK) uart_printf("ACK\r\n"); else uart_printf("NO ACK\r\n");
    }
    uart_printf("I2C scan done\r\n");
}

static void I2C_TestAddress(uint8_t addr7)
{
    uint8_t dev = (uint8_t)(addr7 << 1);
    uint8_t w = 0x00; /* set all outputs low */
    uint8_t r = 0xFF;
    uart_printf("I2C_TEST: addr 0x%02X (8bit 0x%02X) -> write 0x%02X ... ", addr7, dev, w);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, dev, &w, 1, 200);
    if (st == HAL_OK) {
        uart_printf("TX_OK. Attempting RX... ");
        HAL_Delay(5);
        st = HAL_I2C_Master_Receive(&hi2c1, dev, &r, 1, 200);
        if (st == HAL_OK) {
            uart_printf("RX_OK, read=0x%02X\r\n", r);
        } else {
            uart_printf("RX_FAIL (status=%d)\r\n", (int)st);
        }
    } else {
        uart_printf("TX_FAIL (status=%d)\r\n", (int)st);
    }
}

/* ---------- trim helper ---------- */
static void trim_inplace(char *s)
{
  if (!s) return;
  char *p = s;
  while (*p && isspace((unsigned char)*p)) p++;
  if (p != s) memmove(s, p, strlen(p)+1);
  int len = (int)strlen(s);
  while (len > 0 && isspace((unsigned char)s[len-1])) { s[len-1] = 0; len--; }
}

/* ---------- process commands ---------- */
static void process_command(const char *cmd_in)
{
  if (!cmd_in) return;
  char buf[RX_BUF_SZ];
  strncpy(buf, cmd_in, RX_BUF_SZ-1);
  buf[RX_BUF_SZ-1] = 0;
  trim_inplace(buf);

  char up[RX_BUF_SZ];
  strncpy(up, buf, RX_BUF_SZ-1);
  up[RX_BUF_SZ-1] = 0;
  for (char *p = up; *p; ++p) *p = (char)toupper((unsigned char)*p);

  if (strcmp(up, "HELP") == 0) {
    uart_printf("Commands: HELP, TEXT <text>, CLEAR, SCAN, TESTLCD, STATUS, MODE TEXT, MODE COUNTER, BEEP <ms>\r\n");
    return;
  }

  if (strncmp(up, "TEXT ", 5) == 0) {
    LCD_Command(0x01); HAL_Delay(2);
    LCD_SetCursor(0,0);
    LCD_PrintPadded(buf + 5, 16);
    uart_printf("OK\r\n");
    return;
  }

  if (strcmp(up, "CLEAR") == 0) {
    LCD_Command(0x01); HAL_Delay(2);
    uart_printf("CLEARED\r\n");
    return;
  }

  if (strcmp(up, "SCAN") == 0) {
      I2C_Scan_Aggressive();
      return;
  }
  if (strcmp(up, "TESTLCD") == 0) {
      I2C_TestAddress(0x27);
      I2C_TestAddress(0x3F);
      return;
  }
  if (strcmp(up, "STATUS") == 0) {
    uart_printf("Mode: %s, Counter: %lu\r\n", mode_counter ? "COUNTER" : "TEXT", (unsigned long)counter_val);
    return;
  }

  if (strncmp(up, "MODE ", 5) == 0) {
    if (strstr(up, "COUNTER")) {
      mode_counter = 1;
      LCD_Command(0x01); HAL_Delay(2);
      LCD_SetCursor(0,0); LCD_PrintPadded("Mode: COUNTER", 16);
      LCD_SetCursor(1,0);
      char t[17]; snprintf(t, sizeof(t), "Cnt:%lu", (unsigned long)counter_val);
      LCD_PrintPadded(t, 16);
      uart_printf("Mode=COUNTER\r\n");
    } else if (strstr(up, "TEXT")) {
      mode_counter = 0;
      LCD_Command(0x01); HAL_Delay(2);
      LCD_SetCursor(0,0); LCD_PrintPadded("Mode: TEXT", 16);
      uart_printf("Mode=TEXT\r\n");
    } else {
      uart_printf("MODE: use TEXT or COUNTER\r\n");
    }
    /* beep feedback */
    beep_end_tick = HAL_GetTick() + 80;
    return;
  }

  if (strncmp(up, "BEEP ", 5) == 0) {
    int ms = atoi(buf + 5);
    if (ms <= 0) ms = 100;
    beep_end_tick = HAL_GetTick() + (uint32_t)ms;
    uart_printf("BEEP %d ms\r\n", ms);
    return;
  }

  uart_printf("Unknown: %s\r\n", buf);
}

/* ---------- outputs maintenance ---------- */
static void update_outputs(void)
{
  if (beep_end_tick != 0 && (int32_t)(HAL_GetTick() - beep_end_tick) >= 0) {
    beep_end_tick = 0;
  }
}

/* ---------------------- main ---------------------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  uart_printf("Before I2C init, I2C state: %d\r\n", HAL_I2C_GetState(&hi2c1));
  MX_I2C1_Init();
  uart_printf("I2C1 state: %d\r\n", HAL_I2C_GetState(&hi2c1));

  HAL_Delay(50);
  LCD_Init();
  LCD_SetCursor(0,0);
  LCD_PrintPadded("I2C LCD ready", 16);

  uart_printf("UART->LCD ready. Type text and press Enter (CR or CR+LF).\r\n");

  rx_idx = 0;
  rx_line_ready = 0;

  uint32_t last_poll = HAL_GetTick();
  uint32_t last_counter_tick = HAL_GetTick();
  uint32_t blink_tick = HAL_GetTick();

  while (1) {
    uint32_t now = HAL_GetTick();

    /* Poll UART (short timeout) */
    uint8_t c;
    if (HAL_UART_Receive(&huart2, &c, 1, 10) == HAL_OK) {
      /* echo */
      HAL_UART_Transmit(&huart2, &c, 1, 20);
      if (c == '\r' || c == '\n') {
        if (rx_idx > 0) {
          rx_buf[rx_idx] = 0;
          rx_line_ready = 1;
        }
        rx_idx = 0;
      } else {
        if (rx_idx < RX_BUF_SZ - 1) rx_buf[rx_idx++] = (char)c;
        else rx_idx = 0;
      }
    }

    /* poll buttons every 20 ms */
    if ((uint32_t)(now - last_poll) >= 20) {
      last_poll = now;

      /* PC13 user button (active low) */
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
        if ((now - last_btn_pc13) > DEBOUNCE_MS) {
          last_btn_pc13 = now;
          mode_counter = !mode_counter;
          uart_printf("BTN_USER toggled mode -> %s\r\n", mode_counter ? "COUNTER" : "TEXT");
          if (mode_counter) {
            LCD_Command(0x01); HAL_Delay(2);
            LCD_SetCursor(0,0); LCD_PrintPadded("Mode: COUNTER", 16);
            LCD_SetCursor(1,0);
            char t[17]; snprintf(t, sizeof(t), "Cnt:%lu", (unsigned long)counter_val);
            LCD_PrintPadded(t, 16);
          } else {
            LCD_Command(0x01); HAL_Delay(2);
            LCD_SetCursor(0,0); LCD_PrintPadded("Mode: TEXT", 16);
          }
        }
      }
    }

    /* process UART line if ready */
    if (rx_line_ready) {
      rx_line_ready = 0;
      process_command(rx_buf);
      const char nl[] = "\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)nl, 2, 50);
      rx_idx = 0;
      rx_buf[0] = 0;
    }

    /* counter periodic update */
    if (mode_counter) {
      if ((uint32_t)(now - last_counter_tick) >= 500) {
        last_counter_tick = now;
        char t[17];
        snprintf(t, sizeof(t), "Cnt:%lu", (unsigned long)counter_val++);
        LCD_SetCursor(1,0);
        LCD_PrintPadded(t, 16);
      }
    }

    /* alive blink */
    if ((uint32_t)(now - blink_tick) >= 500) {
      blink_tick = now;
      HAL_GPIO_TogglePin(LED_PORT, LED_G);
    }

    update_outputs();
    HAL_Delay(5);
  }
}

/* ----------------- Hardware init functions ----------------- */

/* Simple SystemClock HSI -> PLL (avoid PWR calls) */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* LED PA5 for alive blink */
  GPIO_InitStruct.Pin = LED_G;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_PORT, LED_G, GPIO_PIN_RESET);

  /* User button PC13 input pull-up */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void MX_I2C1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* PB8 = SCL, PB9 = SDA: AF4 open-drain */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP; /* external pull-ups recommended */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

void MX_USART2_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  /* PA2 TX / PA3 RX (AF7) */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/* basic error handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LED_PORT, LED_G);
    HAL_Delay(300);
  }
}
