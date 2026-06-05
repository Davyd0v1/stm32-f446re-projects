/* main.c — исправленная версия: ADC IRQ + UART polling + команды + диагностika RAW */
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

/* pins */
#define LED_PORT    GPIOA
#define LED1_PIN    GPIO_PIN_5
#define LED2_PIN    GPIO_PIN_6
#define LED3_PIN    GPIO_PIN_7
#define PIEZO_PIN   GPIO_PIN_8

#define BTN1_PORT   GPIOC
#define BTN1_PIN    GPIO_PIN_13
#define BTN2_PORT   GPIOA
#define BTN2_PIN    GPIO_PIN_4
#define BTN3_PORT   GPIOB
#define BTN3_PIN    GPIO_PIN_0

/* state */
volatile uint32_t adc_value = 0;
volatile uint32_t center = 2048;
volatile int deviation = 0; /* percent */
int sensitivity = 5;
int threshold_percent = 10;
bool measuring_enabled = true;
bool led_mode_relative = true;
bool beep_enable = true;

/* runtime helpers */
volatile bool new_adc_value = false;
int blink = 0;
uint32_t last_blink_ms = 0;
uint32_t last_piezo_ms = 0;
const uint32_t blink_interval_ms = 300;
const uint32_t piezo_toggle_ms = 2;

/* UART buffer (polling) */
#define RX_BUF_SIZE 128
char rx_buf[RX_BUF_SIZE];
uint16_t rx_idx = 0;

/* prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

/* functions */
void update_leds(void);
void Beep_Handler(void);
void Check_Buttons(void);
void Process_Command(char *cmd);
void uart_prompt(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  /* Start ADC in interrupt (continuous) */
  HAL_ADC_Start_IT(&hadc1);

  /* initial prompt */
  uart_prompt();

  uint32_t last_status_ms = HAL_GetTick();

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* UART polling: check RXNE flag */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
      uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF); /* read clears RXNE */
      char c = (char)ch;
      /* echo */
      HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

      if (c == '\r' || c == '\n') {
        if (rx_idx > 0) {
          rx_buf[rx_idx] = '\0';
          /* uppercase for parsing */
          for (uint16_t i=0;i<rx_idx;i++) rx_buf[i] = (char)toupper((unsigned char)rx_buf[i]);
          Process_Command(rx_buf);
        }
        rx_idx = 0;
        uart_prompt();
      } else {
        if (rx_idx < RX_BUF_SIZE-1) rx_buf[rx_idx++] = c;
        else rx_idx = 0;
      }
    }

    /* If ADC IRQ updated value */
    if (new_adc_value) {
      new_adc_value = false;
      update_leds();
    }

    Beep_Handler();
    Check_Buttons();

    /* periodic status every 1s */
    if (now - last_status_ms >= 1000) {
      char msg[120];
      snprintf(msg, sizeof(msg),
               "[LEVEL] Deviation: %+d%% %s, Center: %lu (%.1f%%), Sensitivity: %d%%\r\n",
               deviation, (abs(deviation) > threshold_percent ? "ALARM!" : ""),
               (unsigned long)center, ((double)center*100.0/4095.0), sensitivity);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      last_status_ms = now;
    }

    HAL_Delay(1);
  }
}

/* ADC callback */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc && hadc->Instance == ADC1) {
    adc_value = HAL_ADC_GetValue(&hadc1);
    deviation = ((int)adc_value - (int)center) * 100 / 4095;
    new_adc_value = true;
  }
}

void uart_prompt(void)
{
  const char *p = "> ";
  HAL_UART_Transmit(&huart2, (uint8_t*)p, strlen(p), HAL_MAX_DELAY);
}

/* Commands including RAW diagnostic */
void Process_Command(char *cmd)
{
  char resp[160];
  if (strncmp(cmd, "ANGLE", 5) == 0) {
    snprintf(resp,sizeof(resp), "[OK] Angle: %+d%%\r\n", deviation);
    HAL_UART_Transmit(&huart2,(uint8_t*)resp,strlen(resp),HAL_MAX_DELAY);
  }
  else if (strncmp(cmd, "RAW",3)==0) {
    snprintf(resp,sizeof(resp), "[RAW] ADC=%lu, center=%lu, dev=%+d%%\r\n",
             (unsigned long)adc_value, (unsigned long)center, deviation);
    HAL_UART_Transmit(&huart2,(uint8_t*)resp,strlen(resp),HAL_MAX_DELAY);
  }
  else if (strncmp(cmd,"SENS",4)==0) {
    int v=0; if (sscanf(cmd+4,"%d",&v)>=1) {
      if (v==1||v==2||v==5||v==10) { sensitivity=v; snprintf(resp,sizeof(resp),"[OK] Sens=%d\r\n",v); }
      else snprintf(resp,sizeof(resp),"[ERR] Allowed 1 2 5 10\r\n");
    } else snprintf(resp,sizeof(resp),"[ERR] Usage SENS N\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)resp,strlen(resp),HAL_MAX_DELAY);
  }
  else if (strncmp(cmd,"CENTER",6)==0 || strncmp(cmd,"CALIBRATE",9)==0) {
    center = adc_value; snprintf(resp,sizeof(resp),"[OK] Center=%lu\r\n",(unsigned long)center);
    HAL_UART_Transmit(&huart2,(uint8_t*)resp,strlen(resp),HAL_MAX_DELAY);
  }
  else if (strncmp(cmd,"LED MODE",8)==0 || strncmp(cmd,"LEDMODE",7)==0) {
    led_mode_relative = !led_mode_relative; snprintf(resp,sizeof(resp),"[OK] LED MODE: %s\r\n",led_mode_relative?"REL":"ABS");
    HAL_UART_Transmit(&huart2,(uint8_t*)resp,strlen(resp),HAL_MAX_DELAY);
  }
  else if (strncmp(cmd,"BEEP ON",7)==0) { beep_enable=true; HAL_UART_Transmit(&huart2,(uint8_t*)"[OK] BEEP ON\r\n",10,HAL_MAX_DELAY); }
  else if (strncmp(cmd,"BEEP OFF",8)==0) { beep_enable=false; HAL_GPIO_WritePin(LED_PORT,PIEZO_PIN,GPIO_PIN_RESET); HAL_UART_Transmit(&huart2,(uint8_t*)"[OK] BEEP OFF\r\n",11,HAL_MAX_DELAY); }
  else if (strncmp(cmd,"START",5)==0) { if(!measuring_enabled) { HAL_ADC_Start_IT(&hadc1); measuring_enabled=true;} HAL_UART_Transmit(&huart2,(uint8_t*)"[OK] START\r\n",11,HAL_MAX_DELAY); }
  else if (strncmp(cmd,"STOP",4)==0) { if(measuring_enabled) { HAL_ADC_Stop_IT(&hadc1); measuring_enabled=false; HAL_GPIO_WritePin(LED_PORT,LED1_PIN|LED2_PIN|LED3_PIN|PIEZO_PIN,GPIO_PIN_RESET);} HAL_UART_Transmit(&huart2,(uint8_t*)"[OK] STOP\r\n",10,HAL_MAX_DELAY); }
  else if (strncmp(cmd,"VALUE",5)==0) { snprintf(resp,sizeof(resp),"[OK] ADC=%lu\r\n",(unsigned long)adc_value); HAL_UART_Transmit(&huart2,(uint8_t*)resp,strlen(resp),HAL_MAX_DELAY); }
  else if (strncmp(cmd,"THRESHOLD",9)==0) { int v=0; if (sscanf(cmd+9,"%d",&v)>=1 && v>=0 && v<=100) { threshold_percent=v; snprintf(resp,sizeof(resp),"[OK] THR=%d%%\r\n",v);} else snprintf(resp,sizeof(resp),"[ERR] THRESHOLD N (0-100)\r\n"); HAL_UART_Transmit(&huart2,(uint8_t*)resp,strlen(resp),HAL_MAX_DELAY); }
  else if (strncmp(cmd,"STATUS",6)==0) { snprintf(resp,sizeof(resp),"[STATUS] C=%lu ADC=%lu Dev=%+d%% Sens=%d Thr=%d LED=%s BEEP=%s\r\n",(unsigned long)center,(unsigned long)adc_value,deviation,sensitivity,threshold_percent, led_mode_relative?"REL":"ABS", beep_enable?"ON":"OFF"); HAL_UART_Transmit(&huart2,(uint8_t*)resp,strlen(resp),HAL_MAX_DELAY); }
  else if (strncmp(cmd,"RESET",5)==0) { center=2048;sensitivity=5;threshold_percent=10;led_mode_relative=true;beep_enable=true;measuring_enabled=true; HAL_ADC_Start_IT(&hadc1); HAL_UART_Transmit(&huart2,(uint8_t*)"[OK] RESET\r\n",10,HAL_MAX_DELAY); }
  /* ИСПРАВЛЕНО: вывод HELP в столбик + пустая строка после */
  else if (strncmp(cmd, "HELP", 4) == 0) {
      char help_buf[512];
      int len = 0;
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "[HELP] Available commands:\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  ANGLE         – show current angle deviation\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  RAW           – show raw ADC, center, deviation\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  SENS <N>      – set sensitivity (1,2,5,10)\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  CENTER        – calibrate center to current ADC\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  LED MODE      – toggle relative/absolute LED mode\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  BEEP ON/OFF   – enable/disable beep\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  START         – start measurements\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  STOP          – stop measurements\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  STATUS        – show system status\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  RESET         – reset to default settings\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  VALUE         – show current ADC value\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  THRESHOLD <N> – set alarm threshold (0-100%%)\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "  HELP          – show this help\r\n");
      len += snprintf(help_buf + len, sizeof(help_buf) - len, "\r\n"); // пустая строка для разделения
      HAL_UART_Transmit(&huart2, (uint8_t*)help_buf, len, HAL_MAX_DELAY);
  }
  else { HAL_UART_Transmit(&huart2,(uint8_t*)"[ERR] Unknown command. HELP\r\n",28,HAL_MAX_DELAY); }
}

/* LEDs update */
void update_leds(void)
{
  if (!measuring_enabled) { HAL_GPIO_WritePin(LED_PORT, LED1_PIN|LED2_PIN|LED3_PIN|PIEZO_PIN, GPIO_PIN_RESET); return; }

  if (!led_mode_relative) {
    uint32_t pct = (adc_value * 100UL) / 4095UL;
    HAL_GPIO_WritePin(LED_PORT, LED1_PIN|LED2_PIN|LED3_PIN, GPIO_PIN_RESET);
    if (pct <= 33) HAL_GPIO_WritePin(LED_PORT, LED1_PIN, GPIO_PIN_SET);
    else if (pct >= 67) HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
    return;
  }

  HAL_GPIO_WritePin(LED_PORT, LED1_PIN|LED2_PIN|LED3_PIN, GPIO_PIN_RESET);
  int absdev = abs(deviation);
  if (absdev <= sensitivity) { HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET); return; }
  if (absdev <= sensitivity + 1) { HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET); return; }

  if (deviation < 0) {
    if (absdev > threshold_percent) {
      uint32_t now = HAL_GetTick();
      if (now - last_blink_ms >= blink_interval_ms) { blink = !blink; last_blink_ms = now; }
      if (blink) HAL_GPIO_WritePin(LED_PORT, LED1_PIN, GPIO_PIN_SET);
    } else HAL_GPIO_WritePin(LED_PORT, LED1_PIN, GPIO_PIN_SET);
  } else {
    if (absdev > threshold_percent) {
      uint32_t now = HAL_GetTick();
      if (now - last_blink_ms >= blink_interval_ms) { blink = !blink; last_blink_ms = now; }
      if (blink) HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
    } else HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
  }
}

/* Beep handler */
void Beep_Handler(void)
{
  uint32_t now = HAL_GetTick();
  if (measuring_enabled && beep_enable && (abs(deviation) > threshold_percent)) {
    if (now - last_piezo_ms >= piezo_toggle_ms) { HAL_GPIO_TogglePin(LED_PORT, PIEZO_PIN); last_piezo_ms = now; }
  } else {
    HAL_GPIO_WritePin(LED_PORT, PIEZO_PIN, GPIO_PIN_RESET);
  }
}

/* Buttons */
void Check_Buttons(void)
{
  if (HAL_GPIO_ReadPin(BTN1_PORT,BTN1_PIN) == GPIO_PIN_RESET) { center = adc_value; HAL_Delay(200); }
  if (HAL_GPIO_ReadPin(BTN2_PORT,BTN2_PIN) == GPIO_PIN_RESET) { if (sensitivity==1) sensitivity=2; else if (sensitivity==2) sensitivity=5; else if (sensitivity==5) sensitivity=10; else sensitivity=1; HAL_Delay(200); }
  if (HAL_GPIO_ReadPin(BTN3_PORT,BTN3_PIN) == GPIO_PIN_RESET) { center=2048;sensitivity=5;threshold_percent=10;led_mode_relative=true;beep_enable=true;measuring_enabled=true; HAL_ADC_Start_IT(&hadc1); HAL_Delay(200); }
}

/* ---------------- MX minimal functions --------------------------
   ВАЖНО: НИКОГДА не инициализируйте PA0 как GPIO input здесь!
   PA0 должен остаться свободным для ADC (analog).
-----------------------------------------------------------------*/
void SystemClock_Config(void) { /* ... copy your working clock config ... */ }
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();

  /* LEDs & piezo */
  HAL_GPIO_WritePin(LED_PORT, LED1_PIN|LED2_PIN|LED3_PIN|PIEZO_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED1_PIN|LED2_PIN|LED3_PIN|PIEZO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  /* Buttons: PC13, PA4, PB0 (pullups) */
  GPIO_InitStruct.Pin = BTN1_PIN; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(BTN1_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = BTN2_PIN; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(BTN2_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = BTN3_PIN; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(BTN3_PORT, &GPIO_InitStruct);

  /* UART pins PA2/PA3 - AF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Pull = GPIO_PULLUP; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* IMPORTANT: DO NOT configure PA0 here. PA0 must remain analog for ADC1_IN0. */
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_0; sConfig.Rank = 1; sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2; huart2.Init.BaudRate = 115200; huart2.Init.WordLength = UART_WORDLENGTH_8B; huart2.Init.StopBits = UART_STOPBITS_1; huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX; huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
