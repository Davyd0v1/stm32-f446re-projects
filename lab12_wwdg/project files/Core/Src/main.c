/* main.c — WWDG demo (исправления: менее навязчивый LED2, BUZZ ON/OFF тест) */
/* Nucleo-F446RE, HAL, software buzzer (DWT), UART RX IT */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* weak fallback Error_Handler (если у тебя уже есть, она переопределится) */
void Error_Handler(void) __attribute__((weak));
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
        HAL_Delay(200);
    }
}

/* Handles */
WWDG_HandleTypeDef hwwdg;
UART_HandleTypeDef huart2;

/* Pins */
#define LED_PORT    GPIOA
#define LED1_PIN    GPIO_PIN_5
#define LED2_PIN    GPIO_PIN_6
#define LED3_PIN    GPIO_PIN_7
#define BUZZ_PORT   GPIOA
#define BUZZ_PIN    GPIO_PIN_8

/* WWDG params */
volatile uint32_t wwdg_window = 100;  /* default (65..127 recommended) */
volatile uint32_t wwdg_counter = 127; /* default */
volatile uint8_t  wwdg_enabled = 0;

/* Exec modes */
typedef enum { MODE_NORMAL = 0, MODE_FAST, MODE_SLOW } ExecMode_t;
volatile ExecMode_t exec_mode = MODE_NORMAL;

/* UART buffer */
#define CMD_BUF_LEN 128
static char cmd_buf[CMD_BUF_LEN];
static volatile uint8_t cmd_idx = 0;
static volatile uint8_t cmd_ready = 0;
static uint8_t uart_rx_byte = 0;
static volatile uint8_t echo_pending = 0;
static volatile uint8_t echo_char = 0;

/* Simple event log */
#define LOG_LINES 16
#define LOG_LINE_LEN 64
static char event_log[LOG_LINES][LOG_LINE_LEN];
static int log_head = 0;
static int log_count = 0;
static void log_add(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    vsnprintf(event_log[log_head], LOG_LINE_LEN, fmt, ap);
    va_end(ap);
    log_head = (log_head + 1) % LOG_LINES;
    if (log_count < LOG_LINES) ++log_count;
}

/* Prototypes */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void MX_WWDG_Init(void);

/* UART printf helper */
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &c, 1, 50);
    return ch;
}
static void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)strlen(buf), 200);
}

/* DWT micro timing for software buzzer */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t dwt_cycles(void) { return DWT->CYCCNT; }

/* software beep ~2 kHz on PA8 for ms */
static void beep_ms(uint32_t ms)
{
    dwt_init();
    const uint32_t half_us = 250U; /* 250us half => 2kHz */
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0) cycles_per_us = 1;
    uint32_t cycles_half = cycles_per_us * half_us;
    uint32_t total_cycles = cycles_per_us * 1000U * ms;
    uint32_t start = dwt_cycles();
    while ((uint32_t)(dwt_cycles() - start) < total_cycles) {
        HAL_GPIO_TogglePin(BUZZ_PORT, BUZZ_PIN);
        uint32_t t0 = dwt_cycles();
        while ((uint32_t)(dwt_cycles() - t0) < cycles_half) { __NOP(); }
    }
    HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);
}
static void do_beep(uint8_t repeats, uint16_t on_ms, uint16_t off_ms)
{
    for (uint8_t i = 0; i < repeats; ++i) {
        beep_ms(on_ms);
        if (off_ms) HAL_Delay(off_ms);
    }
}

/* compute WWDG times (integer ms) */
static void compute_wwdg_times_ms_int(uint32_t pclk1_hz, uint32_t prescaler_div,
                                      uint32_t counter, uint32_t window,
                                      uint32_t *min_ms, uint32_t *max_ms, uint32_t *mid_ms)
{
    uint64_t numer = 4096ULL * (uint64_t)prescaler_div * 1000ULL;
    uint32_t tick_ms = (uint32_t)(numer / pclk1_hz);
    if (tick_ms == 0) tick_ms = 1;
    if (counter <= window) *min_ms = 0;
    else *min_ms = tick_ms * (counter - window);
    if (counter <= 64) *max_ms = tick_ms * 1;
    else *max_ms = tick_ms * (counter - 64);
    *mid_ms = (*min_ms + *max_ms) / 2;
}

/* Save stub */
static void SaveCriticalData(void)
{
    uart_printf("Saving critical data (stub)\r\n");
    log_add("Saved critical data");
}

/* EWI callback */
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg_handle)
{
    uart_printf(">>> EWI callback triggered (tick=%lu)\r\n", HAL_GetTick());
    HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
    log_add("EWI");
    SaveCriticalData();
    do_beep(6, 120, 40);
    uart_printf("WWDG EWI executed (beep)\r\n");
}

/* Button EXTI callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        exec_mode = MODE_FAST; uart_printf("BTN1 -> MODE_FAST\r\n"); log_add("BTN1 FAST");
    } else if (GPIO_Pin == GPIO_PIN_0) {
        exec_mode = MODE_SLOW; uart_printf("BTN2 -> MODE_SLOW\r\n"); log_add("BTN2 SLOW");
    } else if (GPIO_Pin == GPIO_PIN_1) {
        exec_mode = MODE_NORMAL; uart_printf("BTN3 -> MODE_NORMAL\r\n"); log_add("BTN3 NORMAL");
    }
}

/* Start WWDG */
void MX_WWDG_Init(void)
{
    if (!(wwdg_window >= 65 && wwdg_window <= 127 && wwdg_counter >= 64 && wwdg_counter <= 127 && wwdg_counter > wwdg_window)) {
        uart_printf("WWDG params invalid: window=%lu counter=%lu (need 65..127 and counter>window)\r\n", (unsigned long)wwdg_window, (unsigned long)wwdg_counter);
        log_add("WWDG params invalid");
        return;
    }

    HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_RESET); /* clear red on start */

    hwwdg.Instance = WWDG;
    hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
    hwwdg.Init.Window = (uint32_t)(wwdg_window & 0x7F);
    hwwdg.Init.Counter = (uint32_t)(wwdg_counter & 0x7F);
    hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
    if (HAL_WWDG_Init(&hwwdg) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(WWDG_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(WWDG_IRQn);

    wwdg_enabled = 1;
    uart_printf("WWDG started: window=%lu counter=%lu\r\n", (unsigned long)wwdg_window, (unsigned long)wwdg_counter);
    log_add("WWDG ON w=%lu c=%lu", (unsigned long)wwdg_window, (unsigned long)wwdg_counter);
}

/* Kick WWDG */
static void KickWWDG(void)
{
    if (!wwdg_enabled) return;
    if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK) {
        uart_printf("WWDG refresh failed\r\n");
        log_add("WWDG refresh failed");
    } else {
        HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
        uart_printf("WWDG KICK at tick=%lu\r\n", HAL_GetTick());
        log_add("KICK %lu", HAL_GetTick());
    }
}

/* Process UART commands */
static void ProcessCommand(const char *cmd)
{
    int v;
    if (strncmp(cmd, "HELP", 4) == 0) {
        uart_printf("Commands: HELP STATUS WWDG ON WWDG OFF WINDOW <n> COUNTER <n> FAST SLOW NORMAL RESET CAUSE KICK LOG HANG BEEP <ms> BUZZ ON BUZZ OFF\r\n");
    } else if (strncmp(cmd, "STATUS", 6) == 0) {
        uart_printf("Mode: %s, WWDG: %s window=%lu counter=%lu\r\n", exec_mode==MODE_NORMAL?"NORMAL":(exec_mode==MODE_FAST?"FAST":"SLOW"), wwdg_enabled?"ON":"OFF", wwdg_window, wwdg_counter);
    } else if (strncmp(cmd, "WWDG ON", 7) == 0) {
        MX_WWDG_Init();
    } else if (strncmp(cmd, "WWDG OFF", 8) == 0) {
        wwdg_enabled = 0;
        HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_RESET); /* explicitly clear red */
        HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
        uart_printf("WWDG disabled (software)\r\n");
        log_add("WWDG OFF");
    } else if (sscanf(cmd, "WINDOW %d", &v) == 1) {
        if (v >= 65 && v <= 127) { wwdg_window = (uint32_t)v; uart_printf("Window set to %d (applies on next WWDG start)\r\n", v); log_add("Window=%d", v); }
        else uart_printf("Window must be 65..127\r\n");
    } else if (sscanf(cmd, "COUNTER %d", &v) == 1) {
        if (v >= 64 && v <= 127) { wwdg_counter = (uint32_t)v; uart_printf("Counter set to %d (applies on next WWDG start)\r\n", v); log_add("Counter=%d", v); }
        else uart_printf("Counter must be 64..127\r\n");
    } else if (strncmp(cmd, "FAST", 4) == 0) { exec_mode = MODE_FAST; uart_printf("Mode FAST\r\n"); log_add("Mode FAST"); }
    else if (strncmp(cmd, "SLOW", 4) == 0) { exec_mode = MODE_SLOW; uart_printf("Mode SLOW\r\n"); log_add("Mode SLOW"); }
    else if (strncmp(cmd, "NORMAL", 6) == 0) { exec_mode = MODE_NORMAL; uart_printf("Mode NORMAL\r\n"); log_add("Mode NORMAL"); }
    else if (strncmp(cmd, "KICK", 4) == 0) { KickWWDG(); uart_printf("Manual KICK\r\n"); log_add("Manual KICK"); }
    else if (strncmp(cmd, "RESET", 5) == 0) { uart_printf("Software reset...\r\n"); log_add("Software reset"); NVIC_SystemReset(); }
    else if (strncmp(cmd, "CAUSE", 5) == 0) {
        uint8_t cause = 0;
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) cause |= 1;
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) cause |= 2;
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) cause |= 4;
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) cause |= 8;
        __HAL_RCC_CLEAR_RESET_FLAGS();
        if (cause & 4) uart_printf("Last reset: WWDG\r\n");
        else if (cause) uart_printf("Last reset: other (0x%02X)\r\n", cause);
        else uart_printf("Last reset: unknown\r\n");
        log_add("CAUSE");
    } else if (strncmp(cmd, "HANG", 4) == 0) {
        uart_printf("Simulating hang (infinite loop)...\r\n");
        log_add("HANG");
        while (1) {
            HAL_Delay(100); // даём жить прерываниям!
        } /* if WWDG enabled -> reset */
    } else if (strncmp(cmd, "LOG", 3) == 0) {
        uart_printf("Event log (last %d):\r\n", LOG_LINES);
        int idx = (log_head - log_count + LOG_LINES) % LOG_LINES;
        for (int i = 0; i < log_count; ++i) {
            uart_printf("%02d: %s\r\n", i+1, event_log[idx]);
            idx = (idx + 1) % LOG_LINES;
        }
    } else if (strncmp(cmd, "BEEP", 4) == 0) {
        if (sscanf(cmd+4, "%d", &v) == 1 && v > 0) { uart_printf("Beep %d ms\r\n", v); beep_ms((uint32_t)v); }
        else { uart_printf("Manual beep\r\n"); do_beep(3, 150, 80); }
        log_add("Manual BEEP");
    } else if (strncmp(cmd, "BUZZ ON", 7) == 0) {
        /* Hardware test: drive PA8 high (DC) — useful for active buzzers */
        HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_SET);
        uart_printf("BUZZ ON (PA8 high) - hardware test for active buzzer\r\n");
        log_add("BUZZ ON");
    } else if (strncmp(cmd, "BUZZ OFF", 8) == 0) {
        HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);
        uart_printf("BUZZ OFF\r\n");
        log_add("BUZZ OFF");
    } else {
        uart_printf("Unknown: %s\r\n", cmd);
    }
}

/* UART RxCplt callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t b = uart_rx_byte;
        echo_char = b; echo_pending = 1;
        if (b == '\r' || b == '\n') {
            if (cmd_idx > 0) { cmd_buf[cmd_idx] = 0; cmd_ready = 1; cmd_idx = 0; }
        } else {
            if (cmd_idx < (CMD_BUF_LEN - 1)) cmd_buf[cmd_idx++] = (char)b;
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

/* main */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();

    /* clear possible stale indicators */
    HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);

    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    uart_printf("WWDG demo start. window=%lu counter=%lu\r\n", (unsigned long)wwdg_window, (unsigned long)wwdg_counter);
    uart_printf("Type HELP (end with CR/LF)\r\n");
    log_add("System start");

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
        uart_printf("Previous reset: WWDG\r\n");
        log_add("Prev reset: WWDG");
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();

    wwdg_enabled = 0;

    /* compute WWDG timing (for info) */
    const uint32_t PCLK1_HZ = 50000000UL; /* APB1 = 50 MHz (as set in SystemClock_Config) */
    const uint32_t PRESC_DIV = 8;
    uint32_t min_ms = 0, max_ms = 0, mid_ms = 0;
    compute_wwdg_times_ms_int(PCLK1_HZ, PRESC_DIV, wwdg_counter, wwdg_window, &min_ms, &max_ms, &mid_ms);
    uart_printf("WWDG timing: min=%lums max=%lums mid=%lums\r\n", (unsigned long)min_ms, (unsigned long)max_ms, (unsigned long)mid_ms);
    log_add("Times min=%lums max=%lums", (unsigned long)min_ms, (unsigned long)max_ms);

    uint32_t last_refresh_tick = HAL_GetTick();
    uint32_t last_blink = HAL_GetTick();
    const uint32_t blink_interval = 500; /* LED1 blink */
    uint32_t refresh_interval_ms = (mid_ms < 5 ? 10 : mid_ms);
    GPIO_PinState led1_state = HAL_GPIO_ReadPin(LED_PORT, LED1_PIN);

    while (1) {
        /* echo */
        if (echo_pending) {
            uint8_t c = echo_char;
            HAL_UART_Transmit(&huart2, &c, 1, 20);
            echo_pending = 0;
        }

        /* commands */
        if (cmd_ready) {
            ProcessCommand(cmd_buf);
            /* recompute timings after changes */
            compute_wwdg_times_ms_int(PCLK1_HZ, PRESC_DIV, wwdg_counter, wwdg_window, &min_ms, &max_ms, &mid_ms);
            refresh_interval_ms = (mid_ms < 5 ? 10 : mid_ms);
            uart_printf("WWDG timing: min=%lums max=%lums mid=%lums\r\n", (unsigned long)min_ms, (unsigned long)max_ms, (unsigned long)mid_ms);
            cmd_ready = 0;
        }

        /* behaviors */
        if (exec_mode == MODE_NORMAL) {
            /* LED1 blink */
            if ((HAL_GetTick() - last_blink) >= blink_interval) {
                last_blink = HAL_GetTick();
                HAL_GPIO_TogglePin(LED_PORT, LED1_PIN);
                led1_state = HAL_GPIO_ReadPin(LED_PORT, LED1_PIN);
            }

            /* LED2 approach: only when WWDG enabled, and only during last fraction of interval */
            if (wwdg_enabled == 1) {
                /* use 20% of refresh interval or at least 10ms, but not more than refresh_interval */
                uint32_t approach_margin = (refresh_interval_ms * 20) / 100;
                if (approach_margin < 10) approach_margin = (refresh_interval_ms / 2); /* small intervals -> half */
                if (approach_margin > refresh_interval_ms) approach_margin = refresh_interval_ms;
                uint32_t elapsed = HAL_GetTick() - last_refresh_tick;
                /* turn on only if elapsed is within [refresh_interval - approach_margin, refresh_interval) */
                if ((elapsed >= (refresh_interval_ms > approach_margin ? refresh_interval_ms - approach_margin : 0)) && (elapsed < refresh_interval_ms)) {
                    HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
                } else {
                    HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
                }

            } else {
                HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
            }

            /* refresh at midpoint */
            if ((HAL_GetTick() - last_refresh_tick) >= refresh_interval_ms) {
                KickWWDG();
                last_refresh_tick = HAL_GetTick();
            }

            HAL_Delay(5);
        }
        else if (exec_mode == MODE_FAST) {
            uart_printf("FAST demo: rapid kicks (may cause immediate reset)\r\n");
            log_add("FAST demo");
            for (int i = 0; i < 25; ++i) { KickWWDG(); HAL_Delay(2); }
            exec_mode = MODE_NORMAL;
            uart_printf("FAST demo done\r\n");
            log_add("FAST done");
        }
        else { /* SLOW */
            uart_printf("SLOW demo: long task, skipping refresh (EWI expected)\r\n");
            log_add("SLOW start");
            led1_state = HAL_GPIO_ReadPin(LED_PORT, LED1_PIN);
            uint32_t hang_start = HAL_GetTick();
            while ((HAL_GetTick() - hang_start) < 2000) {
                HAL_GPIO_WritePin(LED_PORT, LED1_PIN, led1_state); /* freeze LED1 in current state */
                HAL_Delay(50); /* allow interrupts */
            }
            exec_mode = MODE_NORMAL;
            uart_printf("SLOW demo done\r\n");
            log_add("SLOW done");
            last_refresh_tick = HAL_GetTick();
            last_blink = HAL_GetTick();
        }
        HAL_Delay(2);
    }
}

/* GPIO init */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* LEDs + BUZZ output */
    GPIO_InitStruct.Pin = LED1_PIN | LED2_PIN | LED3_PIN | BUZZ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_PORT, LED1_PIN | LED2_PIN | LED3_PIN | BUZZ_PIN, GPIO_PIN_RESET);

    /* Buttons: PC13, PA0, PA1 -> EXTI falling */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* NVIC */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0); HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0); HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0); HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USART2 init */
void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* SystemClock_Config: HSE 8MHz -> SYSCLK 100MHz, APB1=50MHz */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 200;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; /* 50 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; /* 100 MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}
