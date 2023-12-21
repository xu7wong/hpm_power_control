/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef _HPM_BOARD_H
#define _HPM_BOARD_H
// #include <stdio.h>
#include "hpm_common.h"
// #include "hpm_clock_drv.h"
#include "hpm_soc.h"
#include "hpm_soc_feature.h"

#define BOARD_NAME "hpm6200evk"
#define BOARD_UF2_SIGNATURE (0x0A4D5048UL)

#define BOARD_CONSOLE_BAUDRATE (115200UL)

/* LED */
#define BOARD_R_GPIO_CTRL HPM_GPIO0
#define BOARD_R_GPIO_INDEX GPIO_DI_GPIOB
#define BOARD_R_GPIO_PIN 18
#define BOARD_G_GPIO_CTRL HPM_GPIO0
#define BOARD_G_GPIO_INDEX GPIO_DI_GPIOB
#define BOARD_G_GPIO_PIN 19

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


#define LOOGING_LEVEL LOG_INFO
#include <stdint.h>
#include <stdbool.h>
typedef enum
{
    LOG_INFO,
    LOG_DEBUG,
    LOG_WARNING,
    LOG_ERROR
} logging_level_t;
int Logging(logging_level_t level, const char *format, ...);
// typedef void (*board_timer_cb)(void);

// void board_init(void);
void board_init_console(void);

void board_init_clock(void);

// void board_delay_us(uint32_t us);
void board_delay_ms(uint32_t ms);

// Customized
void board_init_led(void);
void board_set_red_led(bool state);
void board_set_green_led(bool state);
void board_toggle_red_led(void);
void board_toggle_green_led(void);
void board_init_adc(void);
void board_init_pwm(void);
void board_init_hrpwm(void);

void board_init_pmp(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */
#endif /* _HPM_BOARD_H */
