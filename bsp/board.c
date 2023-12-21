/*
 * Copyright (c) 2023 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *
 */

#include "board.h"
#include "hpm_uart_drv.h"
// #include "hpm_gptmr_drv.h"
// #include "hpm_lcdc_drv.h"
// #include "hpm_i2c_drv.h"
#include "hpm_gpio_drv.h"
// #include "pinmux.h"
#include "hpm_interrupt.h"
#include "hpm_pmp_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_trgm_drv.h"
// #include "hpm_trgm_regs.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_adc16_drv.h"
// #include "hpm_pcfg_drv.h"

// static board_timer_cb timer_cb;

/**
 * @brief FLASH configuration option definitions:
 * option[0]:
 *    [31:16] 0xfcf9 - FLASH configuration option tag
 *    [15:4]  0 - Reserved
 *    [3:0]   option words (exclude option[0])
 * option[1]:
 *    [31:28] Flash probe type
 *      0 - SFDP SDR / 1 - SFDP DDR
 *      2 - 1-4-4 Read (0xEB, 24-bit address) / 3 - 1-2-2 Read(0xBB, 24-bit address)
 *      4 - HyperFLASH 1.8V / 5 - HyperFLASH 3V
 *      6 - OctaBus DDR (SPI -> OPI DDR)
 *      8 - Xccela DDR (SPI -> OPI DDR)
 *      10 - EcoXiP DDR (SPI -> OPI DDR)
 *    [27:24] Command Pads after Power-on Reset
 *      0 - SPI / 1 - DPI / 2 - QPI / 3 - OPI
 *    [23:20] Command Pads after Configuring FLASH
 *      0 - SPI / 1 - DPI / 2 - QPI / 3 - OPI
 *    [19:16] Quad Enable Sequence (for the device support SFDP 1.0 only)
 *      0 - Not needed
 *      1 - QE bit is at bit 6 in Status Register 1
 *      2 - QE bit is at bit1 in Status Register 2
 *      3 - QE bit is at bit7 in Status Register 2
 *      4 - QE bit is at bit1 in Status Register 2 and should be programmed by 0x31
 *    [15:8] Dummy cycles
 *      0 - Auto-probed / detected / default value
 *      Others - User specified value, for DDR read, the dummy cycles should be 2 * cycles on FLASH datasheet
 *    [7:4] Misc.
 *      0 - Not used
 *      1 - SPI mode
 *      2 - Internal loopback
 *      3 - External DQS
 *    [3:0] Frequency option
 *      1 - 30MHz / 2 - 50MHz / 3 - 66MHz / 4 - 80MHz / 5 - 100MHz / 6 - 120MHz / 7 - 133MHz / 8 - 166MHz
 *
 * option[2] (Effective only if the bit[3:0] in option[0] > 1)
 *    [31:20]  Reserved
 *    [19:16] IO voltage
 *      0 - 3V / 1 - 1.8V
 *    [15:12] Pin group
 *      0 - 1st group / 1 - 2nd group
 *    [11:8] Connection selection
 *      0 - CA_CS0 / 1 - CB_CS0 / 2 - CA_CS0 + CB_CS0 (Two FLASH connected to CA and CB respectively)
 *    [7:0] Drive Strength
 *      0 - Default value
 * option[3] (Effective only if the bit[3:0] in option[0] > 2, required only for the QSPI NOR FLASH that not supports
 *              JESD216)
 *    [31:16] reserved
 *    [15:12] Sector Erase Command Option, not required here
 *    [11:8]  Sector Size Option, not required here
 *    [7:0] Flash Size Option
 *      0 - 4MB / 1 - 8MB / 2 - 16MB
 */
#if defined(FLASH_XIP) && FLASH_XIP
__attribute__((section(".nor_cfg_option"))) const uint32_t option[4] = {0xfcf90001, 0x00000007, 0x0, 0x0};
#endif

#if defined(FLASH_UF2) && FLASH_UF2
ATTR_PLACE_AT(".uf2_signature")
const uint32_t uf2_signature = BOARD_UF2_SIGNATURE;
#endif

void board_delay_ms(uint32_t ms)
{
    clock_cpu_delay_ms(ms);
}

void board_init_clock()
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);
    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ)
    {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }
    /* Add most Clocks to group 0 */
    /* not open uart clock in this API, uart should configure pin function before opening clock */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahbp, 0);
    clock_add_to_group(clock_axic, 0);
    clock_add_to_group(clock_axis, 0);

    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_gptmr0, 0);
    clock_add_to_group(clock_gptmr1, 0);
    clock_add_to_group(clock_gptmr2, 0);
    clock_add_to_group(clock_gptmr3, 0);
    clock_add_to_group(clock_i2c0, 0);
    clock_add_to_group(clock_i2c1, 0);
    clock_add_to_group(clock_i2c2, 0);
    clock_add_to_group(clock_i2c3, 0);
    clock_add_to_group(clock_lin0, 0);
    clock_add_to_group(clock_lin1, 0);
    clock_add_to_group(clock_lin2, 0);
    clock_add_to_group(clock_lin3, 0);
    clock_add_to_group(clock_spi0, 0);
    clock_add_to_group(clock_spi1, 0);
    clock_add_to_group(clock_spi2, 0);
    clock_add_to_group(clock_spi3, 0);
    clock_add_to_group(clock_can0, 0);
    clock_add_to_group(clock_can1, 0);
    clock_add_to_group(clock_can2, 0);
    clock_add_to_group(clock_can3, 0);
    clock_add_to_group(clock_ptpc, 0);
    clock_add_to_group(clock_ref0, 0);
    clock_add_to_group(clock_ref1, 0);
    clock_add_to_group(clock_watchdog0, 0);
    clock_add_to_group(clock_sdp, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_usb0, 0);
    clock_add_to_group(clock_kman, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_mbx0, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_rng, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_mot1, 0);
    clock_add_to_group(clock_mot2, 0);
    clock_add_to_group(clock_mot3, 0);
    clock_add_to_group(clock_acmp, 0);
    clock_add_to_group(clock_msyn, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_lmm1, 0);

    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_adc1, 0);
    clock_add_to_group(clock_adc2, 0);

    clock_add_to_group(clock_dac0, 0);
    clock_add_to_group(clock_dac1, 0);

    clock_add_to_group(clock_tsns, 0);
    clock_add_to_group(clock_crc0, 0);
    clock_add_to_group(clock_sdm0, 0);

    clock_add_to_group(clock_uart0, 0);

    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Add the CPU1 clock to Group1 */
    clock_add_to_group(clock_mchtmr1, 1);

    /* Connect Group1 to CPU1 */
    clock_connect_group_to_cpu(1, 1);

    /* Bump up DCDC voltage to 1275mv */
    // pcfg_dcdc_set_voltage(HPM_PCFG, 1275);

    /* Connect CAN2/CAN3 to pll0clk0*/
    clock_set_source_divider(clock_can2, clk_src_pll0_clk0, 1);
    clock_set_source_divider(clock_can3, clk_src_pll0_clk0, 1);

    /* Configure CPU to 600MHz, AXI/AHB to 200MHz */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll1_clk0, 1, 3, 3);
    /* Configure PLL1_CLK0 Post Divider to 1 */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 1, 0, 0);
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, 1, 600000000);
    clock_update_core_clock();

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
    clock_set_source_divider(clock_mchtmr1, clk_src_osc24m, 1);
}

void board_init_console()
{
    HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART0_RXD;
    HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART0_TXD;
    /* PY port IO needs to configure PIOC */
    HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_SOC_GPIO_Y_07;
    HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_SOC_GPIO_Y_06;

    clock_set_source_divider(clock_uart0, clk_src_osc24m, 1U);

    hpm_stat_t stat = status_fail;
    uart_config_t config = {0};
    uart_default_config(HPM_UART0, &config);
    config.src_freq_in_hz = clock_get_frequency(clock_uart0);
    config.baudrate = BOARD_CONSOLE_BAUDRATE;
    stat = uart_init(HPM_UART0, &config);
    if (status_success == stat)
    {
    }
}

void board_init_led(void)
{
    HPM_IOC->PAD[IOC_PAD_PB18].FUNC_CTL = IOC_PB18_FUNC_CTL_GPIO_B_18;
    HPM_IOC->PAD[IOC_PAD_PB19].FUNC_CTL = IOC_PB19_FUNC_CTL_GPIO_B_19;
    // gpio_set_pin_output_with_initial(BOARD_R_GPIO_CTRL, BOARD_R_GPIO_INDEX, BOARD_R_GPIO_PIN, 0);
    gpio_set_pin_output_with_initial(BOARD_G_GPIO_CTRL, BOARD_G_GPIO_INDEX, BOARD_G_GPIO_PIN, 1);
    gpio_set_pin_output_with_initial(BOARD_R_GPIO_CTRL, BOARD_R_GPIO_INDEX, BOARD_R_GPIO_PIN, 1);

    // HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL = IOC_PB00_FUNC_CTL_GPIO_B_00;
    // HPM_IOC->PAD[IOC_PAD_PB01].FUNC_CTL = IOC_PB01_FUNC_CTL_GPIO_B_01;
    // HPM_IOC->PAD[IOC_PAD_PB02].FUNC_CTL = IOC_PB02_FUNC_CTL_GPIO_B_02;
    // HPM_IOC->PAD[IOC_PAD_PB03].FUNC_CTL = IOC_PB03_FUNC_CTL_GPIO_B_03;

    // gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DI_GPIOB, 0, 1);
    // gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DI_GPIOB, 1, 1);
    // gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DI_GPIOB, 2, 1);
    // gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DI_GPIOB, 3, 1);
}

void board_set_red_led(bool state)
{
    gpio_write_pin(BOARD_R_GPIO_CTRL, BOARD_R_GPIO_INDEX, BOARD_R_GPIO_PIN, !state);
}
void board_set_green_led(bool state)
{
    gpio_write_pin(BOARD_G_GPIO_CTRL, BOARD_G_GPIO_INDEX, BOARD_G_GPIO_PIN, !state);
}
void board_toggle_red_led(void)
{
    gpio_toggle_pin(BOARD_R_GPIO_CTRL, BOARD_R_GPIO_INDEX, BOARD_R_GPIO_PIN);
}
void board_toggle_green_led(void)
{
    gpio_toggle_pin(BOARD_G_GPIO_CTRL, BOARD_G_GPIO_INDEX, BOARD_G_GPIO_PIN);
}

static void reset_pwm_counter(void)
{
    pwm_enable_reload_at_synci(HPM_PWM1);
    // trgm_output_update_source(HPM_TRGM0, TRGM_TRGOCFG_PWM0_SYNCI, 1);
    // trgm_output_update_source(HPM_TRGM0, TRGM_TRGOCFG_PWM0_SYNCI, 0);
}

uint32_t reload = 0;

// #define TEST_LOOP (200)
#define PWM_PERIOD_IN_US (10)
void board_init_pwm(void)
{
    uint32_t freq = clock_get_frequency(clock_mot1);

    reload = freq / 1000000 * PWM_PERIOD_IN_US - 1;
    Logging(LOG_DEBUG, "pwm clock = %d, reload = %d\n", freq, reload);

    HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL = IOC_PB00_FUNC_CTL_PWM1_P_0;
    HPM_IOC->PAD[IOC_PAD_PB01].FUNC_CTL = IOC_PB01_FUNC_CTL_PWM1_P_1;
    HPM_IOC->PAD[IOC_PAD_PB02].FUNC_CTL = IOC_PB02_FUNC_CTL_PWM1_P_2;
    HPM_IOC->PAD[IOC_PAD_PB03].FUNC_CTL = IOC_PB03_FUNC_CTL_PWM1_P_3;

    // HPM_IOC->PAD[IOC_PAD_PB17].FUNC_CTL = IOC_PB17_FUNC_CTL_PWM0_P_5;
    // HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_PWM0_P_3;
    // HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_PWM0_P_1;
    // HPM_IOC->PAD[IOC_PAD_PB16].FUNC_CTL = IOC_PB16_FUNC_CTL_PWM0_P_4;
    // HPM_IOC->PAD[IOC_PAD_PB14].FUNC_CTL = IOC_PB14_FUNC_CTL_PWM0_P_2;
    // HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_PWM0_P_0;

    uint8_t cmp_index = 0;
    // uint32_t duty, duty_step;
    // bool increase_duty_cycle = true;
    pwm_cmp_config_t cmp_config[3] = {0};

    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = reload / 4;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_modify;
    cmp_config[0].enable_hrcmp = true;

    cmp_config[1].mode = pwm_cmp_mode_output_compare;
    cmp_config[1].cmp = reload * 3 / 4;
    cmp_config[1].update_trigger = pwm_shadow_register_update_on_modify;
    cmp_config[1].enable_hrcmp = true;

    cmp_config[2].mode = pwm_cmp_mode_output_compare;
    cmp_config[2].cmp = reload / 4;
    cmp_config[2].update_trigger = pwm_shadow_register_update_on_modify;
    cmp_config[2].enable_hrcmp = true;

    cmp_config[3].mode = pwm_cmp_mode_output_compare;
    cmp_config[3].cmp = reload * 3 / 4;
    cmp_config[3].update_trigger = pwm_shadow_register_update_on_modify;
    cmp_config[3].enable_hrcmp = true;

    // pwm_config_t pwm_config = {0};

    pwm_pair_config_t pwm_pair_config;
    pwm_get_default_pwm_pair_config(HPM_PWM1, &pwm_pair_config);
    pwm_pair_config.pwm[0].enable_output = true;
    pwm_pair_config.pwm[0].dead_zone_in_half_cycle = 1;
    pwm_pair_config.pwm[0].invert_output = false;

    pwm_pair_config.pwm[1].enable_output = true;
    pwm_pair_config.pwm[1].dead_zone_in_half_cycle = 1;
    pwm_pair_config.pwm[1].invert_output = false;

    pwm_setup_waveform_in_pair(HPM_PWM1, 0, &pwm_pair_config, cmp_index, &cmp_config[0], 2);
    // Logging(LOG_DEBUG, "setup = %d,\n", s);
    // pwm_pair_config.pwm->invert_output = false;
    // pwm_setup_waveform_in_pair(HPM_PWM1, 1, &pwm_pair_config, cmp_index + 1, &cmp_config[0], 1);
    // pwm_pair_config.pwm->invert_output = true;
    pwm_setup_waveform_in_pair(HPM_PWM1, 2, &pwm_pair_config, cmp_index + 2, &cmp_config[0], 2);
    // pwm_setup_waveform_in_pair(HPM_PWM1, 2, &pwm_pair_config, cmp_index + 2, &cmp_config[1], 1);
    //  pwm_pair_config.pwm->invert_output = false;
    //  pwm_setup_waveform_in_pair(HPM_PWM1, 3, &pwm_pair_config, cmp_index + 3, &cmp_config[0], 1);

    pwm_stop_counter(HPM_PWM1);
    reset_pwm_counter();
    // pwm_get_default_pwm_config(HPM_PWM1, &pwm_config);

    // pwm_config.enable_output = true;
    // pwm_config.dead_zone_in_half_cycle = 0;
    // pwm_config.invert_output = false;

    /*
     * reload and start counter
     */
    pwm_set_reload(HPM_PWM1, 0, reload);

    intc_m_enable_irq_with_priority(IRQn_PWM1, 6);
    pwm_enable_irq(HPM_PWM1, PWM_IRQ_HALF_RELOAD | PWM_IRQ_RELOAD);
    // pwm_enable_irq(HPM_PWM1, PWM_IRQ_CMP(0));

    pwm_set_start_count(HPM_PWM1, 0, 0);

    pwm_load_cmp_shadow_on_match(HPM_PWM1, cmp_index, &cmp_config[0]);
    // pwm_load_cmp_shadow_on_match(HPM_PWM1, cmp_index, &cmp_config[0]);
    pwm_start_counter(HPM_PWM1);
    pwm_issue_shadow_register_lock_event(HPM_PWM1);

    pwm_update_raw_cmp_edge_aligned(HPM_PWM1, cmp_index, 0);
    pwm_update_raw_cmp_edge_aligned(HPM_PWM1, cmp_index + 1, reload / 2);

    pwm_update_raw_cmp_edge_aligned(HPM_PWM1, cmp_index + 2, 200);
    pwm_update_raw_cmp_edge_aligned(HPM_PWM1, cmp_index + 3, reload / 2 + 200);
}

#define HRPWM_SET_IN_PWM_CLK (128)
#define PWM_PERIOD_IN_MS (0.01)
#define TEST_LOOP (100)
void board_init_hrpwm(void)
{
    uint32_t freq = clock_get_frequency(clock_mot1);

    reload = freq / 1000 * PWM_PERIOD_IN_MS - 1;
    Logging(LOG_DEBUG, "pwm clock = %d, reload = %d\n", freq, reload);

    HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL = IOC_PB00_FUNC_CTL_PWM1_P_0;
    HPM_IOC->PAD[IOC_PAD_PB01].FUNC_CTL = IOC_PB01_FUNC_CTL_PWM1_P_1;
    HPM_IOC->PAD[IOC_PAD_PB02].FUNC_CTL = IOC_PB02_FUNC_CTL_PWM1_P_2;
    HPM_IOC->PAD[IOC_PAD_PB03].FUNC_CTL = IOC_PB03_FUNC_CTL_PWM1_P_3;

    uint8_t cmp_index = 2;
    // uint32_t duty, duty_step;
    //

    pwm_stop_counter(HPM_PWM1);
    pwm_disable_hrpwm(HPM_PWM1);
    reset_pwm_counter();
    // pwm_get_default_pwm_config(HPM_PWM1, &pwm_config);

    pwm_cal_hrpwm_chn_start(HPM_PWM1, cmp_index);
    pwm_cal_hrpwm_chn_start(HPM_PWM1, cmp_index + 1);
    pwm_cal_hrpwm_chn_start(HPM_PWM1, cmp_index + 2);
    pwm_cal_hrpwm_chn_start(HPM_PWM1, cmp_index + 3);
    pwm_cal_hrpwm_chn_wait(HPM_PWM1, cmp_index);
    pwm_cal_hrpwm_chn_wait(HPM_PWM1, cmp_index + 1);
    pwm_cal_hrpwm_chn_wait(HPM_PWM1, cmp_index + 2);
    pwm_cal_hrpwm_chn_wait(HPM_PWM1, cmp_index + 3);

    // pwm_enable_hrpwm(HPM_PWM1);
    /*
     * reload and start counter
     */
    pwm_set_cnt_shadow_trig_reload(HPM_PWM1, true);
    pwm_set_hrpwm_reload(HPM_PWM1, 0, reload);

    intc_m_enable_irq_with_priority(IRQn_PWM1, 6);
    pwm_enable_irq(HPM_PWM1, PWM_IRQ_HALF_RELOAD | PWM_IRQ_RELOAD);

    pwm_set_start_count(HPM_PWM1, 0, 0);

    /*
     * config cmp = RELOAD + 1
     */
    pwm_cmp_config_t cmp_config[5] = {0};
    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = 10; /// 4 - 15;
    cmp_config[0].enable_hrcmp = true;
    cmp_config[0].hrcmp = 0;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[1].mode = pwm_cmp_mode_output_compare;
    cmp_config[1].cmp = reload / 2; // reload+2;//4 + 15;
    cmp_config[1].enable_hrcmp = true;
    cmp_config[1].hrcmp = 0;
    cmp_config[1].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[2].mode = pwm_cmp_mode_output_compare;
    cmp_config[2].cmp = 10; // reload+1;///4;
    cmp_config[2].enable_hrcmp = true;
    cmp_config[2].hrcmp = 0;
    cmp_config[2].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[3].mode = pwm_cmp_mode_output_compare;
    cmp_config[3].cmp = reload / 2; //*3/4;
    cmp_config[3].enable_hrcmp = true;
    cmp_config[3].hrcmp = 0;
    cmp_config[3].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[4].mode = pwm_cmp_mode_output_compare;
    cmp_config[4].cmp = reload; //*3/4;
    cmp_config[4].enable_hrcmp = true;
    cmp_config[4].hrcmp = 0;
    cmp_config[4].update_trigger = pwm_shadow_register_update_on_hw_event;

    // pwm_pair_config_t pwm_pair_config;
    // pwm_get_default_pwm_pair_config(HPM_PWM1, &pwm_pair_config);
    // pwm_pair_config.pwm[0].enable_output = false; //enable later
    // pwm_pair_config.pwm[0].dead_zone_in_half_cycle = 1;
    // pwm_pair_config.pwm[0].invert_output = true;

    // pwm_pair_config.pwm[1].enable_output = false; //enable later
    // pwm_pair_config.pwm[1].dead_zone_in_half_cycle = 1;
    // pwm_pair_config.pwm[1].invert_output = false;

    

    pwm_output_channel_t ch_config_a;
    ch_config_a.cmp_start_index = cmp_index + 0;
    ch_config_a.cmp_end_index = cmp_index + 1;
    ch_config_a.invert_output = true;

    pwm_output_channel_t ch_config_b;
    ch_config_b.cmp_start_index = cmp_index + 2;
    ch_config_b.cmp_end_index = cmp_index + 3;
    ch_config_b.invert_output = true;

    pwm_config_output_channel(HPM_PWM1, 0, &ch_config_a);
    pwm_config_output_channel(HPM_PWM1, 1, &ch_config_a);
    pwm_config_output_channel(HPM_PWM1, 2, &ch_config_b);
    pwm_config_output_channel(HPM_PWM1, 3, &ch_config_b);

    pwm_issue_shadow_register_lock_event(HPM_PWM1);

    pwm_config_t pwm_config = {0};
    pwm_config.hrpwm_update_mode = true;
    pwm_config.enable_output = true;
    pwm_config.dead_zone_in_half_cycle = 2;
    pwm_config.invert_output = false;
    
    // pwm_config_cmp(HPM_PWM1, cmp_index + 0, &cmp_config[0]);
    // pwm_config_cmp(HPM_PWM1, cmp_index + 1, &cmp_config[1]);
    // pwm_config_cmp(HPM_PWM1, cmp_index + 2, &cmp_config[2]);
    // pwm_config_cmp(HPM_PWM1, cmp_index + 3, &cmp_config[3]);


    

    pwm_config_cmp(HPM_PWM1, cmp_index + 0, &cmp_config[0]);
    pwm_config_cmp(HPM_PWM1, cmp_index + 1, &cmp_config[1]);
    pwm_config_cmp(HPM_PWM1, cmp_index + 2, &cmp_config[2]);
    pwm_config_cmp(HPM_PWM1, cmp_index + 3, &cmp_config[3]);

    /*
     * config pwm as output driven by cmp
     */
    // if (status_success != pwm_setup_waveform_in_pair(HPM_PWM1, 0, &pwm_pair_config, cmp_index, &cmp_config[0], 2)) {
    // printf("failed to setup waveform\n");
    // while (1) {
    // };
    // }
    // cmp_config[0].cmp = reload >> 1;
    /*
     * config pwm as reference
     */
    // if (status_success != pwm_setup_waveform_in_pair(HPM_PWM1, 2, &pwm_pair_config, cmp_index + 2, &cmp_config[2], 2)) {
    // printf("failed to setup waveform\n");
    // while (1) {
    // };
    // }

    pwm_load_cmp_shadow_on_match(HPM_PWM1, cmp_index+4, &cmp_config[4]);

    pwm_config_pwm(HPM_PWM1, 0, &pwm_config, true);
    pwm_config_pwm(HPM_PWM1, 1, &pwm_config, true);
    pwm_config_pwm(HPM_PWM1, 2, &pwm_config, true);
    pwm_config_pwm(HPM_PWM1, 3, &pwm_config, true);

    pwm_enable_hrpwm(HPM_PWM1);
    pwm_start_counter(HPM_PWM1);
    

    // duty = 200;
    // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index, reload/4, HRPWM_SET_IN_PWM_CLK);
    // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 1, reload*3/4, 0);

    // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 2, reload/4+200, HRPWM_SET_IN_PWM_CLK);
    // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 3, reload*3/4+200, 0);

    // duty_step = reload / TEST_LOOP;
    // duty = reload / TEST_LOOP;
    // for (uint32_t i = 0; i < TEST_LOOP; i++) {
    //     if ((duty + duty_step) >= reload) {
    //         duty = duty_step;
    //     } else {
    //         duty += duty_step;
    //     }
    //     pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index, reload - duty, HRPWM_SET_IN_PWM_CLK);
    //     pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 1, reload - duty, 0);
    //     board_delay_ms(100);
    // }
}
// __IO uint8_t pwm_updated_half = 0;
// __IO uint8_t pwm_updated2 = 0;

__IO bool pwm_output_enable_status = false;
__IO bool pwm_output_enabled_request = true;
__IO uint32_t pwm_half_counter = 0;
void isr_pwm(void)
{
    uint32_t status = pwm_get_status(HPM_PWM1);
    pwm_clear_status(HPM_PWM1, status);
    // if(status & PWM_IRQ_CMP(0)){
    if (status & PWM_IRQ_HALF_RELOAD)
    {
        //board_set_red_led(true);

        if (pwm_output_enabled_request != pwm_output_enable_status)
        {

            // pwm_enable_hrpwm(HPM_PWM1);
            // if (pwm_output_enabled_request)
            // {
            //     HPM_PWM1->PWMCFG[0] = HPM_PWM1->PWMCFG[0] | PWM_PWMCFG_OEN_SET(true);
            //     HPM_PWM1->PWMCFG[1] = HPM_PWM1->PWMCFG[1] | PWM_PWMCFG_OEN_SET(true);
            //     HPM_PWM1->PWMCFG[2] = HPM_PWM1->PWMCFG[2] | PWM_PWMCFG_OEN_SET(true);
            //     HPM_PWM1->PWMCFG[3] = HPM_PWM1->PWMCFG[3] | PWM_PWMCFG_OEN_SET(true);
            // }
            // else
            // {
            //     HPM_PWM1->PWMCFG[0] = HPM_PWM1->PWMCFG[0] & (~PWM_PWMCFG_OEN_SET(true));
            //     HPM_PWM1->PWMCFG[1] = HPM_PWM1->PWMCFG[1] & (~PWM_PWMCFG_OEN_SET(true));
            //     HPM_PWM1->PWMCFG[2] = HPM_PWM1->PWMCFG[2] & (~PWM_PWMCFG_OEN_SET(true));
            //     HPM_PWM1->PWMCFG[3] = HPM_PWM1->PWMCFG[3] & (~PWM_PWMCFG_OEN_SET(true));
            // }

            pwm_output_enable_status = pwm_output_enabled_request;
        }
        pwm_half_counter++;

        // uint8_t cmp_index = 0;
        if (pwm_half_counter == 10)
        {
            pwm_output_enabled_request = false;
        //     pwm_updated_half = 0;
        //     // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 2, reload/4+200, HRPWM_SET_IN_PWM_CLK);
        //     // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 3, reload/2+400, 0);
        }
        // else
        // {
        //     pwm_updated_half = 1;
        //     // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 2, reload/4+200, HRPWM_SET_IN_PWM_CLK);
        //     // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 3, reload/2+200, 0);
        // }
    }
    else if (status & PWM_IRQ_RELOAD)
    {
        //board_set_red_led(false);
        // pwm_full_counter++;
        // if(pwm_counter==2){
        //  HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL = IOC_PB00_FUNC_CTL_PWM1_P_0;
        //  HPM_IOC->PAD[IOC_PAD_PB01].FUNC_CTL = IOC_PB01_FUNC_CTL_PWM1_P_1;
        //  HPM_IOC->PAD[IOC_PAD_PB02].FUNC_CTL = IOC_PB02_FUNC_CTL_PWM1_P_2;
        //  HPM_IOC->PAD[IOC_PAD_PB03].FUNC_CTL = IOC_PB03_FUNC_CTL_PWM1_P_3;

        // HPM_PWM1->PWMCFG[0] = HPM_PWM1->PWMCFG[0] | PWM_PWMCFG_OEN_SET(true);
        // HPM_PWM1->PWMCFG[1] = HPM_PWM1->PWMCFG[1] | PWM_PWMCFG_OEN_SET(true);
        // HPM_PWM1->PWMCFG[2] = HPM_PWM1->PWMCFG[2] | PWM_PWMCFG_OEN_SET(true);
        // HPM_PWM1->PWMCFG[3] = HPM_PWM1->PWMCFG[3] | PWM_PWMCFG_OEN_SET(true);

        //}
        // else{
        //     uint8_t cmp_index = 0;
        //     // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index, reload/4, HRPWM_SET_IN_PWM_CLK);
        //     // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 1, reload*3/4, 0);
        //     if(pwm_counter%2==0){
        //         pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 2, reload/4+200, HRPWM_SET_IN_PWM_CLK);
        //         pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 3, reload*3/4+200, 0);
        //     }

        // }

        // uint8_t cmp_index = 0;
        // if(pwm_updated2>0){
        //     pwm_updated2 = 0;
        //     //pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 2, 400, HRPWM_SET_IN_PWM_CLK);
        //     // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 3, reload/2+400, 0);
        // }
        // else{
        //     pwm_updated2 = 1;
        //     //pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 2, 200, HRPWM_SET_IN_PWM_CLK);
        //     // pwm_update_raw_hrcmp_edge_aligned(HPM_PWM1, cmp_index + 3, reload/2+200, 0);
        // }
    }
}
SDK_DECLARE_EXT_ISR_M(IRQn_PWM1, isr_pwm);

void board_init_pmp(void)
{
    uint32_t start_addr;
    uint32_t end_addr;
    uint32_t length;
    pmp_entry_t pmp_entry[16];
    uint8_t index = 0;

    /* Init noncachable memory */
    extern uint32_t __noncacheable_start__[];
    extern uint32_t __noncacheable_end__[];
    start_addr = (uint32_t)__noncacheable_start__;
    end_addr = (uint32_t)__noncacheable_end__;
    length = end_addr - start_addr;
    if (length > 0)
    {
        /* Ensure the address and the length are power of 2 aligned */
        assert((length & (length - 1U)) == 0U);
        assert((start_addr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    /* Init share memory */
    extern uint32_t __share_mem_start__[];
    extern uint32_t __share_mem_end__[];
    start_addr = (uint32_t)__share_mem_start__;
    end_addr = (uint32_t)__share_mem_end__;
    length = end_addr - start_addr;
    if (length > 0)
    {
        /* Ensure the address and the length are power of 2 aligned */
        assert((length & (length - 1U)) == 0U);
        assert((start_addr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    pmp_config(&pmp_entry[0], index);
}


// https://gist.github.com/elevendroids/d5c3a6cef840f55f7258

/*
    Copyright 2001, 2002 Georges Menie (www.menie.org)
    stdarg version contributed by Christian Ettinger

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*
    putchar is the only external dependency for this file,
    if you have a working putchar, leave it commented out.
    If not, uncomment the define below and
    replace outbyte(c) by your own function call.

#define putchar(c) outbyte(c)
*/

#include <stdarg.h>

// void Print_Buffer(uint8_t *buffer, uint16_t length)
// {
// 	for (uint16_t i = 0; i < length; i++)
// 	{
// 		USART_SendData(USART1, buffer[i]);
// 		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
// 		{
// 		};
// 	}
// }

static int __io_putchar(const char c)
{
    // USART_SendData(USART1, c);
    // while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    // {
    // };
    uart_send_byte(HPM_UART0, c);
    return 1;
    // return (write(1, &c, 1));
}

static void __printchar(char **str, int c)
{
    // extern int putchar(int c);

    if (str)
    {
        **str = c;
        ++(*str);
    }
    else
        __io_putchar(c); //(void)putchar(c);
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int __prints(char **out, const char *string, int width, int pad)
{
    register int pc = 0, padchar = ' ';

    if (width > 0)
    {
        register int len = 0;
        register const char *ptr;
        for (ptr = string; *ptr; ++ptr)
            ++len;
        if (len >= width)
            width = 0;
        else
            width -= len;
        if (pad & PAD_ZERO)
            padchar = '0';
    }
    if (!(pad & PAD_RIGHT))
    {
        for (; width > 0; --width)
        {
            __printchar(out, padchar);
            ++pc;
        }
    }
    for (; *string; ++string)
    {
        __printchar(out, *string);
        ++pc;
    }
    for (; width > 0; --width)
    {
        __printchar(out, padchar);
        ++pc;
    }

    return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int __printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
    char print_buf[PRINT_BUF_LEN];
    register char *s;
    register int t, neg = 0, pc = 0;
    register unsigned int u = i;

    if (i == 0)
    {
        print_buf[0] = '0';
        print_buf[1] = '\0';
        return __prints(out, print_buf, width, pad);
    }

    if (sg && b == 10 && i < 0)
    {
        neg = 1;
        u = -i;
    }

    s = print_buf + PRINT_BUF_LEN - 1;
    *s = '\0';

    while (u)
    {
        t = u % b;
        if (t >= 10)
            t += letbase - '0' - 10;
        *--s = t + '0';
        u /= b;
    }

    if (neg)
    {
        if (width && (pad & PAD_ZERO))
        {
            __printchar(out, '-');
            ++pc;
            --width;
        }
        else
        {
            *--s = '-';
        }
    }

    return pc + __prints(out, s, width, pad);
}

static int __print(char **out, const char *format, va_list args)
{
    register int width, pad;
    register int pc = 0;
    char scr[2];

    for (; *format != 0; ++format)
    {
        if (*format == '%')
        {
            ++format;
            width = pad = 0;
            if (*format == '\0')
                break;
            if (*format == '%')
                goto out;
            if (*format == '-')
            {
                ++format;
                pad = PAD_RIGHT;
            }
            while (*format == '0')
            {
                ++format;
                pad |= PAD_ZERO;
            }
            for (; *format >= '0' && *format <= '9'; ++format)
            {
                width *= 10;
                width += *format - '0';
            }
            if (*format == 's')
            {
                register char *s = (char *)va_arg(args, int);
                pc += __prints(out, s ? s : "(null)", width, pad);
                continue;
            }
            if (*format == 'd')
            {
                pc += __printi(out, va_arg(args, int), 10, 1, width, pad, 'a');
                continue;
            }
            if (*format == 'x')
            {
                pc += __printi(out, va_arg(args, int), 16, 0, width, pad, 'a');
                continue;
            }
            if (*format == 'X')
            {
                pc += __printi(out, va_arg(args, int), 16, 0, width, pad, 'A');
                continue;
            }
            if (*format == 'u')
            {
                pc += __printi(out, va_arg(args, int), 10, 0, width, pad, 'a');
                continue;
            }
            if (*format == 'c')
            {
                /* char are converted to int then pushed on the stack */
                scr[0] = (char)va_arg(args, int);
                scr[1] = '\0';
                pc += __prints(out, scr, width, pad);
                continue;
            }
        }
        else
        {
        out:
            __printchar(out, *format);
            ++pc;
        }
    }
    if (out)
        **out = '\0';
    va_end(args);
    return pc;
}

int Logging(logging_level_t level, const char *format, ...)
{
    if (level >= LOOGING_LEVEL)
    {
        va_list args;
        va_start(args, format);
        return __print(0, format, args);
    }
    return -1;
}