#include "board.h"
#include "hpm_adc16_drv.h"
#include "hpm_interrupt.h"
#include "hpm_clock_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_dma_drv.h"
#include "hpm_dmamux_drv.h"

#define APP_ADC_TRIG_FREQ                 (2000U)
#define APP_ADC16_DMA_BUFF_LEN_IN_BYTES   (1024U)
#define APP_ADC16_PMT_PWM_REFCH_A (8U)

#define APP_ADC16_CORE HPM_CORE0
#define APP_ADC16_PMT_PWM HPM_PWM0
#define APP_ADC16_PMT_TRGM HPM_TRGM0
#define APP_ADC16_BASE_TRIGGER HPM_ADC0
#define APP_ADC16_CH_NTC (1)

static uint8_t trig_adc_channel[] = {APP_ADC16_CH_NTC};
ATTR_RAMFUNC_WITH_ALIGNMENT(8) dma_linked_descriptor_t adc_descriptors1[6];

ATTR_RAMFUNC_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t pmt_buff[ADC_SOC_PMT_MAX_DMA_BUFF_LEN_IN_4BYTES];
ATTR_RAMFUNC_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint16_t adc_buff[APP_ADC16_DMA_BUFF_LEN_IN_BYTES * 2];
ATTR_RAMFUNC_WITH_ALIGNMENT(8) uint8_t adc_done[2];
static void init_common_config(void);

static void init_preemption_config(void);

static void init_trigger_mux(TRGM_Type *ptr);
static void init_trigger_source(PWM_Type *ptr, uint32_t sample_freq);
static void init_trigger_target(ADC16_Type *ptr, uint8_t trig_ch, bool inten);
static void hdma_auto_config(void);
static void hdma_dma_chain_config(void);

void power_control_init(void)
{
    init_common_config();
    init_preemption_config();
}

static void init_common_config(void)
{
    adc16_config_t cfg;

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);
    cfg.res = adc16_res_16_bits;
    cfg.conv_mode = adc16_conv_mode_preemption;
    cfg.adc_clk_div = adc16_clock_divider_3;
    cfg.sel_sync_ahb = true;// (clk_adc_src_ahb0 == clock_get_source(BOARD_APP_ADC16_CLK_NAME)) ? true : false;

    if (cfg.conv_mode == adc16_conv_mode_sequence ||
        cfg.conv_mode == adc16_conv_mode_preemption)
    {
        cfg.adc_ahb_en = true;
    }

    /* adc16 initialization */
    adc16_init(APP_ADC16_BASE_TRIGGER, &cfg);
    /* enable irq */
    intc_m_enable_irq_with_priority(IRQn_ADC0, 1);
        
}

static void init_trigger_source(PWM_Type *ptr, uint32_t sample_freq)
{
    pwm_cmp_config_t pwm_cmp_cfg;
    pwm_cmp_config_t pwm_cmp_dma_cfg;
    pwm_output_channel_t pwm_output_ch_cfg;
    uint32_t freq, reload;

    /* TODO: Set PWM Clock Source and divider */
    if (ptr == HPM_PWM0)
    {
        freq = clock_get_frequency(clock_mot0);
    }
    else if (ptr == HPM_PWM1)
    {
        freq = clock_get_frequency(clock_mot1);
    }

    reload = freq / sample_freq - 1;
    Logging(LOG_DEBUG, "clock_mot freq:%d, reload:%d\r\n", freq, reload);

    pwm_set_reload(ptr, 0, reload);

    /* Set a comparator */
    memset(&pwm_cmp_cfg, 0x00, sizeof(pwm_cmp_config_t));
    pwm_cmp_cfg.enable_ex_cmp = false;
    pwm_cmp_cfg.mode = pwm_cmp_mode_output_compare;
    pwm_cmp_cfg.update_trigger = pwm_shadow_register_update_on_shlk;

    /* Select comp8 and trigger at the middle of a pwm cycle */
    pwm_cmp_cfg.cmp = 1;
    pwm_config_cmp(ptr, APP_ADC16_PMT_PWM_REFCH_A, &pwm_cmp_cfg);

    /* Set a comparator */
    memset(&pwm_cmp_dma_cfg, 0x00, sizeof(pwm_cmp_config_t));
    pwm_cmp_dma_cfg.enable_ex_cmp = false;
    pwm_cmp_dma_cfg.mode = pwm_cmp_mode_output_compare;
    pwm_cmp_dma_cfg.update_trigger = pwm_shadow_register_update_on_shlk;

    /* Select comp8 and trigger at the middle of a pwm cycle */
    pwm_cmp_dma_cfg.cmp = reload - 2;
    pwm_config_cmp(ptr, APP_ADC16_PMT_PWM_REFCH_A + 1, &pwm_cmp_dma_cfg);
    pwm_enable_dma_request(ptr, PWM_IRQ_CMP(9));

    /* Issue a shadow lock */
    pwm_issue_shadow_register_lock_event(APP_ADC16_PMT_PWM);

    /* Set comparator channel to generate a trigger signal */
    pwm_output_ch_cfg.cmp_start_index = APP_ADC16_PMT_PWM_REFCH_A; /* start channel */
    pwm_output_ch_cfg.cmp_end_index = APP_ADC16_PMT_PWM_REFCH_A;   /* end channel */
    pwm_output_ch_cfg.invert_output = false;
    pwm_config_output_channel(ptr, APP_ADC16_PMT_PWM_REFCH_A, &pwm_output_ch_cfg);

    /* Start the comparator counter */
    pwm_start_counter(ptr);
}

static void init_trigger_mux(TRGM_Type *ptr)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type = trgm_output_same_as_input;

    trgm_output_cfg.input = HPM_TRGM0_INPUT_SRC_PWM0_CH8REF;
    trgm_output_config(ptr, TRGM_TRGOCFG_ADC0_STRGI, &trgm_output_cfg);

    trgm_dma_request_config(ptr, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM0_CMP9);
}

static void init_trigger_target(ADC16_Type *ptr, uint8_t trig_ch, bool inten)
{
    adc16_pmt_config_t pmt_cfg;

    pmt_cfg.trig_len = sizeof(trig_adc_channel);
    pmt_cfg.trig_ch = trig_ch;

    for (int i = 0; i < pmt_cfg.trig_len; i++)
    {
        pmt_cfg.adc_ch[i] = trig_adc_channel[i];
        pmt_cfg.inten[i] = false;
    }

    pmt_cfg.inten[pmt_cfg.trig_len - 1] = inten;

    adc16_set_pmt_config(ptr, &pmt_cfg);
    adc16_set_pmt_queue_enable(ptr, trig_ch, true);
}
static void hdma_dma_chain_config(void)
{
    dma_channel_config_t dma_ch_config;
    static uint8_t dummy_cmd = 1;

    dma_default_channel_config(HPM_HDMA, &dma_ch_config);

    /* dma trans */
    dma_ch_config.size_in_byte = APP_ADC16_DMA_BUFF_LEN_IN_BYTES * sizeof(uint16_t);
    dma_ch_config.src_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)pmt_buff);
    dma_ch_config.dst_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_buff[0]);
    dma_ch_config.src_width = DMA_TRANSFER_WIDTH_HALF_WORD;
    dma_ch_config.dst_width = DMA_TRANSFER_WIDTH_HALF_WORD;
    dma_ch_config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dma_ch_config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    dma_ch_config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dma_ch_config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_ch_config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    dma_ch_config.linked_ptr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_descriptors1[1]);
    dma_config_linked_descriptor(HPM_HDMA, &adc_descriptors1[0], 0, &dma_ch_config);

    /*set done*/
    dma_ch_config.size_in_byte = 1;
    dma_ch_config.src_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&dummy_cmd);
    dma_ch_config.dst_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_done[0]);
    dma_ch_config.src_width = DMA_TRANSFER_WIDTH_BYTE;
    dma_ch_config.dst_width = DMA_TRANSFER_WIDTH_BYTE;
    dma_ch_config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dma_ch_config.src_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dma_ch_config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dma_ch_config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_ch_config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_ch_config.linked_ptr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_descriptors1[2]);
    dma_config_linked_descriptor(HPM_HDMA, &adc_descriptors1[1], 0, &dma_ch_config);

    /* dma trans */
    dma_ch_config.size_in_byte = APP_ADC16_DMA_BUFF_LEN_IN_BYTES * sizeof(uint16_t);
    dma_ch_config.src_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)pmt_buff);
    dma_ch_config.dst_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_buff[APP_ADC16_DMA_BUFF_LEN_IN_BYTES]);
    dma_ch_config.src_width = DMA_TRANSFER_WIDTH_HALF_WORD;
    dma_ch_config.dst_width = DMA_TRANSFER_WIDTH_HALF_WORD;
    dma_ch_config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dma_ch_config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    dma_ch_config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dma_ch_config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_ch_config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    dma_ch_config.linked_ptr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_descriptors1[3]);
    dma_config_linked_descriptor(HPM_HDMA, &adc_descriptors1[2], 0, &dma_ch_config);

    /*set done*/
    dma_ch_config.size_in_byte = 1;
    dma_ch_config.src_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&dummy_cmd);
    dma_ch_config.dst_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_done[1]);
    dma_ch_config.src_width = DMA_TRANSFER_WIDTH_BYTE;
    dma_ch_config.dst_width = DMA_TRANSFER_WIDTH_BYTE;
    dma_ch_config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dma_ch_config.src_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dma_ch_config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dma_ch_config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_ch_config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_ch_config.linked_ptr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_descriptors1[0]);
    dma_config_linked_descriptor(HPM_HDMA, &adc_descriptors1[3], 0, &dma_ch_config);
}
static void hdma_auto_config(void)
{
    // int i;
    // hpm_stat_t stat;
    dma_channel_config_t ch_config = {0};
    static uint32_t dummy_data1 = 0xff, dummy_data2 = 0xff;

    // gpio_dummy_intterrupt_config();
    hdma_dma_chain_config();

    dma_reset(HPM_HDMA);

    dma_default_channel_config(HPM_HDMA, &ch_config);

    // TX
    ch_config.src_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&dummy_data1);
    ch_config.dst_addr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&dummy_data2);
    ch_config.src_width = DMA_TRANSFER_WIDTH_BYTE;
    ch_config.dst_width = DMA_TRANSFER_WIDTH_BYTE;
    ch_config.size_in_byte = 1;
    ch_config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    ch_config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    ch_config.src_mode = DMA_HANDSHAKE_MODE_NORMAL;
    ch_config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    ch_config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    ch_config.linked_ptr = core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)&adc_descriptors1[0]);

    // warning! fixed addr no support burst!
    if (status_success != dma_setup_channel(HPM_HDMA, 0, &ch_config, true))
    {
        Logging(LOG_ERROR, "dma setup channel failed 0\n");
        return;
    }

    //**********************************
    dmamux_config(HPM_DMAMUX, DMAMUX_MUXCFG_HDMA_MUX0, HPM_DMA_SRC_MOT0_0, true);
}
static void init_preemption_config(void)
{
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 4;

    for (int i = 0; i < sizeof(trig_adc_channel); i++)
    {
        ch_cfg.ch = trig_adc_channel[i];
        adc16_init_channel(APP_ADC16_BASE_TRIGGER, &ch_cfg);
    }

// #ifndef ADC_SOC_PMT_NO_TRIGSOURCE
    /* Trigger source initialization */
    init_trigger_source(APP_ADC16_PMT_PWM, APP_ADC_TRIG_FREQ);

    /* Trigger mux initialization */
    init_trigger_mux(APP_ADC16_PMT_TRGM);
// #endif

    /* Trigger target initialization */
    init_trigger_target(APP_ADC16_BASE_TRIGGER, ADC16_CONFIG_TRG0A, true);

    /* Set DMA start address for preemption mode */
    adc16_init_pmt_dma(APP_ADC16_BASE_TRIGGER, core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)pmt_buff));

    /* Enable trigger complete interrupt */
    // adc16_enable_interrupts(BOARD_APP_ADC16_BASE, APP_ADC16_PMT_IRQ_EVENT);

    // hdma_transfer_start();
    hdma_auto_config();
}

void adc_clear_done(uint8_t index)
{
    adc_done[index] = 0;
}

uint8_t adc_get_done(uint8_t index)
{
    return adc_done[index];
}

uint8_t *adc_get_buf_addr_point(uint8_t index)
{
    return (index == 0) ? (uint8_t *)&adc_buff[0] : (uint8_t *)&adc_buff[APP_ADC16_DMA_BUFF_LEN_IN_BYTES];
}
