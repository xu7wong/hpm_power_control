#include "board.h"
#include "adc_trigger.h"
uint32_t debug = 0;

int main(void)
{

    board_init_clock();
    board_init_console();
    board_init_pmp();
    board_init_led();
    board_init_hrpwm();
    
    power_control_init();
    // board_init_adc();

    // board_init_timer();
    //board_delay_ms(1000);

    while (1)
    {
        // board_set_red_led(1);
        board_set_green_led(0);
        //board_delay_ms(150);
        // board_set_red_led(0);
        board_set_green_led(1);
        //board_delay_ms(150);
        if (adc_get_done(0) && adc_get_done(1))
        {
            adc_clear_done(0);
            adc_clear_done(1);
            Logging(LOG_DEBUG, "adc dual buff full up!\n");
        }
        else{
            if (adc_get_done(0))
            {
            adc_clear_done(0);
            Logging(LOG_DEBUG, "adc 0 done!\n");
            }
            else if (adc_get_done(1))
            {
            adc_clear_done(1);
            Logging(LOG_DEBUG, "adc 1 done!\n");
            }
        }
        // Logging(LOG_DEBUG, "hello world -> %d\n", debug++);
        // sequence_handler();
        // dma_handler();
        // preemption_handler();
        // console_send_byte('a');
        // uart_send_byte(HPM_UART0, 'a');
    }
    return 0;
}
