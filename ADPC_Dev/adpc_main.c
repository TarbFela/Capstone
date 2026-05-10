#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "ADPC_ADC.h"

#include "adpc_core1.h"
#include "adpc_dsp.h"

int main() {
    /*======================================*
     *            INITIALIZATIONS           *
     *======================================*/
    stdio_init_all();
    sleep_ms(1000);

    //wait for input
    char ui[64];
    scanf(" %c",ui);
    printf("[input recieved]\n");
    if(ui[0] == 'q') goto reboot;

    multicore_launch_core1(adpc_core1);
    sleep_ms(1000);
    if(multicore_fifo_pop_blocking() != 0xFACE) {
        printf("Failed to initialize core 1!");
        goto reboot;
    }

    adpc_adc_init(adpc_dsp_dma_handler_1);



    /*======================================*
     *               REBOOTING              *
     *======================================*/
    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
}
