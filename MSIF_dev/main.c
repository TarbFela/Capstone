#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "MSIF_cfg.h"

int main(void) {
                        /*** INITS ***/
    stdio_init_all();
    sleep_ms(1000);

    printf("Hello Capstone World!\n");


    while(1) {
        int uii = stdio_getchar_timeout_us(100);
        if(uii != PICO_ERROR_TIMEOUT) {
            char ui = (char)uii;
            // QUIT
            if (ui == 'q') break;
            else printf("Input recieved! [press q for BOOTSEL mode]\n");
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}