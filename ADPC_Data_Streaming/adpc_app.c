#include "adpc_app.h"
#include "pico/stdlib.h"
#include <string.h>
#include "tusb.h"

#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "mcp3x6xR_driver/mcp_pio.h"
#include "ada4255_driver/ada4255.h"

#include "../ADPC_Dev/ADPC_cfg.h"
#include "../ADPC_Dev/ADPC_ADC.h"
#include "../ADPC_Dev/adpc_gpio_pwm.h"

#include "adpc_app_funcs.h"


volatile app_state_t ui_state = {
        .level = 0,
        .ui_cursor = 0,
        .is_streaming = false,
        .pwm_enabled = false,
        .ph_enabled = false,
        .ui = {1,0}
};


app_result_t app_dispatch(app_state_t *s) {
    char *argv[MAX_UI_ARGS];
    printf("%s\n",s->ui);
    char *ui = s->ui;

    if (s->ui[0] == 'q') return APP_REBOOT;
    if(strncmp(s->ui, "init",4) == 0) {return adpc_init();}

    return APP_OK;
}


// Get user input strings that are newline-terminated
// Has a built-in 1ms getchar() timeout so it shares time.
app_result_t app_shell_task(app_state_t *s) {
    if(s->is_streaming) return APP_ERROR;
    // check if we're on a new line; dispatch should reset the cursor!
    if(s->ui_cursor == 0 && s->ui[0] != 0) {
        printf(">> ");
        s->ui[0] = 0;
    }
    int uic = getchar_timeout_us(1000);
    if(uic == PICO_ERROR_TIMEOUT) {
        return APP_RUNNING;
    }
    // check for different
    if((char)uic == UI_BACKSPACE) {
        if(s->ui_cursor == 0) return APP_RUNNING;
        printf("\b \b");
        s->ui_cursor -= 1;
    }
    // echo
    putchar((char)uic);

    // populate ui
    if((char)uic == '\n') {
        s->ui[s->ui_cursor] = 0;
        s->ui_cursor = 0;
        return app_dispatch(s);
    }
    s->ui[s->ui_cursor] = (char)uic;
    if(s->ui_cursor < UI_BUFF_SIZE - 1) s->ui_cursor++;
    else printf("\b");
    return APP_RUNNING;
}

