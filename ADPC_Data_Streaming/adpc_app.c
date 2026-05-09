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
    static bool inited = false;
    char *argv[MAX_UI_ARGS];
    char *ui = s->ui;

    if (s->ui[0] == 'q') return APP_REBOOT;
    if(strncmp(s->ui, "init",4) == 0) {
        if(inited) {
            if(!s->is_streaming) printf("Already initialized. Restart to initialize again!\n");
            return APP_OK;
        }
        inited = true;
        return adpc_init();
    }
    if(strncmp(s->ui, "level  ",6) == 0) {
        int level = atoi(s->ui + 6);
        mphb_set_dlevel_all(level);
        if(!s->is_streaming) printf("[set level to %d]\n",level);
    }
    if(strncmp(ui,"phen",4) == 0) {
        mphb_set_ph_en(HB1B, true);
        mphb_set_ph_en(HB2B, true);
        printf("[set ph_en ENABLED]\n");
    }
    if(strncmp(ui,"phd",3) == 0) {
        mphb_set_ph_en(HB1B, false);
        mphb_set_ph_en(HB2B, false);
        printf("[set ph_en DISABLED]\n");
    }
    if(strncmp(ui,"pwmen",5) == 0) {
        mphb_set_pwm_en(HB1B, true);
        mphb_set_pwm_en(HB2B, true);
        printf("[set pwm_en ENABLED]\n");
    }
    if(strncmp(ui,"pwmd",4) == 0) {
        mphb_set_levels_all(0,0);
        printf("[set pwm to 0]\n");
    }
    if(strncmp(ui,"rstream",7) == 0) {
        return app_cmd_rstream(s);
    }
    if(strncmp(ui,"r0 ",3) == 0) {
        uint n = (uint)atoi(ui + 3);
        if(n==0) return APP_INVALID_ARG;
        printf("DMA last written %n values:\n");
        int32_t *buff = (int32_t *)mpio_0.buff + (dma0_last_written - 1) * DMA_BUFF_SIZE;
        for(int i = 0; i<n; i++) {
            printf("\t%ld\n",buff[i]);
        }
        return APP_OK;
    }
    if(strncmp(ui,"r1 ",3) == 0) {
        uint n = (uint)atoi(ui + 3);
        if(n==0) return APP_INVALID_ARG;
        printf("DMA last written %n values:\n");
        uint32_t *buff = mpio_1.buff + (dma1_last_written - 1) * DMA_BUFF_SIZE;
        for(int i = 0; i<n; i++) {
//            printf("\t ");
//            for(int ii = 0; i<; i++) printf("%02X ",buff[i]>>(32-8*i));
            printf("\t[%d] %ld\n", buff[i]>>28,buff[i]&0xFFFFFF);
        }
        return APP_OK;
    }

    printf("%s\n",ui);
    return APP_OK;
}

app_result_t app_dispatch_single_char(app_state_t *s, char ui) {
    if(ui == 'e') {
        mphb_set_ph_en(HB1B, true);
        mphb_set_ph_en(HB2B, true);
        return APP_OK;
//                            printf("ENABLE PIN ON\n");
    }
    if(ui == 'd') {
        mphb_set_ph_en(HB1B, false);
        mphb_set_ph_en(HB2B, false);
        return APP_OK;
//                            printf("ENABLE PIN OFF\n");
    }
    else if ((ui >= '0') && (ui <= '9')) {
        s->level = (ui-'0')*5;
//                            printf("%d offset\n",level);
        mphb_set_dlevel_all( s->level);
        return APP_OK;
    }
    else if (ui == 'p' || ui == 'l') {
        s->level += (ui == 'p') ? 1 : -1;
//                            printf("%d offset\n",level);
        mphb_set_dlevel_all( s->level);
        return APP_OK;
    }
    return APP_STOP_STREAM;
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
        return APP_RUNNING;
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

