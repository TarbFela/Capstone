#include "adpc_app.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <string.h>
#include "tusb.h"

#include "mcp3x6xR_driver/mcp3x6xR.h"
#include "mcp3x6xR_driver/mcp_pio.h"
#include "ada4255_driver/ada4255.h"

#include "../ADPC_Dev/ADPC_cfg.h"
#include "../ADPC_Dev/ADPC_ADC.h"
#include "../ADPC_Dev/adpc_gpio_pwm.h"

#include "adpc_app_funcs.h"

#include "adpc_core1.h"


volatile app_state_t ui_state = {
        .ui_cursor = 0,
        .level = 0,
        .initialized = false,
        .is_streaming = false,
        .pwm_enabled = false,
        .ph_enabled = false,
        .cl_ictl = false,
        .ui = {1,0},
};


app_result_t app_dispatch(app_state_t *s) {
    static bool inited = false;
    char *argv[MAX_UI_ARGS];
    char *ui = s->ui;

    if (s->ui[0] == 'q') return APP_REBOOT;
    if(strncmp(ui, "init",4) == 0) {
        if(inited) {
            if(!s->is_streaming) printf("Already initialized. Restart to initialize again!\n");
            return APP_OK;
        }
        inited = true;
        return adpc_init();
    }
    if(strncmp(ui, "level",5) == 0) {
        if(ui[5] == 0xD) {
            if(!s->is_streaming) printf("Level is %.1f\n",s->level);
            return APP_OK;
        }
        float level = atof(ui + 6);
        if(ictl_level_bounds_check(level)) {
            if(!s->is_streaming) printf("Level is out of bounds!\n");
            return APP_INVALID_ARG;
        }
        s->level = level;
        mphb_set_dlevel_all(level * MPHB_PWM_WRAP);
        if(!s->is_streaming) printf("[set level to %.4f%%]\n",100*level);
        return APP_OK;
    }
    if(strncmp(s->ui, "isp ",4) == 0) {
        app_cmd_isp(s,  atof(s->ui + 4));
    }
    if(strncmp(ui,"phen",4) == 0) {
        mphb_set_ph_en_all(true);
        printf("[set ph_en ENABLED]\n");
    }
    if(strncmp(ui,"phd",3) == 0) {
        mphb_set_ph_en_all(false);
        printf("[set ph_en DISABLED]\n");
    }
    if(strncmp(ui,"pwmen",5) == 0) {
        mphb_set_pwm_en_all(true);
        printf("[set pwm_en ENABLED]\n");
    }
    if(strncmp(ui,"pwmd",4) == 0) {
        mphb_set_levels_all(0,0);
        printf("[set pwm to 0]\n");
    }
    if(strncmp(ui,"start",5) == 0) {
        if(adpc_init() != APP_OK) return APP_ERROR;
        s->initialized = true;
        mphb_set_pwm_en_all(true);
        printf("[set pwm_en ENABLED]\n");
        mphb_set_dlevel_all(0);
        mphb_set_ph_en_all(true);
        printf("[set ph_en ENABLED]\n");
        return APP_OK;

    }
    if(strncmp(ui,"rstream",7) == 0) {
        return app_cmd_rstream(s);
    }
    if(strncmp(ui,"r0",2) == 0) {
        uint n = (uint)atoi(ui + 3);
        if(n > DMA_BUFF_SIZE) return APP_INVALID_ARG;
        if(n==0) n=1;
        int32_t *buff = (int32_t *)mpio_0.buff + (dma0_last_written - 1) * DMA_BUFF_SIZE;
        printf("DMA last written %d values:\n",n);
        for(int i = 0; i<n; i++) {
            printf("\t%ld\n",buff[i]);
        }
        return APP_OK;
    }
    if(strncmp(ui,"r1",2) == 0) {
        uint n = (uint)atoi(ui + 3);
        if(n > DMA_BUFF_SIZE) return APP_INVALID_ARG;
        if(n==0) n = 2;
        printf("DMA last written %d values:\n",n);
        uint32_t *buff = mpio_1.buff + (dma1_last_written - 1) * DMA_BUFF_SIZE;
        for(int i = 0; i<n; i++) {
//            printf("\t ");
//            for(int ii = 0; i<; i++) printf("%02X ",buff[i]>>(32-8*i));
            // channel extract and sign-extend
            printf("\t[%ld] ", buff[i]>>28);
            sign_extend_24_to_32(buff[i]);
            printf("%ld\n", buff[i]);
        }
        return APP_OK;
    }
    if(strncmp(ui,"ictl",4) == 0) {
        app_cmd_ictl(s);
    }
    if(strncmp(ui,"coeff i ",8)==0) {
        float val = atof(ui+8);
        ictlInfo.i_coeff = val;
        printf("[Set ictl I coeff to %.5f]\n",ictlInfo.i_coeff);
    }
    if(strncmp(ui,"coeff p ",8)==0) {
        float val = atof(ui+8);
        ictlInfo.p_coeff = val;
        printf("[Set ictl P coeff to %.5f]\n",ictlInfo.p_coeff);
    }
    if(strncmp(ui,"iprog",5) == 0) {
        // TODO: DANGEROUS. Length of UI can exceed iprog_ui's size.
        printf("\t+--------------------+\n"
               "\t| Current Programmer |\n"
               "\t+--------------------+\n"
               "This tool allows sequences of current setpoints\n"
               "to be iterated through. Please provide a time-step\n"
               "in milliseconds and then a series of setpoints in\n"
               "amps (decimal values are ok). Terminate the new\n"
               "current program with \"END\".\n"
               "\n\n"
               "A maximum of %d setpoints may be set. Invalid data\n"
               "entry will be recorded as zeroes.\n", ICTL_PROG_MAX_SIZE);
        char iprog_ui[64];
        float timestep = 0;
        while(timestep == 0) {
            printf("\n\tTimestep (ms): ");
            scanf(" %s",iprog_ui);
            timestep = atof(iprog_ui);
            if(timestep == 0) printf("Invalid Input.\n");
        }
        s->current_program.timestep = timestep;
        printf("\nTimestep set to %.1f ms.\n",timestep);
        s->current_program.N = 0;
        while(1) {
            printf("\nEnter setpoint #%d: ",s->current_program.N+1);
            scanf("%s",iprog_ui);
            if(strncmp(iprog_ui,"END",3)==0) break;
            float setpoint = atof(iprog_ui);
            s->current_program.setpoints[s->current_program.N] = setpoint;
            s->current_program.N++;
            if(s->current_program.N == ICTL_PROG_MAX_SIZE) break;
        }
        printf("Setpoint programming complete. Program:\n");
        for(int i = 0; i<s->current_program.N; i++) printf("[%d]\t%.2f\n",i,s->current_program.setpoints[i]);
        printf("Programed stored. Use \"irun\" to run this program.\n");
        return APP_OK;
    }
    if(strncmp(ui,"irun",4) == 0) {
        printf("\t+--------------------+\n"
               "\t|   Program Runner   |\n"
               "\t+--------------------+\n"
               "Use argument \"stream\" to stream ADC data\n"
               "during the execution of the current program.\n");
        if(s->current_program.N == 0) {
            printf("No program loaded. \nPlease use \"iprog\" to load a program.\n");
            return APP_OK;
        }
        printf("Press any key to cancel the program.\n");
        printf("Starting the program...\n");
        if(strncmp(ui+5,"stream",6) == 0) {
            return app_cmd_irun_streaming(s);
        }
        return app_cmd_irun(s);

    }
    if(strncmp(ui,"sdith",5) == 0) {
        float val = atof(ui+5);
        mphb_set_dlevel_all_spatial_dithering(val);
        return APP_OK;
    }

    printf("%s\n",ui);
    return APP_OK;
}

app_result_t app_dispatch_single_char(app_state_t *s, char ui) {
//    if(ui == 'e') {
//        mphb_set_ph_en(HB1B, true);
//        mphb_set_ph_en(HB2B, true);
//        mphb_set_ph_en(HB3B, true);
//        return APP_OK;
////                            printf("ENABLE PIN ON\n");
//    }
//    if(ui == 'd') {
//        mphb_set_ph_en(HB1B, false);
//        mphb_set_ph_en(HB2B, false);
//        mphb_set_ph_en(HB3B, false);
//        return APP_OK;
////                            printf("ENABLE PIN OFF\n");
//    }
//    else if ((ui >= '0') && (ui <= '9')) {
//        s->level = (ui-'0') * ;
////                            printf("%d offset\n",level);
//        mphb_set_dlevel_all( s->level * MPHB_PWM_WRAP);
//        return APP_OK;
//    }
//    else if (ui == 'p' || ui == 'l') {
//        s->level += (ui == 'p') ? 1 : -1;
////                            printf("%d offset\n",level);
//        mphb_set_dlevel_all( s->level);
//        return APP_OK;
//    }
    return APP_STOP_STREAM;
}


// Get user input strings that are newline-terminated
// Has a built-in getchar() timeout so it shares time.
app_result_t app_shell_task(app_state_t *s) {
    //if(s->is_streaming) return APP_ERROR;
    // check if we're on a new line; dispatch should reset the cursor!
    if(s->ui_cursor == 0 && s->ui[0] != 0) {
        if(!s->is_streaming) printf(">> ");
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
    if(!s->is_streaming) putchar((char)uic);

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

