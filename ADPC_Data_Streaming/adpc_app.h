#ifndef ADPC_APP_H
#define ADPC_APP_H

#include "pico/stdlib.h"

#define UI_BACKSPACE 0x7F

#define MAX_UI_ARGS 4



#define ICTL_PROG_MAX_SIZE 128
struct ictl_prog {
    float timestep;
    float setpoints[ICTL_PROG_MAX_SIZE];
    int N;
    int cycles;
};

#define UI_BUFF_SIZE 64
typedef struct {
    char ui[UI_BUFF_SIZE];
    char ui_cursor;
    float level;
    float current_setpoint;
    bool initialized;
    bool is_streaming;
    bool pwm_enabled;
    bool ph_enabled;
    bool cl_ictl;
    struct ictl_prog current_program ;
} app_state_t;


typedef enum {APP_OK, APP_RUNNING, APP_INVALID_ARG, APP_TIMEOUT, APP_ERROR, APP_STOP_STREAM, APP_REBOOT} app_result_t;

extern volatile app_state_t ui_state;

app_result_t app_dispatch(app_state_t *s);
app_result_t app_dispatch_single_char(app_state_t *s, char ui);
app_result_t app_shell_task(app_state_t *s);

#endif