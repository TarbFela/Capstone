#ifndef ADPC_CORE1_H
#define ADPC_CORE1_H

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

#include "adpc_app.h"
#include "adpc_app_funcs.h"

#define MC_FIFO_STOP_FLAG 0xBEEF

void core1_ictl(void);

typedef struct {
    float accum;
    float i_coeff; // PI control
    float p_coeff;
    float accum_low_bound;
    float accum_high_bound;
    float level_low_bound;
    float level_high_bound;
} ictl_info_t;

extern volatile ictl_info_t ictlInfo;

int ictl_level_bounds_check(float val) {
    return (val>ictlInfo.level_high_bound) || (val < ictlInfo.level_low_bound);
}

#endif