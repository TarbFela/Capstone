#include "adpc_gpio_pwm.h"
#include "ADPC_cfg.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"


mphb2_gpio_pwm_t hb_1B = {
        .pwm_a_pin = PWM_B_1_PIN,
        .pwm_c_pin = PWM_D_1_PIN,
        .ph_en_pin = PH_EN_B1_PIN,
        .initialized = false,
};
mphb2_gpio_pwm_t hb_2B = {
        .pwm_a_pin = PWM_B_2_PIN,
        .pwm_c_pin = PWM_D_2_PIN,
        .ph_en_pin = PH_EN_B2_PIN,
        .initialized = false,
};
mphb2_gpio_pwm_t hb_3B = {
        .pwm_a_pin = PWM_B_3_PIN,
        .pwm_c_pin = PWM_D_3_PIN,
        .ph_en_pin = PH_EN_B3_PIN,
        .initialized = false,
};
mphb2_gpio_pwm_t hb_1A = {
        .pwm_a_pin = PWM_A_1_PIN,
        .pwm_c_pin = PWM_C_1_PIN,
        .ph_en_pin = PH_EN_A1_PIN,
        .initialized = false,
};
mphb2_gpio_pwm_t hb_2A = {
        .pwm_a_pin = PWM_A_2_PIN,
        .pwm_c_pin = PWM_C_2_PIN,
        .ph_en_pin = PH_EN_A2_PIN,
        .initialized = false,
};
mphb2_gpio_pwm_t hb_3A = {
        .pwm_a_pin = PWM_A_3_PIN,
        .pwm_c_pin = PWM_C_3_PIN,
        .ph_en_pin = PH_EN_A3_PIN,
        .initialized = false,
};

mphb2_gpio_pwm_t *mphb2_arr[6] = {
        &hb_1A,
        &hb_2A,
        &hb_3A,
        &hb_1B,
        &hb_2B,
        &hb_3B
        };

volatile uint32_t mphb_pwm_cm_level = 500;

void mphb_gpio_init(mphb_port_t i) {
    int pwm_ab_pin = mphb2_arr[i]->pwm_a_pin;
    int pwm_cd_pin = mphb2_arr[i]->pwm_c_pin;
    int ph_en_pin = mphb2_arr[i]->ph_en_pin;

    gpio_init(pwm_ab_pin);
    gpio_init(pwm_cd_pin);
    gpio_set_function(pwm_ab_pin,GPIO_FUNC_PWM);
    gpio_set_slew_rate(pwm_ab_pin, GPIO_SLEW_RATE_FAST);
    gpio_set_function(pwm_cd_pin,GPIO_FUNC_PWM);
    gpio_set_slew_rate(pwm_cd_pin, GPIO_SLEW_RATE_FAST);
    gpio_set_dir(pwm_ab_pin,GPIO_OUT);
    gpio_set_dir(pwm_cd_pin,GPIO_OUT);

    uint slice = pwm_gpio_to_slice_num(pwm_ab_pin);
    uint ch_A = pwm_gpio_to_channel(pwm_ab_pin);
    uint ch_C = pwm_gpio_to_channel(pwm_cd_pin);

    mphb2_arr[i]->slice = slice;
    mphb2_arr[i]->ch_A = ch_A;
    mphb2_arr[i]->ch_C = ch_C;

    pwm_config cfg = pwm_get_default_config();
    // 125MHz --> 100khz | 125M / 1.25 = 100MHz | Top = 500; Level = 1000
    pwm_config_set_clkdiv(&cfg, 1.25f);
    pwm_config_set_wrap(&cfg, 1000);
    pwm_init(slice,&cfg,false);
    mphb_set_dlevel(i, 0);


    // TODO: check connection with PH_EN_PIN as input...
    gpio_init(ph_en_pin);
    gpio_put(ph_en_pin, 0);
    gpio_set_dir(ph_en_pin, GPIO_OUT);

    mphb2_arr[i]->initialized = true;
}

void mphb_set_levels(mphb_port_t i, uint level_A, uint level_C) {
    pwm_set_chan_level(PWM_BD_1_SLICE,PWM_B_1_CHAN,level_A);
    pwm_set_chan_level(PWM_BD_1_SLICE,PWM_D_1_CHAN,level_C);
}

void mphb_set_dlevel(mphb_port_t i, uint dlevel) {
    mphb_set_levels(i, mphb_pwm_cm_level + (dlevel + 1)/2, mphb_pwm_cm_level - (dlevel/2) );
}


void mphb_set_ph_en(mphb_port_t i, bool enable) {
    gpio_put(mphb2_arr[i]->ph_en_pin, enable);
    mphb2_arr[i]->ph_en = enable;
}

void mphb_set_pwm_en(mphb_port_t i, bool enable) {
    pwm_set_enabled(mphb2_arr[i]->slice, enable);
    mphb2_arr[i]->pwm_en = enable;
}



