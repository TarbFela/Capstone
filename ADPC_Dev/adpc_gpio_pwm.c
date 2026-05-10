#include "adpc_gpio_pwm.h"
#include "ADPC_cfg.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <stdio.h>

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

volatile uint32_t mphb_pwm_cm_level = MPHB_PWM_WRAP/2;

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
    pwm_config_set_wrap(&cfg, MPHB_PWM_WRAP);
    pwm_init(slice,&cfg,false);
    mphb_set_dlevel(i, 0);


    // TODO: check connection with PH_EN_PIN as input...
    gpio_init(ph_en_pin);
    gpio_put(ph_en_pin, 0);
    gpio_set_dir(ph_en_pin, GPIO_OUT);

    mphb2_arr[i]->initialized = true;
}

//  pass a mask constructed like so:
// (1U<<HB1A) | (1U<<HB2B)
// or whatever your situation is
//  Will automatically assign phase shifts (PWM counter offsets) based on number of bits masked.
//  Probably best to perform all necessary mphb_gpio_inits **first**.
void mphb_setup_multiphase_masked(uint32_t phases_mask) {
    int n = __builtin_popcount(phases_mask); // number of bits set in mask
    int ni = 0; // counter as we find bits in mask
    for(int i = HB1A; i<=HB3B; i++) { // go through the bits
        if ( (phases_mask>>i) & 0x1 ) pwm_set_counter(mphb2_arr[i]->slice, (ni*mphb_pwm_cm_level)/n);
    }
}

void mphb_set_levels(mphb_port_t i, uint level_A, uint level_C) {
    pwm_set_chan_level(mphb2_arr[i]->slice,mphb2_arr[i]->ch_A,level_A);
    pwm_set_chan_level(mphb2_arr[i]->slice,mphb2_arr[i]->ch_C,level_C);
}

void mphb_set_levels_all(uint level_A, uint level_C) {
    for(mphb_port_t i = 0; i<6; i++) {
        mphb_set_levels(i, level_A, level_C);
    }
}

void mphb_set_dlevel(mphb_port_t i, int dlevel) {
    mphb_set_levels(i, mphb_pwm_cm_level + (dlevel + 1)/2, mphb_pwm_cm_level - (dlevel/2) );
}

void mphb_set_dlevel_all(int dlevel) {
    for(mphb_port_t i = 0; i<6; i++) {
        mphb_set_dlevel(i, dlevel);
    }
}

void mphb_set_dlevel_all_spatial_dithering(float d) {
    float Nph = 0; // number of active phases
    for(mphb_port_t i = HB1A; i<=HB3B; i++) Nph += mphb2_arr[i]->ph_en;
//    printf("%d\n",Nph);
    float n = 0;
    for(mphb_port_t i = HB1A; i<=HB3B; i++) {
        if(!mphb2_arr[i]->ph_en) continue; // only do active phases.
        // convert to PWM value
        float flevel = (500*d + (1+n)/(2*Nph));
//        printf("%.2f\t",flevel);
         mphb_set_dlevel(i, (int) flevel);
//        printf("%d\n",level);
        n++;
    }
}


void mphb_set_ph_en(mphb_port_t i, bool enable) {
    gpio_put(mphb2_arr[i]->ph_en_pin, enable);
    mphb2_arr[i]->ph_en = enable;
}

// sets the enable for all **initialized** items.
void mphb_set_ph_en_all(bool enable) {
    for(mphb_port_t i = HB1A; i<=HB3B; i++) {
        if(mphb2_arr[i]->initialized) mphb_set_ph_en(i,enable);
    }
}

void mphb_set_pwm_en(mphb_port_t i, bool enable) {
    pwm_set_enabled(mphb2_arr[i]->slice, enable);
    mphb2_arr[i]->pwm_en = enable;
}

// sets the enable for all **initialized** items.
void mphb_set_pwm_en_all(bool enable) {
    for(mphb_port_t i = HB1A; i<=HB3B; i++) {
        if(mphb2_arr[i]->initialized) mphb_set_pwm_en(i,enable);
    }
}



