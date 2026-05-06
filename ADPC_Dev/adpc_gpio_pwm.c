#include "adpc_gpio_pwm.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

void mphb_gpio_init(mphb2_gpio_pwm_t *s, int pwm_ab_pin, int pwm_cd_pin, int ph_en_pin) {
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

    pwm_config cfg = pwm_get_default_config();
    // 125MHz --> 100khz | 125M / 1.25 = 100MHz | Top = 500; Level = 1000
    pwm_config_set_clkdiv(&cfg, 1.25f);
    pwm_config_set_wrap(&cfg, 1000);
    pwm_init(slice,&cfg,false);
    pwm_set_chan_level(slice,ch_A,300);
    pwm_set_chan_level(slice,ch_C,300);

    gpio_init(ph_en_pin);
    gpio_put(ph_en_pin, 0);
    gpio_set_dir(ph_en_pin, GPIO_OUT);

    s->pwm_a_pin = pwm_ab_pin;
    s->pwm_c_pin = pwm_cd_pin;
    s->ph_en_pin = ph_en_pin;
    s->slice = slice;
    s->ch_A = ch_A;
    s->ch_C = ch_C;

}

#endif
