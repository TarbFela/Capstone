#ifndef ADPC_GPIO_PWM_H
#define ADPC_GPIO_PWM_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

/*
 *  The SW interface for the MPHB-002 boards allows the ph_en and PWM (A & C or B & D) pins to be driven.
 *  Provided are already-instantiated structs (e.g. hb_1A) corresponding to the connectors (labelled on the PCB)
 * as well as an array and enum for indexing into said array. In this way, function like mphb2_gpio_init() can be called
 * using arguments like HB1A.
 *
 */

#define MPHB_PWM_WRAP 1000

typedef struct {
    int pwm_a_pin;
    int pwm_c_pin;
    int ph_en_pin;
    uint ch_A;
    uint ch_C;
    uint slice;
    bool initialized;
    uint32_t pwm_offset;
    bool pwm_en;
    bool ph_en;
} mphb2_gpio_pwm_t;

extern mphb2_gpio_pwm_t hb_2A;
extern mphb2_gpio_pwm_t hb_3B;
extern mphb2_gpio_pwm_t hb_1A;
extern mphb2_gpio_pwm_t hb_3A;
extern mphb2_gpio_pwm_t hb_2B;
extern mphb2_gpio_pwm_t hb_1B;
extern mphb2_gpio_pwm_t *mphb2_arr[6];

extern volatile uint32_t mphb_pwm_cm_level;

typedef enum {HB1A, HB2A, HB3A, HB1B, HB2B, HB3B} mphb_port_t;

// initialize gpio pins, pwm channels, and populate the info struct.
void mphb_gpio_init(mphb_port_t i);

// set each level individually
void mphb_set_levels(mphb_port_t i, uint level_A, uint level_C);
void mphb_set_levels_all(uint level_A, uint level_C);

// set levels differentially about the common mode level (mphb_pwm_cm_level)
void mphb_set_dlevel(mphb_port_t i, int dlevel);
void mphb_set_dlevel_all(int dlevel);

void mphb_set_ph_en(mphb_port_t i, bool enable);
void mphb_set_pwm_en(mphb_port_t i, bool enable);

void mphb_set_ph_en_all(bool enable);
void mphb_set_pwm_en_all(bool enable);

void mphb_setup_multiphase_masked(uint32_t phases_mask);

#endif