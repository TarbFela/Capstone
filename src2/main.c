#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//#include <math.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "capstone_pwm.h"
#include "capstone_dsp.h"
#include "capstone_adc.h"


#define PMIC_I2C_SDA_PIN 8
#define PMIC_I2C_SCL_PIN 9
#define PMIC_I2C i2c0


// 48 MHz
#define ADC_BASE_CLOCK_HZ 48000000

int PI_setpoint = 10;
PI_controller_t current_controller;

volatile capstone_adc_struct_t *cas;



void isns_dma_handler() {
    static int d = 100;
    //debugging toggle
    sio_hw->gpio_togl = 0x1<<18;
    // if the second [1] of the DMAs has triggered an interrupt, we can probably assume (DANGEROUS) that we should look halfway through the buffer.
    int data_offset = (ADC_BUFFER_SIZE / 2) * (int) dma_channel_get_irq0_status(cas->adc_dma_daisy_chain[1]);
    // this shouldn't produce a branch... I think?
    int culprit_dma_daisy_chain_index = (data_offset != 0);
    uint16_t *data = &(cas->adc_dma_buffer[data_offset]);
    // a little stupid, but set the write address to where we starting getting data from.
    // TODO: configure the DMAs as wrapping rings so that they don't need to be reset. or use a third, cleanup DMA
    (cas->adc_dma_daisy_chain_hw[culprit_dma_daisy_chain_index])->write_addr = (uintptr_t)(data);

    // ultra-shitty DSP
    int avg = 0;
    for(int i = 0; i<ADC_BUFFER_SIZE/2; i++) avg+= data[i];
    // TODO: the current buffer size is 256 with a half-size of 128, 2^7 is 128, so divide by 2^7 to get the average
    avg>>=7;

    // 62 = 4096 * 0.05 / 3.3
    int targ = current_controller.PI_SP * 62;
    d += -(avg > targ) + (avg < targ)
        -(avg+20 > targ) + (avg-20 < targ)
        -(avg+40 > targ) + (avg-40 < targ);

    if(d>330) d= 330;
    if(d<110) d= 110;
    pwm_set_gpio_level(PWM1_GPIO_PIN,d);
    pwm_set_gpio_level(PWM3_GPIO_PIN,d);
    // clear the correct interrupt
    dma_hw->ints0 = 0x1 << cas->adc_dma_daisy_chain[culprit_dma_daisy_chain_index];
    // debugging toggler
    sio_hw->gpio_togl = 0x1<<18;
}

void other_core() {
    multicore_fifo_push_blocking(0xBEEF);
    printf("MULTICORE EXPLOSION!!!\n");

    cas = capstone_adc_init();
    irq_set_exclusive_handler(DMA_IRQ_0, isns_dma_handler);

    PI_controller_init(&current_controller,
                       PI_setpoint,
                       569,
                       9,
                       27314,
                       440,
                       500000000,
                       110,
                       250
    );

    void **other_stuff_arr;
    other_stuff_arr = (void *)malloc(sizeof(size_t)*2);
    other_stuff_arr[0] = (void *)(&current_controller);
    other_stuff_arr[1] = NULL;

    gpio_init(18);
    gpio_set_dir(18,GPIO_OUT);
    gpio_pull_up(18);
    gpio_set_slew_rate(18,GPIO_SLEW_RATE_FAST);
    while(1) {
        while(multicore_fifo_pop_blocking()!=0xBEEF); // wait for an input of 1, i.e. CL toggle. input of 0 is just a request for status printing.

        printf("[CL control started. Press 'r' to see status, press the spacebar to pause.]\n");

        // DMA transfer count is a count-DOWN; therefore we are inverting our sample processor counter correspondingly.
        uint32_t samples_processed_inv = ADC_BUFFER_SIZE;
        uint32_t dma_rr_i = 0;
        uint32_t iii = 0;

        pwm_set_gpio_level(PWM1_GPIO_PIN,100);
        pwm_set_gpio_level(PWM2_GPIO_PIN,100);
        pwm_set_gpio_level(PWM3_GPIO_PIN,100);
        pwm_set_gpio_level(PWM4_GPIO_PIN,100);

        capstone_adc_start(cas);

        while(1) {
            int sig = multicore_fifo_pop_blocking();
            if (sig == 0xBEEF) break;
            current_controller.PI_SP = sig;
        }

        printf("[CL control paused, all outputs to 0]\n");
        capstone_adc_stop(cas);
        multicore_fifo_drain();

        pwm_set_gpio_level(PWM1_GPIO_PIN,0);
        pwm_set_gpio_level(PWM2_GPIO_PIN,0);
        pwm_set_gpio_level(PWM3_GPIO_PIN,0);
        pwm_set_gpio_level(PWM4_GPIO_PIN,0);
    }

}

int main(void) {
    stdio_init_all();
    sleep_ms(1000);

    multicore_launch_core1(other_core);
    sleep_ms(1000);
    multicore_fifo_pop_blocking();

    //sleep_ms(5000);

    capstone_pwm_init();
    printf("Hello Capstone World!\n");

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PMIC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PMIC_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PMIC_I2C_SDA_PIN);
    gpio_pull_up(PMIC_I2C_SCL_PIN);

    char ui = 0;
    int latch = 0;
    int pi_en_latch = 0;
    int D1_thresh = 10;
    int D1_thresh_setting_multiplier = 2;
    while(1) {
        scanf("%c",&ui);
        if(ui=='q') break;

        if(ui == 'm') {
            if((++D1_thresh_setting_multiplier)>6) D1_thresh_setting_multiplier=1;
            printf("MULTIPLIER SET TO %d\n",D1_thresh_setting_multiplier);
        }
        if(ui >= '0' && ui <= '9') {
            PI_setpoint = ((uint16_t)ui - '0')*D1_thresh_setting_multiplier;
            printf("PI setpoint: %dA\n",PI_setpoint);
            if(latch) multicore_fifo_push_blocking(PI_setpoint);
        }
        if(ui == 'p' || ui == 'l') {
            if(ui=='p') PI_setpoint+= D1_thresh_setting_multiplier/4;
            if(ui=='l') PI_setpoint+= -D1_thresh_setting_multiplier/4;
            printf("PI setpoint: %dA\n",PI_setpoint);
            if(latch) multicore_fifo_push_blocking(PI_setpoint);
        }
        if(ui == 'i') {
            printf("Writing to INA236...\n");
            uint8_t INA236B_msg[3] = {7,0xAB,0xCD};
            uint8_t *INA236_read_dst;
            INA236_read_dst = (uint8_t *)malloc(sizeof(uint8_t) * 2);
            i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 3, 1, 1000000);
            i2c_read_timeout_us(i2c0,0x48,INA236_read_dst,2,0,1000000);
            printf("Read values: 0x%02X%02X\n",INA236_read_dst[0],INA236_read_dst[1]);

            // 4127h
            INA236B_msg[0] = 0; //config reg
            INA236B_msg[1] = 0x41 | 0x1<<4; // default config MSbyte with shunt ADC range set to 20.48mV
            INA236B_msg[2] = 0x27; // default config LSbyte

            i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 3, 1, 1000000);

            free(INA236_read_dst);
        }
        if(ui == 'u') {
            printf("Reading INA236 Shunt Voltage...\n");
            uint8_t INA236B_msg[1] = {0x1};
            uint8_t *INA236_read_dst;
            INA236_read_dst = (uint8_t *)malloc(sizeof(uint8_t) * 2);
            int16_t *INA236_ADC_val = (uint16_t *)INA236_read_dst;
            i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 1, 1, 1000000);
            i2c_read_timeout_us(i2c0,0x48,INA236_read_dst,2,0,1000000);
            printf("Read values: 0x%02X%02X\n",INA236_read_dst[0],INA236_read_dst[1]);
            printf("ADC value: %d\n",*INA236_ADC_val);
            free(INA236_read_dst);
        }
        if(ui == 'v') {
            printf("Reading INA236 Bus Voltage...\n");
            uint8_t INA236B_msg[1] = {0x2};
            uint8_t *INA236_read_dst;
            INA236_read_dst = (uint8_t *)malloc(sizeof(uint8_t) * 2);

            i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 1, 1, 1000000);
            i2c_read_timeout_us(i2c0,0x48,INA236_read_dst,2,0,1000000);
            printf("Read values: 0x%02X%02X\n",INA236_read_dst[0],INA236_read_dst[1]);
            printf("ADC value: %d\n",((int16_t)INA236_read_dst[0] << 8) | ((int16_t)INA236_read_dst[1]));
            free(INA236_read_dst);
        }
        if(ui == ' ') {
            latch = !latch;
            multicore_fifo_push_blocking(0xBEEF);
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}