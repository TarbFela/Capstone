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


void other_core() {
    multicore_fifo_push_blocking(0xBEEF);
    printf("MULTICORE EXPLOSION!!!\n");

    capstone_adc_struct_t *cas = capstone_adc_init();

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

    while(1) {
        while(multicore_fifo_pop_blocking()!=1); // wait for an input of 1, i.e. CL toggle. input of 0 is just a request for status printing.


        printf("[CL control started. Press 'r' to see status, press the spacebar to pause.]\n");
        adc_run(false);
        adc_fifo_drain();
        adc_select_input(ISNS_ADC_PIN-26);

        // DMA transfer count is a count-DOWN; therefore we are inverting our sample processor counter correspondingly.
        uint32_t samples_processed_inv = ADC_BUFFER_SIZE;
        uint32_t dma_rr_i = 0;
        uint32_t iii = 0;



        pwm_set_gpio_level(PWM1_GPIO_PIN,100);
        pwm_set_gpio_level(PWM2_GPIO_PIN,100);
        pwm_set_gpio_level(PWM3_GPIO_PIN,100);
        pwm_set_gpio_level(PWM4_GPIO_PIN,100);

        capstone_adc_start(cas);
        while(!multicore_fifo_rvalid()) {
            while(samples_processed_inv > cas->adc_dma_daisy_chain_hw[dma_rr_i]->transfer_count);
            sample_processors[samples_processed_inv&0x1](
                    &(cas->adc_dma_buffer[ADC_BUFFER_SIZE-samples_processed_inv]),
                    (void *)(&current_controller)
                    );
            samples_processed_inv--;
            // when samples_processed_inv WRAPS below zero, the MSB will be high. Use this to increment which dma we're looking at.
            //dma_rr_i = (dma_rr_i + (samples_processed_inv>>31))&0x1;
            if(samples_processed_inv>ADC_BUFFER_SIZE) {
                cas->adc_dma_daisy_chain_hw[dma_rr_i]->write_addr = (uintptr_t) (cas->adc_dma_buffer);
                dma_rr_i = 1 - dma_rr_i;
                samples_processed_inv = ADC_BUFFER_SIZE;

                current_controller.PI_SP = PI_setpoint;
                //printf("W\n");
                iii++;
                if(!(iii&0x3)) {
                    printf("D%d\n",current_controller.d);
                    printf("Y%d\n",current_controller.y_k_IS);
                }
                //current_controller.PI_SP = PI_setpoint;
            }

            //
            //
        }
        pwm_set_gpio_level(PWM1_GPIO_PIN,0);
        pwm_set_gpio_level(PWM2_GPIO_PIN,0);
        pwm_set_gpio_level(PWM3_GPIO_PIN,0);
        pwm_set_gpio_level(PWM4_GPIO_PIN,0);
        printf("[CL control paused, all outputs to 0]\n");
        adc_run(false);
        multicore_fifo_drain();
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
        }
        if(ui == 'p' || ui == 'l') {
            if(ui=='p') PI_setpoint+= D1_thresh_setting_multiplier/4;
            if(ui=='l') PI_setpoint+= -D1_thresh_setting_multiplier/4;
            printf("PI setpoint: %dA\n",PI_setpoint);
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
            multicore_fifo_push_blocking(1);
        }
        if(ui == 'r') {
            multicore_fifo_push_blocking(0);
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}