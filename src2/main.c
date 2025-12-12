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


#define PMIC_I2C_SDA_PIN 8
#define PMIC_I2C_SCL_PIN 9
#define PMIC_I2C i2c0

#define ISNS_ADC_PIN 28
#define TSNS_ADC_PIN 27

// shouldn't need a huge buffer, just enough to "catch up" to unprocessed samples
#define ADC_DMA_BUFFER_SIZE 64

// 48 MHz
#define ADC_BASE_CLOCK_HZ 48000000

#define ADC_BUFFER_SIZE 2048
#define ADC_BUFFER_SIZE_WRAP_MASK 0x7FF

int PI_setpoint = 10;

void process_ISNS(uint16_t *sample, uint32_t counter) {
    if(!((counter+1)&0xFFFF)) printf("I %d\n",*sample);
}
void process_TSNS(uint16_t *sample, uint32_t counter) {
    if(!(counter&0xFFFF)) printf("T %d\n",*sample);
}

void (*sample_processors[])(uint16_t *, uint32_t) = {process_ISNS, process_TSNS};

void other_core() {
    multicore_fifo_push_blocking(0xBEEF);
    printf("MULTICORE EXPLOSION!!!\n");

    adc_init();
    adc_run(false);
    adc_gpio_init(ISNS_ADC_PIN);
    adc_gpio_init(TSNS_ADC_PIN);
    adc_select_input(ISNS_ADC_PIN-26);
    adc_fifo_setup(
            true,    // Write each completed conversion to the sample FIFO
            true,    // Enable DMA data request (DREQ)
            1,       // DREQ (and IRQ) asserted when at least 1 sample present
            false,   // We won't see the ERR bit because of 8 bit reads; disable.
            false     // Shift each sample to 8 bits when pushing to FIFO?
    );

    // ROUND ROBIN ORDER:
    // AINSEL initial value is ISNS (28)
    // subsequent RR goes TSNS, ISNS, TSNS, ISNS, [...]
    // So, actual order is: ISNS, TSNS, ISNS, TSNS, [...]
    adc_set_round_robin(0x1<<(ISNS_ADC_PIN-26) | 0x1<<(TSNS_ADC_PIN-26));
    // 96kHz = 48MHz / 500
    adc_set_clkdiv(500);

    uint *adc_dma_daisy_chain;
    adc_dma_daisy_chain = (uint *)malloc(sizeof(uint)*2);
    adc_dma_daisy_chain[0] = dma_claim_unused_channel(true);
    adc_dma_daisy_chain[1] = dma_claim_unused_channel(true);
    uint16_t *adc_dma_buffer;
    adc_dma_buffer = (uint16_t *)malloc(sizeof(uint16_t)*ADC_BUFFER_SIZE);

    dma_channel_hw_t *adc_dma_daisy_chain_hw[2];
    adc_dma_daisy_chain_hw[0] = &dma_hw->ch[adc_dma_daisy_chain[0]];
    adc_dma_daisy_chain_hw[1] = &dma_hw->ch[adc_dma_daisy_chain[1]];

    while(1) {
        while(multicore_fifo_pop_blocking()!=1); // wait for an input of 1, i.e. CL toggle. input of 0 is just a request for status printing.
        adc_run(false);


        int sample;
        // "IS" refers to "integer scaling" where I am scaling up the values to improve precision in stored values, which are bit-shifted down to usbale duty cycle values.
        // "IS" variables *already factor in* PI constants (ki = 0.41677, kp = 0.00041677)
        int err_IS;
        int y_k_IS = 0;
        int d_IS;
        int d;

        int i = 0;
        printf("[CL control started. Press 'r' to see status, press the spacebar to pause.]\n");
        adc_select_input(ISNS_ADC_PIN-26);
        adc_fifo_drain();


        dma_channel_config cfg = dma_channel_get_default_config(adc_dma_daisy_chain[0]);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_dreq(&cfg, DREQ_ADC);
        channel_config_set_chain_to(&cfg, adc_dma_daisy_chain[1]);
        dma_channel_configure(adc_dma_daisy_chain[0], &cfg,
                              adc_dma_buffer,    // dst
                              &adc_hw->fifo,  // src
                              ADC_BUFFER_SIZE,  // transfer count
                              false            // DON'T start immediately
        );
        dma_channel_config cfg1 = dma_channel_get_default_config(adc_dma_daisy_chain[1]);
        channel_config_set_transfer_data_size(&cfg1, DMA_SIZE_16);
        channel_config_set_read_increment(&cfg1, false);
        channel_config_set_write_increment(&cfg1, true);
        channel_config_set_dreq(&cfg1, DREQ_ADC);
        channel_config_set_chain_to(&cfg1, adc_dma_daisy_chain[0]);
        dma_channel_configure(adc_dma_daisy_chain[1], &cfg1,
                              adc_dma_buffer,    // dst
                              &adc_hw->fifo,  // src
                              ADC_BUFFER_SIZE,  // transfer count
                              false            // DON'T start immediately
        );

        dma_channel_config cfgs[2] = {cfg,cfg1};

        adc_run(true);
        dma_channel_start(adc_dma_daisy_chain[0]);

        // DMA transfer count is a count-DOWN; therefore we are inverting our sample processor counter correspondingly.
        uint32_t samples_processed_inv = ADC_BUFFER_SIZE;
        uint32_t dma_rr_i = 0;
        uint32_t iii = 0;
        while(!multicore_fifo_rvalid()) {
            while(samples_processed_inv > adc_dma_daisy_chain_hw[dma_rr_i]->transfer_count);
            sample_processors[samples_processed_inv&0x1](&adc_dma_buffer[ADC_BUFFER_SIZE-samples_processed_inv], iii++);
            samples_processed_inv--;
            // when samples_processed_inv WRAPS below zero, the MSB will be high. Use this to increment which dma we're looking at.
            //dma_rr_i = (dma_rr_i + (samples_processed_inv>>31))&0x1;
            if(samples_processed_inv>ADC_BUFFER_SIZE) {
                adc_dma_daisy_chain_hw[dma_rr_i]->write_addr = (uintptr_t) adc_dma_buffer;
                dma_rr_i = 1 - dma_rr_i;
                samples_processed_inv = ADC_BUFFER_SIZE;
                //printf("W\n");
            }
            /*
            y_k_IS += 569*PI_setpoint - 9*sample;
            if(y_k_IS>500000000) y_k_IS=5000000;
            if(y_k_IS<-500000000) y_k_IS=-5000000;
            d_IS = y_k_IS + 27314*PI_setpoint - 440*sample;
            d = d_IS>>16;

            if(!((i)%0x3FFFF)) {
                printf("d: %04d ykIS: %08d Imeas: %0.3fA\n",d,y_k_IS,(float)sample * 20.0*3.3/4096.0);
            }

            if(d>330) d = 200;
            if(d<110) d = 110;
            */
            //pwm_set_gpio_level(PWM1_GPIO_PIN,d);
            //pwm_set_gpio_level(PWM3_GPIO_PIN,d);
        }
        multicore_fifo_drain();
        pwm_set_gpio_level(PWM1_GPIO_PIN,0);
        pwm_set_gpio_level(PWM2_GPIO_PIN,0);
        pwm_set_gpio_level(PWM3_GPIO_PIN,0);
        pwm_set_gpio_level(PWM4_GPIO_PIN,0);
        printf("[CL control paused, all outputs to 0]\n");
        adc_run(false);
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