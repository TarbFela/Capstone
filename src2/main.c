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

// 48 MHz
#define ADC_BASE_CLOCK_HZ 48000000

#define ADC_BUFFER_SIZE 16

void other_core() {
    multicore_fifo_push_blocking(0xBEEF);
    printf("MULTICORE EXPLOSION!!!\n");

    adc_init();
    adc_gpio_init(ISNS_ADC_PIN);
    adc_select_input(ISNS_ADC_PIN-26);
    adc_fifo_setup(
            true,    // Write each completed conversion to the sample FIFO
            true,    // Enable DMA data request (DREQ)
            1,       // DREQ (and IRQ) asserted when at least 1 sample present
            false,   // We won't see the ERR bit because of 8 bit reads; disable.
            false     // Shift each sample to 8 bits when pushing to FIFO?
    );

    //100Hz
    adc_set_clkdiv(ADC_BASE_CLOCK_HZ/100);

    uint16_t *adc_buffer;
    adc_buffer = (uint16_t *)malloc(sizeof(uint16_t)*ADC_BUFFER_SIZE);
    if(adc_buffer == NULL) printf("BAD MALLOC!!!\n");
    uint adc_dma_channel = dma_claim_unused_channel(true);
    while(1) {
        multicore_fifo_pop_blocking();
        adc_run(false);
        dma_channel_config cfg = dma_channel_get_default_config(adc_dma_channel);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_dreq(&cfg, DREQ_ADC);
        dma_channel_configure(adc_dma_channel, &cfg,
                              adc_buffer,    // dst
                              &adc_hw->fifo,  // src
                              ADC_BUFFER_SIZE,  // transfer count
                              false            // DON'T start immediately
        );
        adc_run(true);
        dma_channel_start(adc_dma_channel);
        dma_channel_wait_for_finish_blocking(adc_dma_channel);
        multicore_fifo_push_blocking((uint32_t)adc_buffer);
    }

}

int main(void) {
    stdio_init_all();
    sleep_ms(5000);

    multicore_launch_core1(other_core);
    sleep_ms(1000);
    multicore_fifo_pop_blocking();

    sleep_ms(5000);

    capstone_pwm_init();
    printf("Hello Capstone World!\n");

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PMIC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PMIC_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PMIC_I2C_SDA_PIN);
    gpio_pull_up(PMIC_I2C_SCL_PIN);

    char ui = 0;
    int latch = 0;
    uint16_t D1_thresh = 100;
    uint16_t D1_thresh_setting_offset = 100;
    uint16_t D1_thresh_setting_multiplier = 2;
    while(1) {
        scanf("%c",&ui);
        if(ui=='q') break;
        if(ui == ' ') {
            if(latch) {
                latch = 0;
                pwm_set_gpio_level(PWM1_GPIO_PIN, 0);
                pwm_set_gpio_level(PWM2_GPIO_PIN, 0);
                pwm_set_gpio_level(PWM3_GPIO_PIN, 0);
                pwm_set_gpio_level(PWM4_GPIO_PIN, 0);
                printf("PWM OFF\n");
            }
            else {
                latch = 1;
                pwm_set_gpio_level(PWM1_GPIO_PIN, D1_thresh);
                pwm_set_gpio_level(PWM2_GPIO_PIN, 100);
                pwm_set_gpio_level(PWM3_GPIO_PIN, D1_thresh);
                pwm_set_gpio_level(PWM4_GPIO_PIN, 100);
                printf("PWM ON, D = %.2f\n",100.0*(float)D1_thresh/1000);
            }
        }
        if(ui == 'o') {
            if(D1_thresh_setting_offset) {
                D1_thresh_setting_offset = 0;
                printf("BIAS SET TO 0 \n");
            }
            else {
                D1_thresh_setting_offset = 100;
                printf("BIAS SET TO 50 \n");
            }
        }
        if(ui == 'm') {
            D1_thresh_setting_multiplier = D1_thresh_setting_multiplier*1.5 + 1;
            if(D1_thresh_setting_multiplier>100) D1_thresh_setting_multiplier=1;
            printf("MULTIPLIER SET TO %d\n",D1_thresh_setting_multiplier);
        }
        if(ui >= '0' && ui <= '9') {
            D1_thresh = ((uint16_t)ui - '0')*D1_thresh_setting_multiplier + D1_thresh_setting_offset;
            printf("D = %.2f\n",100.0*(float)D1_thresh/1000);
            if(latch) {
                pwm_set_gpio_level(PWM1_GPIO_PIN, D1_thresh);
                pwm_set_gpio_level(PWM3_GPIO_PIN,D1_thresh);
            }
        }
        if(ui == 'p' || ui == 'l') {
            if(ui=='p') D1_thresh+= D1_thresh_setting_multiplier/4;
            if(ui=='l') D1_thresh+= -D1_thresh_setting_multiplier/4;
            printf("D = %.2f\n",100.0*(float)D1_thresh/1000);
            if(latch) {
                pwm_set_gpio_level(PWM1_GPIO_PIN, D1_thresh);
                pwm_set_gpio_level(PWM3_GPIO_PIN,D1_thresh);
            }
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
        if(ui == 'w') {
            multicore_fifo_push_blocking(0);
            uint16_t *buffer = (uint16_t *)multicore_fifo_pop_blocking();
            printf("SAMPLES:\n\t");
            for(int i =0; i<ADC_BUFFER_SIZE; i++) {
                printf("%04d ",buffer[i]);
                if(!((i+1)%4)) printf("\n\t");
            }
            printf("\n");
        }

    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}