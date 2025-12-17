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
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "capstone_pwm.h"
#include "capstone_dsp.h"
#include "capstone_adc.h"


#define PMIC_I2C_SDA_PIN 8
#define PMIC_I2C_SCL_PIN 9
#define PMIC_I2C i2c0

#define NA_ADC_PIN 26

// 48 MHz
#define ADC_BASE_CLOCK_HZ 48000000


volatile PI_controller_t current_controller;

volatile capstone_adc_struct_t *cas;
volatile uint16_t TSNS_ADC_value_12_bit_avg, ISNS_ADC_value_12_bit_avg = 0;


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

    // ultra-shitty DSP:
    int isns_avg = 0;
    for(int i = 0; i<ADC_BUFFER_SIZE/4; i++) isns_avg+= data[2*i];

    int tsns_avg = 0;
    for(int i = 0; i<ADC_BUFFER_SIZE/4; i++) tsns_avg += data[2*i + 1];
    
    // TODO: the current buffer size is 256 with a quarter-size (half per DMA, half per ADC ch) of 64. Divide by 2^6.
    tsns_avg>>=6; // bring TSNS to 14-bit average as opposed to 12-bit
    TSNS_ADC_value_12_bit_avg = tsns_avg;
    isns_avg>>=6;
    ISNS_ADC_value_12_bit_avg = isns_avg;


    // psuedo-integral control. Increment faster for bigger errors.
    // Don't change the duty cycle if the controller is paused.
    if(!current_controller.controller_paused) {
        // 62 = 4096 * 0.05 / 3.3
        int targ = current_controller.PI_SP * 62;
        d += (-(isns_avg > targ) + (isns_avg < targ)
              - 2 * (isns_avg + 16 > targ) + 2 * (isns_avg - 16 < targ)
              - 2 * (isns_avg + 32 > targ) + 2 * (isns_avg - 32 < targ));
        if(d>330) d= 330;
        if(d<110) d= 110;
        pwm_set_gpio_level(PWM1_GPIO_PIN,d);
        pwm_set_gpio_level(PWM3_GPIO_PIN,d);
    }


    // clear the correct interrupt
    dma_hw->ints0 = 0x1 << cas->adc_dma_daisy_chain[culprit_dma_daisy_chain_index];
    // debugging toggler
    sio_hw->gpio_togl = 0x1<<18;
}


void vtc_offset_sweep() {
    printf("[RUNNING VTC OFFSET SWEEP]\n");
    printf("Starting ADC and DMAs...\n");

    pwm_set_gpio_level(PWM1_GPIO_PIN,0);
    pwm_set_gpio_level(PWM2_GPIO_PIN,0);
    pwm_set_gpio_level(PWM3_GPIO_PIN,0);
    pwm_set_gpio_level(PWM4_GPIO_PIN,0);

    current_controller.controller_paused = 1;
    capstone_adc_start(cas);



    sleep_ms(1000);
    uint64_t time_us;
    uint16_t i_measurements[256];
    uint16_t t_measurements[256];
    uint16_t i_setpoints[16];
    int mmi = 0;
    int ispi = 0;

    // sweep currents
    for(int i = 6; i<25; i+=3) {
        current_controller.PI_SP = i;
        sleep_ms(500);
        i_setpoints[ispi++] = i;
        printf("SWEEP LEVEL %d...\n",i);
        // for each current setpoint, take three pairs of measurements
        // first, an initial measurement set:
        //      output-off measurement
        //      output-on measurement (post-settling)
        //      output-on measurement (after brief heating)
        t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
        i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;

        pwm_set_gpio_level(PWM1_GPIO_PIN,100);
        pwm_set_gpio_level(PWM2_GPIO_PIN,100);
        pwm_set_gpio_level(PWM3_GPIO_PIN,100);
        pwm_set_gpio_level(PWM4_GPIO_PIN,100);

        current_controller.controller_paused = 0;
        sleep_ms(400);
        current_controller.controller_paused = 1;
        sleep_ms(100);
        t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
        i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;
        current_controller.controller_paused = 0;
        sleep_ms(400);
        current_controller.controller_paused = 1;
        sleep_ms(100);
        t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
        i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;

        // second, a middle-ground measurement set:
        //      regular-interval measurements
        current_controller.controller_paused = 0;

        for(int ii = 0; ii<10; ii++) {
            printf("\tMID SWEEP %d...\n",ii);
            sleep_ms(500);
            current_controller.controller_paused = 1;
            sleep_ms(500);
            t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
            i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;
        }
        // third, a final ("warm") measurement set:
        //      two well-spaced output-on measurements
        //      two well-spaced output-off measurements
        sleep_ms(1000);
        current_controller.controller_paused = 1;
        sleep_ms(100);
        t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
        i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;
        current_controller.controller_paused = 0;
        sleep_ms(300);
        current_controller.controller_paused = 1;
        sleep_ms(100);
        t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
        i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;

        pwm_set_gpio_level(PWM1_GPIO_PIN,0);
        pwm_set_gpio_level(PWM2_GPIO_PIN,0);
        pwm_set_gpio_level(PWM3_GPIO_PIN,0);
        pwm_set_gpio_level(PWM4_GPIO_PIN,0);

        sleep_ms(400);
        t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
        i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;
        sleep_ms(400);
        t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
        i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;

        printf("Cooling down...\n");
        sleep_ms(500);
    }

    capstone_adc_stop(cas);

    int mmi_ispi_ratio = (mmi+1)/(ispi+1);
    mmi--;
    while(ispi--) {
        printf("%d,\n",i_setpoints[ispi]);
        for(int ii = 0; ii<17; ii++) {
            printf("\t ,%d, %d\n",i_measurements[mmi],t_measurements[mmi--]);
        }
    }


}

void network_analyzer_slave_adc_handler(void) {
    //static uint8_t i = 0;
    uint8_t val = adc_fifo_get() + 100;
    // since the ADC is shifting down to 8 bits, we have a 256 FSR
    // that gives us a nice, 100–356 range. That works well!
    pwm_set_gpio_level(PWM1_GPIO_PIN,val);
    pwm_set_gpio_level(PWM3_GPIO_PIN, val);
    adc_fifo_drain();
}

// override the CAS with our own ADC to PWM handler.
void network_analyzer_slave_start() {
    printf("Initializing Network Analyzer Mode...\n");
    adc_init();
    adc_run(false);
    adc_gpio_init(NA_ADC_PIN);
    adc_select_input(NA_ADC_PIN-26);
    adc_fifo_setup(
            true,    // Write each completed conversion to the sample FIFO
            false,    // Enable DMA data request
            1,       // DREQ (and IRQ) asserted when at least 1 sample present
            false,   // We won't see the ERR bit because of 8 bit reads; disable.
            true     // Shift each sample to 8 bits when pushing to FIFO?
    );
    adc_irq_set_enabled(true);

    irq_set_exclusive_handler(ADC_IRQ_FIFO,network_analyzer_slave_adc_handler);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    // no round-robin
    adc_set_round_robin(0);
    // our PWM is 125kHz, so our ADC should be 125kHz? sure. 48MHz / 125kHz = 384
    adc_set_clkdiv(192);
    printf("Starting PWM Outputs...\n");

    pwm_set_gpio_level(PWM1_GPIO_PIN,100);
    pwm_set_gpio_level(PWM2_GPIO_PIN,100);
    pwm_set_gpio_level(PWM3_GPIO_PIN,100);
    pwm_set_gpio_level(PWM4_GPIO_PIN,100);

    sleep_ms(1000);

    adc_run(true);
    printf("ADC Started.\n");
}

// stops the network analyzer program and re-inits the CAS.
void network_analyzer_slave_stop() {
    printf("Stopping NA mode...\n");
    adc_run(false);
    adc_fifo_drain();
    irq_set_enabled(ADC_IRQ_FIFO,false);
    pwm_set_gpio_level(PWM1_GPIO_PIN,0);
    pwm_set_gpio_level(PWM2_GPIO_PIN,0);
    pwm_set_gpio_level(PWM3_GPIO_PIN,0);
    pwm_set_gpio_level(PWM4_GPIO_PIN,0);
    capstone_adc_init(cas, isns_dma_handler);
    printf("NA mode exited.\n");
}

void other_core() {
    multicore_fifo_push_blocking(0xBEEF);
    printf("MULTICORE EXPLOSION!!!\n");

    cas = (capstone_adc_struct_t *)malloc(sizeof(capstone_adc_struct_t));
    capstone_adc_init(cas, isns_dma_handler);


    PI_controller_init(&current_controller,
                       10,
                       569,
                       9,
                       27314,
                       440,
                       500000000,
                       110,
                       250
    );

    gpio_init(18);
    gpio_set_dir(18,GPIO_OUT);
    gpio_pull_up(18);
    gpio_set_slew_rate(18,GPIO_SLEW_RATE_FAST);


    sleep_ms(100);
    int ui = 0;
    while(1) {
        ui = multicore_fifo_pop_blocking();
        if(ui == 0xBEEF) {

            printf("[CL control started. Press 'r' to see status, press the spacebar to pause.]\n");
            pwm_set_gpio_level(PWM1_GPIO_PIN, 100);
            pwm_set_gpio_level(PWM2_GPIO_PIN, 100);
            pwm_set_gpio_level(PWM3_GPIO_PIN, 100);
            pwm_set_gpio_level(PWM4_GPIO_PIN, 100);

            capstone_adc_start(cas);

            while (1) {
                uint32_t sig = multicore_fifo_pop_blocking();
                if (sig == 0xBEEF) break;
                //current_controller.PI_SP = sig;
            }

            printf("[CL control paused, all outputs to 0]\n");
            capstone_adc_stop(cas);
            multicore_fifo_drain();

            pwm_set_gpio_level(PWM1_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM2_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM3_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM4_GPIO_PIN, 0);
        }
        else if(ui == 0xABCD) {
            network_analyzer_slave_start();
            while (1) {
                uint32_t sig = multicore_fifo_pop_blocking();
                if (sig == 0xABCD) break;
                //current_controller.PI_SP = sig;
            }
            network_analyzer_slave_stop();
        }
        else if(ui == 0xDEAD) {
            vtc_offset_sweep();
        }
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
    int PI_setpoint = 10;
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
        if(ui == 'g') {
            multicore_fifo_push_blocking(0xABCD);
        }
        if(ui == 'c') {
            multicore_fifo_push_blocking(0xDEAD);
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}