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
#include "capstone_thermocouple.h"
#include "capstone_w25.h"


#define PMIC_I2C_SDA_PIN 8
#define PMIC_I2C_SCL_PIN 9
#define PMIC_I2C i2c0
#define PMIC_I2C_TIMEOUT_US 10000

#define NA_ADC_PIN 26

#define PWMTOOL_GPIO_PIN 7

#define UI_SIG_ICTL_START_STOP 0xBEEF
#define UI_SIG_NA_START_STOP 0xABCD
#define UI_SIG_CURRENT_CONTROLLER_PAUSE_UNPAUSE 0xFACE
#define UI_SIG_PWM_INC 0xBABB
#define UI_SIG_PWM_DEC 0xBABA
#define UI_SIG_PWM_START_STOP 0xBAAA
#define UI_SIG_PWM_READ_ISNS 0xBAAB

mutex_t current_controller_lock;
volatile PI_controller_t current_controller;

volatile capstone_adc_struct_t *cas;
volatile uint16_t *TSNS_ADC_value_12_bit_avg, *ISNS_ADC_value_12_bit_avg;
volatile uint16_t d_glob = 0;
volatile uint16_t T_glob = 0;
volatile uint32_t VTCMV_glob = 0;

int32_t INA236_read_bus_voltage() {
    //printf("Reading INA236 Bus Voltage...\n");
    uint8_t INA236B_msg[1] = {0x2};
    uint8_t INA236_read_dst[2];
    int i2cret = i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 1, 1, PMIC_I2C_TIMEOUT_US);
    if(i2cret != 1) return -1000;
    i2cret = i2c_read_timeout_us(i2c0,0x48,INA236_read_dst,2,0,PMIC_I2C_TIMEOUT_US);
    if(i2cret != 2) return -2000;
    //printf("Read values: 0x%02X%02X\n",INA236_read_dst[0],INA236_read_dst[1]);
    //printf("ADC value: %d\n",((int16_t)INA236_read_dst[0] << 8) | ((int16_t)INA236_read_dst[1]));
    return ((int16_t)INA236_read_dst[0] << 8) | ((int16_t)INA236_read_dst[1]);
}

#define DATALOGGING_BASE_ADDR 0x11000
// multicore datalogging signal flag
#define MC_DL_TRIG 0xBEEF0000
#define MC_DL_FLAG_MASK 0xFFFF0000
#define MC_DL_BUFF_I_MASK 0x0000FFFF
volatile int datalogger_q_w = 0;
int datalogger_q_r = 0;
int datalogging_pages_written = 0;
int datalogging_waddr = DATALOGGING_BASE_ADDR;


void isns_dma_handler() {
    static int counter = 0;
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
    tsns_avg >>= 6;
    isns_avg >>= 6;

    // every fourth DSP IRQ should log a "sample" (x2ch)
    if(((++counter)&0x3) == 0) {
        TSNS_ADC_value_12_bit_avg[datalogger_q_w] = tsns_avg;
        ISNS_ADC_value_12_bit_avg[datalogger_q_w] = isns_avg;

        //                                                  DATALOGGING
        // round-robin buffer/queue
        // TRIGGER every 64 samples (128 bytes x 2ch = 256bytes = 1 page)
        // WRAP every 128 samples
        if ((datalogger_q_w & 63) == 63) {
            if (multicore_fifo_wready()) {
                multicore_fifo_push_blocking(MC_DL_TRIG | (MC_DL_BUFF_I_MASK&(datalogger_q_w-63)));
            }
        }
        datalogger_q_w++;
        datalogger_q_w &= 127;
    }

    // BEST FIT LINE FROM XY PLOT
    // Vin =  -214.2785663 * Vdiff + 2.908868568
    // ADC to Vtc_mv_4092:
    // Vtc,mv,4092 = -ADCval * 3300/214.28 + 2908.9 * 4092 / 204
    // 15.4005 and 58,348.5
    // 15.4005 = 1,971.264 / 128 ... 1971 / 128 = 15.3984375
    uint32_t vtc_mv_4092 = 58349 - ((tsns_avg * 1971)>>7);
    VTCMV_glob = vtc_mv_4092;
    vtc_mv_4092 += ktype_voltages_20C_x4092[2];
    uint16_t T = 0;
    for(T = 0; T < ktype_voltages_len; T++) {
        if(ktype_voltages_20C_x4092[T] >= vtc_mv_4092) {
            T_glob = T+20;
            int Tstepsize = ktype_voltages_20C_x4092[T] - ktype_voltages_20C_x4092[T-1];
            int Tdiff = ktype_voltages_20C_x4092[T] - vtc_mv_4092;
            T_glob = (T_glob*8) - ((8*Tdiff)/Tstepsize); // T_glob is eight times the actual temperature, with a resolution of 0.125degC
            break;
        }
    }


    // psuedo-integral control. Increment faster for bigger errors.
    // Don't change the duty cycle if the controller is paused.
    if(!current_controller.controller_paused) {
        //          OLD GAIN VALUE WITH G=100
        // 62 = 4096 * 0.05 / 3.3
        // 15.5 = 62 / 4 since we want that kind of resolution on our PISP
        //          NEW GAIN VALUE WITH G=150
        // 4096 * 0.0005 * 150 / 3.3 = 93.1
        // 23.75 ≈ 24 = 93.1 / 4 as per above resolution comment
        int targ = current_controller.PI_SP * 24;
        d += -(isns_avg*2 > targ) + (isns_avg*2 < targ) - 2 * (isns_avg*2 + 16 > targ) - 2 * (isns_avg*2 + 32 > targ);
        if(d>380) d= 380;
        if(d<90) d= 90;
        pwm_set_gpio_level(PWM2_GPIO_PIN,d);
        pwm_set_gpio_level(PWM4_GPIO_PIN,d);
    }
    else {
        d_glob = d;
    }


    // clear the correct interrupt
    dma_hw->ints0 = 0x1 << cas->adc_dma_daisy_chain[culprit_dma_daisy_chain_index];
    // debugging toggler
    sio_hw->gpio_togl = 0x1<<18;
}

void vtc_offset_sweep() {
    return;
//    printf("[RUNNING VTC OFFSET SWEEP]\n");
//    printf("Starting ADC and DMAs...\n");
//
//    pwm_set_gpio_level(PWM1_GPIO_PIN,0);
//    pwm_set_gpio_level(PWM2_GPIO_PIN,0);
//    pwm_set_gpio_level(PWM3_GPIO_PIN,0);
//    pwm_set_gpio_level(PWM4_GPIO_PIN,0);
//
//    if(!mutex_try_enter(&current_controller_lock,NULL)) {
//        printf("MUTEX CLAIMED!!!\n");
//        return;
//    }
//
//    current_controller.controller_paused = 1;
//    capstone_adc_start(cas);
//    sleep_ms(100);
//
//    uint16_t i_measurements[256];
//    uint16_t d_measurements[256];
//    int v_measurements[256];
//    int v_meas;
//    uint16_t t_measurements[256];
//    uint16_t i_setpoints[16];
//    int mmi = 0;
//    int ispi = 0;
//
//    // sweep currents
//    for(int i = 63; i<=75; i+=3) {
//        printf("SWEEP LEVEL %.2f [%d]...\n",i/4.0,i);
//        current_controller.PI_SP = i;
//        i_setpoints[ispi++] = i;
//
//        t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
//        v_meas = INA236_read_bus_voltage();
//        if(v_meas < -999) printf("BAD I2C...");
//        else v_measurements[mmi] = v_meas;
//        d_measurements[mmi] = 0;
//        i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;
//
//        pwm_set_gpio_level(PWM1_GPIO_PIN,100);
//        pwm_set_gpio_level(PWM2_GPIO_PIN,100);
//        pwm_set_gpio_level(PWM3_GPIO_PIN,100);
//        pwm_set_gpio_level(PWM4_GPIO_PIN,100);
//
//        for(int ii = 0; ii<15; ii++) {
//            current_controller.controller_paused = 0; sleep_ms(900);
//            current_controller.controller_paused = 1; sleep_ms(100);
//
//            t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
//            v_meas = INA236_read_bus_voltage();
//            if (v_meas < -999) printf("BAD I2C...");
//            else v_measurements[mmi] = v_meas;
//            d_measurements[mmi] = d_glob;
//            i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;
//        }
//
//        current_controller.controller_paused = 1;
//        pwm_set_gpio_level(PWM1_GPIO_PIN,0);
//        pwm_set_gpio_level(PWM2_GPIO_PIN,0);
//        pwm_set_gpio_level(PWM3_GPIO_PIN,0);
//        pwm_set_gpio_level(PWM4_GPIO_PIN,0);
//
//        for(int ii = 0; ii<5; ii++) {
//            sleep_ms(1000);
//
//            t_measurements[mmi] = TSNS_ADC_value_12_bit_avg;
//            v_meas = INA236_read_bus_voltage();
//            if (v_meas < -999) printf("BAD I2C...");
//            else v_measurements[mmi] = v_meas;
//            d_measurements[mmi] = 0;
//            i_measurements[mmi++] = ISNS_ADC_value_12_bit_avg;
//        }
//        sleep_ms(500);
//    }
//
//    capstone_adc_stop(cas);
//    mutex_exit(&current_controller_lock);
//
//    int mmi_ispi_ratio = (mmi+1)/(ispi+1);
//    ;
//    while(ispi--) {
//        printf("%d,\n",i_setpoints[ispi]);
//        for(int ii = 0; ii<21; ii++) {
//            printf("\t ,%d, %d, %d, %d,\n",i_measurements[--mmi],t_measurements[mmi],d_measurements[mmi],v_measurements[mmi]);
//        }
//    }


}

void network_analyzer_slave_adc_handler(void) {
    return;
//    //static uint8_t i = 0;
//    uint8_t val = adc_fifo_get() + 100;
//    // since the ADC is shifting down to 8 bits, we have a 256 FSR
//    // that gives us a nice, 100–356 range. That works well!
//    pwm_set_gpio_level(PWM1_GPIO_PIN,val);
//    pwm_set_gpio_level(PWM3_GPIO_PIN, val);
//    adc_fifo_drain();
}

// override the CAS with our own ADC to PWM handler.
void network_analyzer_slave_start() {
    return;
//    printf("Initializing Network Analyzer Mode...\n");
//    adc_init();
//    adc_run(false);
//    adc_gpio_init(NA_ADC_PIN);
//    adc_select_input(NA_ADC_PIN-26);
//    adc_fifo_setup(
//            true,    // Write each completed conversion to the sample FIFO
//            false,    // Enable DMA data request
//            1,       // DREQ (and IRQ) asserted when at least 1 sample present
//            false,   // We won't see the ERR bit because of 8 bit reads; disable.
//            true     // Shift each sample to 8 bits when pushing to FIFO?
//    );
//    adc_irq_set_enabled(true);
//
//    irq_set_exclusive_handler(ADC_IRQ_FIFO,network_analyzer_slave_adc_handler);
//    irq_set_enabled(ADC_IRQ_FIFO, true);
//
//    // no round-robin
//    adc_set_round_robin(0);
//    // our PWM is 125kHz, so our ADC should be 125kHz? sure. 48MHz / 125kHz = 384
//    adc_set_clkdiv(192);
//    printf("Starting PWM Outputs...\n");
//
//    pwm_set_gpio_level(PWM1_GPIO_PIN,100);
//    pwm_set_gpio_level(PWM2_GPIO_PIN,100);
//    pwm_set_gpio_level(PWM3_GPIO_PIN,100);
//    pwm_set_gpio_level(PWM4_GPIO_PIN,100);
//
//    sleep_ms(1000);
//
//    adc_run(true);
//    printf("ADC Started.\n");
}

// stops the network analyzer program and re-inits the CAS.
void network_analyzer_slave_stop() {
    return;
//    printf("Stopping NA mode...\n");
//    adc_run(false);
//    adc_fifo_drain();
//    irq_set_enabled(ADC_IRQ_FIFO,false);
//    pwm_set_gpio_level(PWM1_GPIO_PIN,0);
//    pwm_set_gpio_level(PWM2_GPIO_PIN,0);
//    pwm_set_gpio_level(PWM3_GPIO_PIN,0);
//    pwm_set_gpio_level(PWM4_GPIO_PIN,0);
//    capstone_adc_init(cas, isns_dma_handler);
//    printf("NA mode exited.\n");
}

void other_core() {
    multicore_fifo_push_blocking(UI_SIG_ICTL_START_STOP);
    printf("MULTICORE EXPLOSION!!!\n");

    cas = (capstone_adc_struct_t *)malloc(sizeof(capstone_adc_struct_t));
    capstone_adc_init(cas, isns_dma_handler);
    mutex_init(&current_controller_lock);

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
        if(ui == UI_SIG_ICTL_START_STOP) {

            printf("[CL control started. Press 'r' to see status, press the spacebar to pause.]\n");
            pwm_set_gpio_level(PWM1_GPIO_PIN, 100);
            pwm_set_gpio_level(PWM2_GPIO_PIN, 100);
            pwm_set_gpio_level(PWM3_GPIO_PIN, 100);
            pwm_set_gpio_level(PWM4_GPIO_PIN, 100);

            capstone_adc_start(cas);

            while (1) {
                uint32_t sig = multicore_fifo_pop_blocking();
                if (sig == UI_SIG_ICTL_START_STOP) break;
                if(mutex_try_enter(&current_controller_lock,NULL)) {
                    if (sig == UI_SIG_CURRENT_CONTROLLER_PAUSE_UNPAUSE) { current_controller.controller_paused = !current_controller.controller_paused;
                    printf("%s\n",current_controller.controller_paused ? "PAUSED" : "RESUMED");}
                    else {
                        current_controller.PI_SP = sig;
                        printf("\tPISP %d\n", current_controller.PI_SP);
                    }
                    mutex_exit(&current_controller_lock);
                }
                else printf("MUTEX LOCKED\n");
            }

            printf("[CL control paused, all outputs to 0]\n");
            capstone_adc_stop(cas);
            multicore_fifo_drain();

            pwm_set_gpio_level(PWM1_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM2_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM3_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM4_GPIO_PIN, 0);
        }
        else if(ui == UI_SIG_NA_START_STOP) {
            network_analyzer_slave_start();
            while (1) {
                uint32_t sig = multicore_fifo_pop_blocking();
                if (sig == UI_SIG_NA_START_STOP) break;
                //current_controller.PI_SP = sig;
            }
            network_analyzer_slave_stop();
        }
        else if(ui == UI_SIG_PWM_START_STOP) {
            uint16_t pwm_lvl = 100;
            printf("[PWM started. Press 'r' to see status, press the spacebar to pause.]\n");
            pwm_set_gpio_level(PWM1_GPIO_PIN, 100);
            pwm_set_gpio_level(PWM2_GPIO_PIN, pwm_lvl);
            pwm_set_gpio_level(PWM3_GPIO_PIN, 100);
            pwm_set_gpio_level(PWM4_GPIO_PIN, pwm_lvl);

            adc_select_input(ISNS_ADC_PIN - 26);
            while (1) {
                uint32_t sig = multicore_fifo_pop_blocking();
                if (sig == UI_SIG_PWM_START_STOP) break;
                else if(sig == UI_SIG_PWM_READ_ISNS) {
                    uint16_t val = adc_read();
                    float valamps = (float)val/93.1;
                    printf("ADC %04d\t%2.1f\t",val,valamps);
                    val = adc_read();
                    valamps = (float)val/93.1;
                    printf("ADC %04d\t%2.1f\n",val,valamps);
                    continue;
                }
                else if(sig == UI_SIG_PWM_INC) pwm_lvl += 10;
                else if(sig == UI_SIG_PWM_DEC) pwm_lvl -= 10;
                else if(sig <= 9) pwm_lvl = sig*50 + 100;
                pwm_set_gpio_level(PWM2_GPIO_PIN, pwm_lvl);
                pwm_set_gpio_level(PWM4_GPIO_PIN, pwm_lvl);
                printf("\tPWM %d\n",pwm_lvl);
            }
            multicore_fifo_drain();
            printf("[PWM paused, all outputs to 0]\n");

            pwm_set_gpio_level(PWM1_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM2_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM3_GPIO_PIN, 0);
            pwm_set_gpio_level(PWM4_GPIO_PIN, 0);
        }
        else if(ui == 0xDEAD) {
            vtc_offset_sweep();
        }
    }

}

int main(void) {
                        /*** INITS ***/
    stdio_init_all();
    sleep_ms(1000);

    multicore_launch_core1(other_core);
    sleep_ms(1000);
    multicore_fifo_pop_blocking();

    capstone_pwm_init();
    printf("Hello Capstone World!\n");

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PMIC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PMIC_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PMIC_I2C_SDA_PIN);
    gpio_pull_up(PMIC_I2C_SCL_PIN);

    W25_Init();

    TSNS_ADC_value_12_bit_avg = (uint16_t *)malloc(sizeof(uint16_t) * 256);
    ISNS_ADC_value_12_bit_avg = (uint16_t *)malloc(sizeof(uint16_t) * 256);

    if(TSNS_ADC_value_12_bit_avg == NULL || ISNS_ADC_value_12_bit_avg == NULL) {
        sleep_ms(10000);
        printf("BAD MALLOC ON DLBUFFS\n");
        goto reboot;
    }

    char ui = 0;
    int ict_latch = 0;
    int pwm_latch = 0;
    int pi_en_latch = 0;
    int D1_thresh = 10;
    int D1_thresh_setting_multiplier = 2;
    int PI_setpoint = 10;

    while(1) {
        int uii = stdio_getchar_timeout_us(100);
        if(uii != PICO_ERROR_TIMEOUT) {
            ui = (char)uii;
            // QUIT
            if (ui == 'q') break;
            // HELP MENU
            if (ui == 'h') {
                printf("\t\t[HELP MENU]\n"
                       "s: read Flash\n"
                       "m: change multiplier\n"
                       "0-9: set target current or pwm\n"
                       "p & l: inc/dec target current\n"
                       "i: test INA236 writing\n"
                       "u: dep\n"
                       "v: read INA236 bus voltage\n"
                       "space: start/stop PWM output\n"
                       "k: start/stop PI control\n"
                       "g: use with network analyzer\n"
                       "c: perform voffset sweep (dep)\n"
                       "t: read out temperature conversion\n"
                       "j: direct PWM control\n"
                       "r: read raw ISNS ADC value (PWM mode)\n"
                       "spacebar: current controller\n"
                       "\n\n"
                );
            }
            if (ui == 's') {
                if (ict_latch) {
                    printf("[REJECTED] Stop collection to read.\n");
                    continue;
                }
                //round up
                datalogging_pages_written = (datalogging_waddr-DATALOGGING_BASE_ADDR+W25_PAGE_SIZE-1)/256;
                printf("Number of pages written: %d\n", datalogging_pages_written);
                uint8_t *rx_buff = (uint8_t *) malloc(sizeof(uint8_t) * 256);
                if(rx_buff == NULL) {
                    printf("MALLOC FAILED!!\n\n");
                    goto reboot;
                }
                for (int i = 0; i < datalogging_pages_written; i++) {
                    printf("Page %d (0x%08X)\n", i, DATALOGGING_BASE_ADDR | (i << 8));
                    for (int i = 0; i < 256; i++) rx_buff[i] = 0;
                    W25_Read_Data(DATALOGGING_BASE_ADDR | (i << 8), rx_buff, W25_PAGE_SIZE);
                    uint16_t *rx_data = (uint16_t *) rx_buff;
                    for (int i = 0; i < W25_PAGE_SIZE/2; i++) {
                        if (rx_data[i] == 0xFFFF) printf("ND ");
                        else printf("%04X ", rx_data[i]);
                        if (!((i + 1) & 0xF)) printf("\n");
                    }
                    printf("\n");
                }
                free(rx_buff);

            }
            if (ui == 'm') {
                if ((++D1_thresh_setting_multiplier) > 6) D1_thresh_setting_multiplier = 1;
                printf("MULTIPLIER SET TO %d\n", D1_thresh_setting_multiplier);
            }
            if (ui >= '0' && ui <= '9') {
                if (pwm_latch) {
                    // push the int value entered from 0 to 9
                    multicore_fifo_push_blocking((uint32_t) (ui) - '0');
                    continue;
                }
                PI_setpoint = 4 * ((uint16_t) ui - '0') * D1_thresh_setting_multiplier;
                printf("PI setpoint: %.3fA\n", PI_setpoint / 8.0);
                if (ict_latch) {
                    multicore_fifo_push_blocking(PI_setpoint);
                }
            }
            if (ui == 'p' || ui == 'l') {
                if (ict_latch) {
                    if (ui == 'p') PI_setpoint += D1_thresh_setting_multiplier / 2;
                    if (ui == 'l') PI_setpoint += -D1_thresh_setting_multiplier / 2;
                    printf("PI setpoint: %.3fA\n", PI_setpoint / 8.0);
                    if (ict_latch) {
                        multicore_fifo_push_blocking(PI_setpoint);
                    }
                } else if (pwm_latch) {
                    if (ui == 'p') multicore_fifo_push_blocking(UI_SIG_PWM_INC);
                    if (ui == 'l') multicore_fifo_push_blocking(UI_SIG_PWM_DEC);
                }
            }
            if (ui == 'i') {
                printf("Writing to INA236...\n");
                uint8_t INA236B_msg[3] = {7, 0xAB, 0xCD};
                uint8_t *INA236_read_dst;
                INA236_read_dst = (uint8_t *) malloc(sizeof(uint8_t) * 2);
                i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 3, 1, 1000000);
                i2c_read_timeout_us(i2c0, 0x48, INA236_read_dst, 2, 0, 1000000);
                printf("Read values: 0x%02X%02X\n", INA236_read_dst[0], INA236_read_dst[1]);

                // 4127h
                INA236B_msg[0] = 0; //config reg
                INA236B_msg[1] = 0x41 | 0x1 << 4; // default config MSbyte with shunt ADC range set to 20.48mV
                INA236B_msg[2] = 0x27; // default config LSbyte

                i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 3, 1, 1000000);

                free(INA236_read_dst);
            }
            if (ui == 'u') {
                printf("Reading INA236 Shunt Voltage...\n");
                uint8_t INA236B_msg[1] = {0x1};
                uint8_t *INA236_read_dst;
                INA236_read_dst = (uint8_t *) malloc(sizeof(uint8_t) * 2);
                int16_t *INA236_ADC_val = (uint16_t *) INA236_read_dst;
                i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 1, 1, 1000000);
                i2c_read_timeout_us(i2c0, 0x48, INA236_read_dst, 2, 0, 1000000);
                printf("Read values: 0x%02X%02X\n", INA236_read_dst[0], INA236_read_dst[1]);
                printf("ADC value: %d\n", *INA236_ADC_val);
                free(INA236_read_dst);
            }
            if (ui == 'v') {
                printf("Reading INA236 Bus Voltage...\n");
                uint8_t INA236B_msg[1] = {0x2};
                uint8_t *INA236_read_dst;
                INA236_read_dst = (uint8_t *) malloc(sizeof(uint8_t) * 2);

                i2c_write_timeout_us(i2c0, 0x48, INA236B_msg, 1, 1, 1000000);
                i2c_read_timeout_us(i2c0, 0x48, INA236_read_dst, 2, 0, 1000000);
                printf("Read values: 0x%02X%02X\n", INA236_read_dst[0], INA236_read_dst[1]);
                printf("ADC value: %d\n", ((int16_t) INA236_read_dst[0] << 8) | ((int16_t) INA236_read_dst[1]));
                free(INA236_read_dst);
            }
            if (ui == ' ' && !pwm_latch) {
                ict_latch = !ict_latch;
                multicore_fifo_push_blocking(UI_SIG_ICTL_START_STOP);
            }
            if (ui == 'j' && !ict_latch) {
                pwm_latch = !pwm_latch;
                multicore_fifo_push_blocking(UI_SIG_PWM_START_STOP);
            }
            if (ui == 'r' && pwm_latch) multicore_fifo_push_blocking(UI_SIG_PWM_READ_ISNS);
            if (ui == 'k' && ict_latch) {
                multicore_fifo_push_blocking(UI_SIG_CURRENT_CONTROLLER_PAUSE_UNPAUSE);
            }
            if (ui == 'g') {
                multicore_fifo_push_blocking(UI_SIG_NA_START_STOP);
            }
            if (ui == 'c') {
                //multicore_fifo_push_blocking(0xDEAD);
            }
            if (ui == 't') {
                printf("Temp: %.3f\tADC: %d\tVtc_mv: %d\n", T_glob / 8.0, TSNS_ADC_value_12_bit_avg[0], VTCMV_glob);
            }
        }
        else if (multicore_fifo_rvalid()) {
            int mcdlsig = multicore_fifo_pop_blocking();
            if ((mcdlsig & MC_DL_FLAG_MASK )== MC_DL_TRIG) {
                if((datalogging_waddr&0xFFF) == 0) {
                    printf("clearing sector...");
                    W25_Clear_Sector_Blocking(datalogging_waddr);
                    printf("\t\tclear done (status %d)\n",W25_Read_Status_1());
                }

                datalogger_q_r = mcdlsig & MC_DL_BUFF_I_MASK;
                printf("cursor at %d\n",datalogger_q_r);

                printf("writing ISNS data [0x%08X]...",datalogging_waddr);
                W25_Program_Page_Blocking(datalogging_waddr,(uint8_t *)&ISNS_ADC_value_12_bit_avg[datalogger_q_r],128);
                printf("\t\twrite done (status %d)\n",W25_Read_Status_1());
                datalogging_waddr += 128;


                printf("writing TSNS data [0x%08X]...",datalogging_waddr);
                W25_Program_Page_Blocking(datalogging_waddr,(uint8_t *)&TSNS_ADC_value_12_bit_avg[datalogger_q_r],128);
                printf("\t\twrite done (status %d)\n",W25_Read_Status_1());
                datalogging_waddr += 128;
            }
        }
    }

    reboot:
    printf("REBOOT!\n");
    reset_usb_boot(0,0);
    return 0;
}