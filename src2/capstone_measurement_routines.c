//
// Created by The Tragedy of Darth Wise on 2/3/26.
//

#include "capstone_measurement_routines.h"


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
