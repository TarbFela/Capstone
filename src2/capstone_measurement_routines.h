//
// Created by The Tragedy of Darth Wise on 2/3/26.
//

#ifndef CAPSTONE_PWM_CAPSTONE_MEASUREMENT_ROUTINES_H
#define CAPSTONE_PWM_CAPSTONE_MEASUREMENT_ROUTINES_H


void vtc_offset_sweep();

void network_analyzer_slave_adc_handler(void);

// override the CAS with our own ADC to PWM handler.
void network_analyzer_slave_start();

// stops the network analyzer program and re-inits the CAS.
void network_analyzer_slave_stop();

#endif //CAPSTONE_PWM_CAPSTONE_MEASUREMENT_ROUTINES_H
