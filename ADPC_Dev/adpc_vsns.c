#include "adpc_vsns.h"
#include "hardware/gpio.h"
#include "ADPC_cfg.h"

// Initialize gain sel pin to 0.196V/V setting.
void adpc_vsns_init() {
	gpio_init(VSNS_AMP_GAIN_SEL_PIN);
	gpio_put(VSNS_AMP_GAIN_SEL_PIN, VSNS_AMP_GAIN_SEL_0V196);
	gpio_set_dir(VSNS_AMP_GAIN_SEL_PIN, GPIO_OUT);
}

// Set the gain sel to either 0.196V/V (0) or 0.476V/V (1) 
// Will silently fail if a non-0-or-1 value is passed.
// Use the VSNS_AMP_GAIN_SEL_xVxxx macros.
void adpc_vsns_select_gain(int gain_sel) {
	if(gain_sel < 0 || gain_sel > 1) return;
	gpio_put(VSNS_AMP_GAIN_SEL_PIN, gain_sel);
}
