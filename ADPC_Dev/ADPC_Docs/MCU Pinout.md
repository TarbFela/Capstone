# ADPC MCU Pinout (RP2350)

This table lists the pins of the PGA2350, the nets which they are connected to (typically descriptive of devices such as the ADCs, temperature sensors, etc.) and then their appropriate GPIO Function configuration. See the RP2350 Datasheet Section 1.2.3 for reference (replicated in this directory as a CSV). 

## "Notes"
The "Notes" column lists—in addition to some notes—the possibility of some pins to be used as SPI pins.
### SPI Flash Fix
The **bold** entries refer to a plan to use the HB-3B connector (6-pin MPHB-002 connector) as a SPI connector to a W25Q128 flash chip (on a breakout board such as the one produced by Adafruit). This plan requires soldering a wire from an unused PGA2350 pin to an unused HB-B3 pin. This plan may not be necessary if there is adequate storage on the PGA2350's flash chip (also a W25Q128) and if flash writes to this chip do not interfere with other code. This plan would require that the relevant SPI pins only be configured as SPI when the devices on the same spi_n channel are not being used in that way; i.e. when the MCP ADC SPI pins are under "ownership" of the PIO machines which perform data-ready-triggered "SPI" reads. The MCP SPI pins should not attempt to reclaim the SPI pins if their ownership has been transfered to the SPI flash chip. Some construct should be implemented to organize this ownership.

### CS Pins

SPI chip-select pins are configured as GPIO outputs and manually set and reset (with some delay) in code before and after SPI interactions.

### PIO

PIO machines are used to perform triggered SPI-like reads of the two MCP3x62R ADCs. Other ADC communications use the RP2350's SPI blocks. Pin "ownership" is transferred between the two machines according to the `mcp_pio` driver code.

## Pinout

| PIN       | NET                                            | FUNCTION      | NOTES                                                                                                                |
|-----------|------------------------------------------------|:--------------|----------------------------------------------------------------------------------------------------------------------|
| GPIO 0		  | ADC_16_SPI_SDO            		                   | SPI0          |                                                                                                                      |
| GPIO 1		  | ADC_16_MCLK & ADC_24_MCLK 		                   | PWM 0A	       |                                                                                                                      |
| GPIO 2		  | ADC_16_IRQ                		                   | SIO / PIO     |                                                                                                                      |
| GPIO 3		  | ADC_16_SPI_SDI            		                   | SPI0 / PIO	   |                                                                                                                      |
| GPIO 4		  | INAMP_GPIO2_CALIB_BUSY    		                   | SIO           |                                                                                                                      |
| GPIO 5		  | ADC_16_SPI_CS             		                   | SPI0 / SIO    |
| GPIO 6		  | ADC_16_SPI_SCK            		                   | SPI0 / PIO    |                                                                                                                      |
| GPIO 7		  | INAMP_SPI_CS              		                   | SIO           |                                                                                                                      |
| GPIO 8		  | INAMP_GPIO3_FAULT         		                   | SIO           |                                                                                                                      |
| GPIO 9		  | ADC_24_IRQ                		                   | SIO / PIO     |                                                                                                                      |
| GPIO 10		 | TMP_12C_SDA               		                   | I2C1          |                                                                                                                      |
| GPIO 11		 | INAMP_SPI_SDI & ADC_24_SDI     	               | SPI1 / PIO    |                                                                                                                      |
| GPIO 12		 | INAMP_SPI_SDO & ADC_24_SDO     	               | SPI1		        |                                                                                                                      |
| GPIO 13		 | ADC_24_SPI_CS             		                   | SPI1			       |                                                                                                                      |
| GPIO 14		 | INAMP_SPI_SCLK & ADC_24_SCLK    	              | 	SPI1 / PIO		 |                                                                                                                      |
| GPIO 15		 | TMP_12C_SCL               		                   | I2C1			       |                                                                                                                      |
| GPIO 16		 | VSNS_AMP_GAIN_SEL         		                   | SIO		         |                                                                                                                      |
| GPIO 17		 | MPHB_VDET_3               		                   | SIO		         |                                                                                                                      |
| GPIO 18		 | PWM_A_1                   		                   | PWM 1A			     | SPI0 SCK                                                                                                             |
| GPIO 19		 | PWM_C_1                   		                   | PWM 1B			     | SPI0 TX                                                                                                              |
| GPIO 20		 | PWM_B_1                   		                   | PWM 2A			     | SPI0 RX                                                                                                              |
| GPIO 21		 | PWM_D_1                   		                   | PWM 2B			     | SPI0 CSn                                                                                                             |
| GPIO 22		 | PWM_C_2                   		                   | PWM 3A			     | SPI0 SCK                                                                                                             |
| GPIO 23		 | PWM_A_2                   		                   | PWM 3B			     | SPI0 TX                                                                                                              |
| GPIO 24		 | PWM_D_2                   		                   | PWM 4A			     | SPI1 RX                                                                                                              |
| GPIO 25		 | PWM_B_2                   		                   | PWM 4B			     | SPI1 CSn                                                                                                             |
| GPIO 26		 | PWM_B_3                   		                   | PWM 5A			     | **SPI1 SCK** SPI Flash Fix: HB-3 Pin 2 = SPI SCK                                                                     |
| GPIO 27		 | PWM_D_3                   		                   | PWM 5B			     | **SPI TX** SPI Flash Fix: HB-3 Pin 1 = SPI MOSI                                                                      |
| GPIO 28		 | PWM_A_3                   		                   | PWM 6A			     | SPI1 RX                                                                                                              |
| GPIO 29		 | PWM_C_3                   		                   | PWM 6B			     | SPI1 CSn                                                                                                             |
| GPIO 30		 | MPHB_VDET_2               		                   | SIO		         |                                                                                                                      |
| GPIO 31		 | MPHB_VDET_1               		                   | SIO		         |                                                                                                                      |
| GPIO 32		 | MCU_INDICATOR_LED_1       		                   | SIO		         |                                                                                                                      |
| GPIO 33		 | UART_RX                   		                   | UART0		       |                                                                                                                      |
| GPIO 34		 | UART_TX                   		                   | UART0		       |                                                                                                                      |
| GPIO 35		 | PH_EN_A2 / MPHB-002 Detect                  		 | SIO           | SPI0 TX		Open-Drain or Input/Output Low to see 3.3V when MPHB-002 is connected and powered and to PH-disable, resp.  |
| GPIO 36		 | PH_EN_B2 / MPHB-002 Detect                  		 | SIO           | SPI0 RX		Open-Drain or Input/Output Low to see 3.3V when MPHB-002 is connected and powered and to PH-disable, resp.  |
| GPIO 37		 | PH_EN_B1 / MPHB-002 Detect                  		 | SIO           | SPI0 CSn		Open-Drain or Input/Output Low to see 3.3V when MPHB-002 is connected and powered and to PH-disable, resp. |
| GPIO 38		 | PH_EN_A3 / MPHB-002 Detect                  		 | SIO           | SPI0 SCK		Open-Drain or Input/Output Low to see 3.3V when MPHB-002 is connected and powered and to PH-disable, resp. |
| GPIO 39		 | PH_EN_A1 / MPHB-002 Detect                  		 | SIO           | SPI0 TX		Open-Drain or Input/Output Low to see 3.3V when MPHB-002 is connected and powered and to PH-disable, resp.  |
| GPIO 40		 | PH_EN_B3 / MPHB-002 Detect                  		 | SIO           | **SPI1 RX** SPI Flash Fix: HB-3 Pin 6 = SPI MISO                                                                     |
| GPIO 41		 | MCU1_41                   		                   | _none_			     | **SIO CSn** SPI Flash Fix: HB-3 Pin 5 = SPI CSn (_NOTE: REQUIRES PCB WIRE REWORK!!!_)                                |
| GPIO 42		 | 12C_HEADER_SDA            		                   | I2C1			       |                                                                                                                      |
| GPIO 43		 | 12C_HEADER_SCL            		                   | I2C1			       |                                                                                                                      |
| GPIO 44		 | MCU_ADC_4                 		                   | ADC				       | SPI1 RX                                                                                                              |
| GPIO 45		 | MCU_ADC_5                 		                   | ADC				       |                                                                                                                      |
| GPIO 46		 | MCU_ADC_6                 		                   | ADC				       |                                                                                                                      |
| GPIO 47		 | MCU1_47                   		                   | _none_			     |                                                                                                                      |
| 3V3		     | MCU_3V3                   		                   |               |                                                                                                                      |
| 3V3N		    | MCU1_3V3EN                		                   |               |                                                                                                                      |
| ADCVRF		  | MCU1_ADCVREF              		                   |               |                                                                                                                      |
| BOOTSL		  | MCU1_BOOTSEL              		                   |               |                                                                                                                      |
| GND		     | GND                       		                   |               |                                                                                                                      |
| RUN		     | MCU1_RUN                  		                   |               |                                                                                                                      |
| SWCLK		   | MCU1_SWCLK                		                   |               |                                                                                                                      |
| SWDIO		   | MCU1_SWDIO                		                   |               |                                                                                                                      |
| USB+		    | USB_D+                    		                   |               |                                                                                                                      |
| USB-		    | USB_D-                    		                   |               |                                                                                                                      |
| VBUS		    | MCU_VBUS                  		                   |               |                                                                                                                      |
|