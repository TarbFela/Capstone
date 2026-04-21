# MCP3x6xR Driver for RP2040/RP2350 [WIP]
This is a simple driver for the MCP3x6xR series ADCs that I will employ in my capstone project. They are SPI devices and I am using RP2350-based boards (namely, the Pimoroni PGA2350).

The driver abstracts basic SPI interactions and provides macros for the register addresses, command structure, and register fields for these ADCs. 
There is also a PIO program and corresponding driver to perform triggered reads of the ADC in continuous read mode. 
Functions pass ownership of GPIO pins between RP2350 SPI and PIO machines.

### Annoying, Sneaky Hardware Requirement
This series of ADCs seems to _require_ that the nIRQ pin be pulled to logic high externally. 
Without this step, the **ADC WILL NOT PERFORM CONVERSIONS**.

## Example Code

The `MCP3x6xR_Dev` directory has an example project using this driver. This project is based around a board I've designed called the ADPC;
if you intend to run this program on your own board you will have to define the RP2350 GPIO pins with `#define ADC_1_PIN_<name> <number>` and remove the references
to `ADPC_cfg.h` (you may even replace it with your own `<>_cfg.h`). 

The example code interfaces with a computer over USB; any serial port tool should work.
The device uses an ASCII interface except when collecting data, when it streams raw bytes over USB CDC. 
Providing a `"q"` input puts the RP2350 back in USB flash mode for further programming. 

`script.py` automates this interface. It requires that you know the path of the USB serial device (on MacOS this is `/dev/cu.usbmodem####`).

`graphing.py` graphs the received data. 

## Structure
This is a work in progress. I am still sorting out how best to define the types and how to structure functions, etc.

I perused the datasheet and wrote defines for a bunch of registers, addresses, etc based on information found therein. While some industry-standard headers (such as those from Raspberry Pi in their hardware/regs headers) use a structure wherein masks, LSB & MSB, and some values are provided for register fields, I am instead opting to provide OR-able field values. For example, in order to construct a value to be written to configuration register 0, you would simply:

```
uint8_t reg_val = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_CURRENT_BIAS_SEL_0u0A | MCP_CFG0_ADC_MODE_STDBY;
```

_(In this particular case, some of these values are unnecessary. Some, however, are necessary, as a default value of 0 in those fields would cause a partial shutdown, e.g.)_

### Abstractions

The following functionalities have since been abstracted into higher-level functions (e.g. `mcp_write_regs()`)

### Reading and Writing

Reading and writing to registers requires a corresponding command type value in the command type field of the command byte. Most command types are 6 bits, but with read/write commands the top four bits correspond instead to the address information. For this reason I've structured the macros such that the read and write commands take an argument corresponding to the address of interest. An example command byte to perform a static (non-incrementing) read of configuration register 2 would be constructed like so:

```
uint8_t cmd = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_STAT(MCP_REG_ADDR_CONFIG2);
```

### ADC MUX Input Selection

A similar macro to the one described above is used to select ADC inputs. Since each ADC input (+/-) may select from any of 15 sources and since the corresponding mux field values are the same between the positive and negative inputs, mux selection is done accordingly:

```
MCP_MUX_P_SEL(MCP_MUX_VAL_CH4) | MCP_MUX_N_SEL(MCP_MUX_VAL_CH0)
```

### Function return values

Because these ADCs read out their status while receiving the command byte, all ADC SPI commands are structured to return an `mcp_status_t` (name might change to be more specific). This is a single byte. Another function might be written to parse this byte. Bitfields are defined as `MCP_STAT_<...>`.

### A Note About Error Handling

Some effort has been made to catch loss of communication with the ADC. However, this was not a comprehensive effort by any means, so the behavior of the driver when communication is lost (e.g. if the ADC loses power) is thoroughly undefined!

## PIO

Included here is a PIO program to operate the device in continuous read mode, wherein the nIRQ pin on the ADC is pulled low when a conversion is ready. This triggers the PIO to initiate a SPI read (i.e. the PIO machine sends out 32 clock cycles and shifts the MISO data into a fifo). DMAs are daisy-chained. A globally accessible DMA buffer (`dma_buff`) is provided. It is aligned such that the DMAs should automatically wrap upon filling their respective halves of the buffer. **It is important that the DMAs be provided an interrupt handler to clear interrupts**. See an example below:

```
void dma_irq_handler(void) {
    // clear the correct interrupt
    int culprit_is_a = dma_hw->ints0 & (1u << mpio_1.dma_a);
    if (culprit_is_a) {
        dma_hw->ints0 = 0x1 << (mpio_1.dma_a);
    }
    else {
        dma_hw->ints0 = 0x1 << (mpio_1.dma_b);
    }
}
```

### Start and stop functions

Start and stop functions are provided which _only_ enter or exit PIO mode. These functions **only** perform configuration tasks (e.g. for pins, dmas, pio, and spi) but do not communicate with the ADC. A separate SPI command should be sent to initiate conversions; once the nIRQ is pulled low, data is (likely) ready and the PIO start() function can be called. **KEEP CS ACTIVE AFTER THE SPI READ COMMAND!** (as per the datasheet).

```
    // write ADC mode
    uint8_t tx[2], rx[2];
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_WRITE_INCR(MCP_REG_ADDR_CONFIG0);
    tx[1] = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_ADC_MODE_CONV;
    gpio_put(ADC_1_PIN_CS,0); sleep_us(MCP_SLEEPTIME_US);
    spi_write_read_blocking(spi1, tx, rx, 2);
    sleep_us(MCP_SLEEPTIME_US); gpio_put(ADC_1_PIN_CS,1);
    // prepare to perform static read of ADC register
    tx[0] = MCP_CMD_DEV_ADDR | MCP_CMD_ADC_REG_READ_STAT(MCP_REG_ADDR_ADCDATA);
    // wait for first IRQ pin signal
    while(!gpio_get(ADC_1_PIN_IRQ)) tight_loop_contents();
    gpio_put(ADC_1_PIN_CS,0); sleep_us(MCP_SLEEPTIME_US);
    // perform first read.
    spi_write_read_blocking(spi1, tx, rx, 1);

    mcp_pio_start(&mpio_1);
```
