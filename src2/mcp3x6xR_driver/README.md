# MCP3x6xR Driver for RP2040/RP2350 [WIP]

This is a simple driver for the MCP3x6xR series ADCs that I will employ in my capstone project. They are SPI devices and I am using RP2350-based boards (namely, the Pimoroni PGA2350).

## Structure

This is a work in progress. I am still sorting out how best to define the types and how to structure functions, etc.

I perused the datasheet and wrote defines for a bunch of registers, addresses, etc based on information found therein. While some industry-standard headers (such as those from Raspberry Pi in their hardware/regs headers) use a structure wherein masks, LSB & MSB, and some values are provided for register fields, I am instead opting to provide OR-able field values. For example, in order to construct a value to be written to configuration register 0, you would simply:

```
uint8_t reg_val = MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN | MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_CURRENT_BIAS_SEL_0u0A | MCP_CFG0_ADC_MODE_STDBY;
```

_(In this particular case, some of these values are unnecessary. Some, however, are necessary, as a default value of 0 in those fields would cause a partial shutdown, e.g.)_

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

