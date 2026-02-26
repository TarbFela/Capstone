#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "pico/multicore.h"

#include "mcp3x6xR_driver/mcp3x6xR.h"

int main() {
    stdio_init_all();
    char ui[64];

    mcp_info_t mcp;
    mcp_spi_init(&mcp, 11,12,13,10);
    printf("MCP STRUCT:\n\tCS %d\n\tMOSI %d\n\tMISO %d\n\tSCK %d\n",mcp.cs,mcp.mosi,mcp.miso,mcp.sck);

    printf("provide a character to continue...\n");
    scanf(" %c",ui);

    uint8_t val, stat;
    for(int i =0; i<4; i++) {
        printf("Reading cfg%d register...\t",i);
        val = 0;
        stat = 0;
        mcp_read_cfgn(&mcp, &val,i);
        printf("Value: 0x%02X \tStatus: %d\n", val, stat);
    }
    // might need some delay  between selecting the clock and readying the ADC
    uint8_t cfg0_val = MCP_CFG0_CLK_SEL_INTERNAL | MCP_CFG0_ADC_MODE_STDBY | MCP_CFG0_VREF_SEL_INTERNAL | MCP_CFG0_NO_PARTIAL_SHUTDOWN;
    uint8_t cfg3_val = MCP_CFG3_CONV_MODE_ONE_SHOT_STDBY;

    printf("Writing cfg regs...");
    mcp_write_cfgn(&mcp,cfg0_val,0);
    mcp_write_cfgn(&mcp,cfg3_val,3);
    printf(" done.\n");

    for(int i =0; i<4; i++) {
        printf("Reading cfg%d register...\t",i);
        val = 0;
        stat = 0;
        mcp_read_cfgn(&mcp, &val,i);
        printf("Value: 0x%02X \tStatus: %d\n", val, stat);
    }

    printf("Writing mux reg...");
    stat = mcp_mux_sel(&mcp,MCP_MUX_VAL_Int_Temp_Diode_P,MCP_MUX_VAL_Int_Temp_Diode_M);
    printf("done. [status 0x%02X]\n",stat);

reads:
    uint16_t adc_val;
    for(int i = 0; i<10; i++) {
        printf("[%d/%d] Attempting ADC read...",i+1,10);
        stat = mcp_single_conversion(&mcp, &adc_val);
        printf("\tval: 0x%04X\tstat: 0x%02X\n",adc_val,stat);
    }
    printf("Done. Enter 'q' to exit. Enter any other character to re-read.\n");
    scanf(" %c",ui);
    if(ui[0]!='q') goto reads;

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}