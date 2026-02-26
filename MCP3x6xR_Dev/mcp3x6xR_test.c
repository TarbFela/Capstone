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

    printf("\tWRITING to cfg3: 0x1<<4...");
    mcp_write_cfgn(&mcp,MCP_CFG3_DATA_FORMAT_LJ32,3);
    printf("\tdone.\n");
    printf("Reading cfg3 register...\t");
    val = 0;
    stat = 0;
    mcp_read_cfgn(&mcp, &val,3);
    printf("Value: 0x%02X \tStatus: %d\n", val, stat);

    printf("\tWRITING to cfg3: 0x2<<4...");
    mcp_write_cfgn(&mcp,MCP_CFG3_DATA_FORMAT_SNG8,3);
    printf("\tdone.\n");
    printf("Reading cfg3 register...\t");
    val = 0;
    stat = 0;
    mcp_read_cfgn(&mcp, &val,3);
    printf("Value: 0x%02X \tStatus: %d\n", val, stat);

    printf("\tWRITING to cfg3: 0x0...");
    mcp_write_cfgn(&mcp,0x00,3);
    printf("\tdone.\n");
    printf("Reading cfg3 register...\t");
    val = 0;
    stat = 0;
    mcp_read_cfgn(&mcp, &val,3);
    printf("Value: 0x%02X \tStatus: %d\n", val, stat);

    uint16_t adc_val;
    for(int i = 0; i<10; i++) {
        printf("[%d/%d] Attempting ADC read...",i,10);
        stat = mcp_singe_conversion(&mcp, &val);
        printf("Status (post): 0x%02X \tStatus (pre): 0x%02X\n", val, stat);
    }
    printf("Done. Provide a character to exit.\n");
    scanf(" %c",ui);

    reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}