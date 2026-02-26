#ifndef MCP3X6XR_H
#define MCP3X6XR_H

#include <stdint.h>

typedef uint8_t mcp_status_t;

typedef struct mcp_info {
    int cs;
    int miso;
    int mosi;
    int sck;
} mcp_info_t;

mcp_status_t mcp_spi_init(mcp_info_t *s, int mosi_pin, int miso_pin, int cs_pin, int sck_pin);

mcp_status_t mcp_read_cfgn(mcp_info_t *s, uint8_t *dst, int cfg_n);

#endif
