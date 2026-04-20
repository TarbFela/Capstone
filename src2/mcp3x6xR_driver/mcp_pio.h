#ifndef MCP_PIO_H
#define MCP_PIO_H

#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "mcp3x6xR.h"

// size of each DMA's buffer.
#define DMA_BUFF_SIZE 128

// Globally-accessible array for MCP/SIO/DMA data (i.e. readings). Aligned for DMA Ring mode.
extern volatile uint32_t dma_buff[DMA_BUFF_SIZE*2];

typedef struct mcp_pio_t {
    PIO pio;
    uint sm;
    mcp_info_t *mcp_info;
    uint dma_a;
    uint dma_b;
    uint32_t *buff;
} mcp_pio_t;

void mcp_pio_init(mcp_pio_t *s,mcp_info_t *mcp,uint32_t *sample_buff, void (*dma_handler)(void));
void mcp_pio_start(mcp_pio_t *s);
void mcp_pio_stop(mcp_pio_t *s);

#endif
