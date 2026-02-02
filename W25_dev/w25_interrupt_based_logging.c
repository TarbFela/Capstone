#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "../src2/capstone_w25.h"

#define FS_BASE_ADDR 0x1000
#define FS_INITED_FLAG 0xAB
#define FS_FINFOS_OFFSET 1
#define FS_FINFO_SIZE 8
#define FS_MAX_NUMBER_FILES 256

#define W25_PAGE_SIZE 256
#define W25_SECTOR_SIZE 4096

/* onboard structure:
 * little endian.
 * FS_BASE_ADDR has a one-byte initialized flag.
 * at the base address plus FS_FINFOS_OFFSET lies the files' information:
 * Following this are the finfo structs, which are 8 bytes (FINFO_SIZE):
 *      24-bit address (in a 32-bit slot)
 *      16-bit size (in a 32-bit slot)
 *
 * There is no "number of files" indicator because I don't want to erase the sector. The "number of files"
 * must be found out by iterating until an 0xFF is reached
 *
 * Files should start a whole sector after the filesystem
 */


// file info struct (address and size)
typedef struct {
    uint32_t addr;
    uint32_t size;
} ulfs_finfo_t;

typedef union {
    uint8_t rdata[8];
    ulfs_finfo_t finfo;
} w25_ulfs_buff_t;

// file system (not really) (number of files, array of finfos)
typedef struct {
    uint8_t n_files;
    ulfs_finfo_t *finfos;
} ulfs_t;


#define DATALOGGING_BASE_ADDR_TEMPORARY 0xC000
int gpiojunk;
int slicejunk;
#define PWMTOOL_GPIO_PIN 7
#define DATALOGGER_BUFF_SIZE 128
uint32_t datalogger_pages_i = 0;



uint8_t *datalogger_buff;
volatile uint8_t *datalogger_r, *datalogger_w;


void datalogger_irq(void) {
    static int i = 0;
    //if(((i+1)&0xF) == 0) printf("\n");
    //queue_try_add(datalogger_buff_queue,&(i));
    *datalogger_w = (uint8_t)i;
    if((++datalogger_w - datalogger_buff) >= DATALOGGER_BUFF_SIZE) datalogger_w = datalogger_buff;
    i++;

    pwm_clear_irq(slicejunk);
}

int main() {
    stdio_init_all();
    W25_Init();

    uint8_t status = W25_Read_Status_1();
    char ui[256];
    scanf("%c",&ui[0]);
    char data[256];
    printf("Read status: 0x%X\n",status);

    W25_Read_Data(FS_BASE_ADDR,(uint8_t *)data,1);
    if(data[0] != (char)FS_INITED_FLAG) {
        printf("NO FILESYSTEM ON FLASH\n");
        printf("clearing sector... ");
        W25_Clear_Sector_Blocking(FS_BASE_ADDR); printf("done.\n");
        data[0] = FS_INITED_FLAG;
        printf("writing init flag... ");
        W25_Program_Page_Blocking(FS_BASE_ADDR, (uint8_t *)data,1); printf("done.\n");
        printf("checking init flag... ");
        data[0] = 0;
        W25_Read_Data(FS_BASE_ADDR,(uint8_t *)data,1);
        switch (data[0]) {
            case 0:
                printf("no data read.\n"); goto reboot;
                break;
            case 0xff:
                printf("data not written?\n"); goto reboot;
                break;
            case FS_INITED_FLAG:
                printf("successfully initialized fs\n");
                break;
            default:
                printf("bad code.\n"); goto reboot;
        };
    }
    else {
        printf("FILESYSTEM FOUND ON FLASH\n");
    }

    ulfs_t my_fs;
    memset(&my_fs, 0, sizeof(my_fs));

    printf("Clearing datalogging sector...");
    W25_Clear_Sector_Blocking(DATALOGGING_BASE_ADDR_TEMPORARY);
    printf("\t\tcleared.\n");
    sleep_ms(100);

    datalogger_buff = (uint8_t *)malloc(sizeof(uint8_t) * DATALOGGER_BUFF_SIZE);
    datalogger_r = datalogger_buff;
    datalogger_w = datalogger_buff;

    gpiojunk = PWMTOOL_GPIO_PIN;
    slicejunk = pwm_gpio_to_slice_num(gpiojunk);
    pwm_config config = pwm_get_default_config();
    // 125MHz / 256 = 488.28kHz,
    pwm_config_set_clkdiv_int(&config, 256);
    // about 14.9Hz
    config.top = 32767;
    irq_set_exclusive_handler(PWM_IRQ_WRAP,datalogger_irq);
    pwm_init(slicejunk, &config, true);
    pwm_set_irq_enabled(slicejunk,true);
    irq_set_enabled(PWM_IRQ_WRAP,true);

    printf("Press h to stop 'datalogging'\n");
    ui[0] = 0;

    while(1) {
        if( stdio_getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) break;
        if(
                ((datalogger_r == datalogger_buff) && (datalogger_w>=(datalogger_buff+DATALOGGER_BUFF_SIZE/2)))
                ||
                ((datalogger_r != datalogger_buff) && (datalogger_w<(datalogger_buff+DATALOGGER_BUFF_SIZE/2)))
                ) {
            printf("write here\t");
            printf("status: %X\t",W25_Read_Status_1());
            W25_Write_Enable();
            printf("status: %X\t",W25_Read_Status_1());
            W25_Program_Page_Blocking(DATALOGGING_BASE_ADDR_TEMPORARY + datalogger_pages_i*256, (uint8_t *)datalogger_r, DATALOGGER_BUFF_SIZE/2);
            printf("write done (%d 0x%X)\n",datalogger_pages_i, DATALOGGING_BASE_ADDR_TEMPORARY + datalogger_pages_i*256);
            datalogger_pages_i++;
            if(datalogger_r == datalogger_buff) datalogger_r+= DATALOGGER_BUFF_SIZE/2;
            else datalogger_r = datalogger_buff;
        }
    }

    pwm_set_enabled(slicejunk,false);
    pwm_set_irq_enabled(slicejunk,true);
    irq_set_enabled(PWM_IRQ_WRAP,true);

    printf("\n\nDatalogging finished. Reading out data.\n");
    uint8_t rx_buff[W25_PAGE_SIZE];
    printf("Number of pages written: %d\n",datalogger_pages_i);
    for(int i = 0; i<datalogger_pages_i; i++) {
        printf("Page %d (0x%08X)\n",i,DATALOGGING_BASE_ADDR_TEMPORARY | (i<<8));
        for(int i =0 ; i<256; i++) rx_buff[i] = 0;
        W25_Read_Data(DATALOGGING_BASE_ADDR_TEMPORARY | (i<<8),rx_buff,W25_PAGE_SIZE);
        for(int i = 0 ; i< (256); i++) {
            if(rx_buff[i]==0xFF) printf("ND ");
            else printf("%05d ",rx_buff[i]);
            if(!((i+1)&0x1F)) printf("\n");
        }
        printf("\n");
    }

    printf("Done. Enter 'q' to quit...\n");
    ui[0] = 0;
    scanf(" %c", ((char *) ui) );
    if(ui[0] == 'q') goto reboot;




reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}