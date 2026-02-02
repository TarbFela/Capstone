#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
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

//    data[0] = 0;
//    my_fs.finfos = (ulfs_finfo_t *)malloc(sizeof(ulfs_finfo_t) * FS_MAX_NUMBER_FILES);
//    for(int i = 0; i<W25_SECTOR_SIZE; i+=FS_FINFO_SIZE) {
//        W25_Read_Data(FS_BASE_ADDR+FS_FINFOS_OFFSET+i, (uint8_t *)data, FS_FINFO_SIZE);
//        if(data[0] == 0xFF) break; // you've run out of files
//        my_fs.finfos[my_fs.n_files].addr =
//        my_fs.n_files++;
//    }

    sleep_ms(100);
    uint8_t *rx_buff = (uint8_t *)malloc(sizeof(uint8_t)*256);
    uint8_t *tx_buff = (uint8_t *)malloc(sizeof(uint8_t)*256);
    strcpy((char *)tx_buff,"test");
    uint32_t waddr = 0x11000;

    printf("clearing sector 0x11000...");
    W25_Clear_Sector_Blocking(0x11000U);
    printf("\t\tcleared with status: %d\n",W25_Read_Status_1());
    while(waddr<0x11600) {
        printf("writing msg at 0x%06X...",waddr);
        W25_Program_Page_Blocking(waddr, tx_buff, 4);
        waddr+=8;
        printf("\t\twrite attempt made with status %d\n",W25_Read_Status_1());

        scanf(" %c",ui);
        if(ui[0] == 'r') {
            printf("reading page (0x%X)...\t\t", waddr & (~0xFF));
            for(int i = 0; i<256; i++) rx_buff[i] = 0;
            W25_Read_Data(waddr & (~0xFF), rx_buff, 256);
            printf("read complete.\n");
            for (int i = 0; i < 256; i++) {
                printf("%c", rx_buff[i]);
                if(((i+1)&0x1F)==0) printf("\n");
            }
            printf("\n\n");
            sleep_ms(500);
        }
        else if(ui[0] == 'q') break;
    }

    printf("Done. Provide a character to exit.\n");
    scanf(" %c",ui);

    reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);

    return 0;
}