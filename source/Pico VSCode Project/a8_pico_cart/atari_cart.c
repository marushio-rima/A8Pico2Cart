/**
 *    _   ___ ___ _       ___    ___          _   
 *   /_\ ( _ ) _ (_)__ _ |_  )/ __|__ _ _ _| |_ 
 *  / _ \/ _ \  _/ / _/_\/ / | (__/ _` | '_|  _|
 * /_/ \_\___/_| |_\__\_/___| \___\__,_|_|  \__|
 *
 * Atari 8-bit cartridge for WeAct RP2350B
 * Based on A8PicoCart by Robin Edwards 2023
 * Port by marushio-rima 2025
 *
 * Needs to be a release NOT debug build for the cartridge emulation to work
 *
 * WeAct RP2350B GPIO Mapping:
 *   Address bus : GP0 -GP12  -> A0 -A12  (left side, odd positions)
 *   Data bus    : GP28-GP35  -> D0 -D7   (right side)
 *   Control     : GP36=/CCTL, GP37=PHI2, GP38=R/W
 *   Select      : GP39=/S4,  GP40=/S5
 *   Cart present: GP26=RD4,  GP27=RD5   (rear pads)
 *   Power       : VSYS=+5V,  GND=GND
 *   Reset       : RUN pad (rear)
 */

#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/sync.h"

#include "ff.h"
#include "fatfs_disk.h"

// ============================================================
// GPIO Masks — WeAct RP2350B (RP2350 has 64-bit gpio_get_all64)
// ============================================================
#define ADDR_GPIO_MASK      ((uint64_t)0x0000000000001FFF)  // GP0 -GP12
#define DATA_GPIO_MASK      ((uint64_t)0x000000FF00000000)  // GP28-GP35
#define CCTL_GPIO_MASK      ((uint64_t)0x0000001000000000)  // GP36
#define PHI2_GPIO_MASK      ((uint64_t)0x0000002000000000)  // GP37
#define RW_GPIO_MASK        ((uint64_t)0x0000004000000000)  // GP38
#define S4_GPIO_MASK        ((uint64_t)0x0000008000000000)  // GP39
#define S5_GPIO_MASK        ((uint64_t)0x0000010000000000)  // GP40

#define S4_S5_GPIO_MASK     (S4_GPIO_MASK | S5_GPIO_MASK)
#define CCTL_RW_GPIO_MASK   (CCTL_GPIO_MASK | RW_GPIO_MASK)

#define RD4_PIN             26
#define RD5_PIN             27
#define DATA_SHIFT          28  // GP28 = bit 28 in gpio_get_all64()

#define RD4_LOW             gpio_put(RD4_PIN, 0)
#define RD4_HIGH            gpio_put(RD4_PIN, 1)
#define RD5_LOW             gpio_put(RD5_PIN, 0)
#define RD5_HIGH            gpio_put(RD5_PIN, 1)
#define SET_DATA_MODE_OUT   gpio_set_dir_out_masked64(DATA_GPIO_MASK)
#define SET_DATA_MODE_IN    gpio_set_dir_in_masked64(DATA_GPIO_MASK)

#include "rom.h"
#include "osrom.h"

unsigned char cart_ram[128*1024];
unsigned char cart_d5xx[256] = {0};
char errorBuf[40];

#define CART_CMD_OPEN_ITEM          0x00
#define CART_CMD_READ_CUR_DIR       0x01
#define CART_CMD_GET_DIR_ENTRY      0x02
#define CART_CMD_UP_DIR             0x03
#define CART_CMD_ROOT_DIR           0x04
#define CART_CMD_SEARCH             0x05
#define CART_CMD_LOAD_SOFT_OS       0x10
#define CART_CMD_SOFT_OS_CHUNK      0x11
#define CART_CMD_MOUNT_ATR          0x20
#define CART_CMD_READ_ATR_SECTOR    0x21
#define CART_CMD_WRITE_ATR_SECTOR   0x22
#define CART_CMD_ATR_HEADER         0x23
#define CART_CMD_RESET_FLASH        0xF0
#define CART_CMD_NO_CART            0xFE
#define CART_CMD_ACTIVATE_CART      0xFF

#define CART_TYPE_NONE              0
#define CART_TYPE_8K                1
#define CART_TYPE_16K               2
#define CART_TYPE_XEGS_32K          3
#define CART_TYPE_XEGS_64K          4
#define CART_TYPE_XEGS_128K         5
#define CART_TYPE_SW_XEGS_32K       6
#define CART_TYPE_SW_XEGS_64K       7
#define CART_TYPE_SW_XEGS_128K      8
#define CART_TYPE_MEGACART_16K      9
#define CART_TYPE_MEGACART_32K      10
#define CART_TYPE_MEGACART_64K      11
#define CART_TYPE_MEGACART_128K     12
#define CART_TYPE_BOUNTY_BOB        13
#define CART_TYPE_ATARIMAX_1MBIT    14
#define CART_TYPE_WILLIAMS_64K      15
#define CART_TYPE_OSS_16K_TYPE_B    16
#define CART_TYPE_OSS_8K            17
#define CART_TYPE_OSS_16K_034M      18
#define CART_TYPE_OSS_16K_043M      19
#define CART_TYPE_SIC_128K          20
#define CART_TYPE_SDX_64K           21
#define CART_TYPE_SDX_128K          22
#define CART_TYPE_DIAMOND_64K       23
#define CART_TYPE_EXPRESS_64K       24
#define CART_TYPE_BLIZZARD_16K      25
#define CART_TYPE_4K                26
#define CART_TYPE_TURBOSOFT_64K     27
#define CART_TYPE_TURBOSOFT_128K    28
#define CART_TYPE_ATRAX_128K        29
#define CART_TYPE_MICROCALC         30
#define CART_TYPE_2K                31
#define CART_TYPE_PHOENIX_8K        32
#define CART_TYPE_BLIZZARD_4K       33
#define CART_TYPE_ATR               254
#define CART_TYPE_XEX               255

typedef struct {
    char isDir;
    char filename[13];
    char long_filename[32];
    char full_path[210];
} DIR_ENTRY;

int num_dir_entries = 0;

int entry_compare(const void* p1, const void* p2)
{
    DIR_ENTRY* e1 = (DIR_ENTRY*)p1;
    DIR_ENTRY* e2 = (DIR_ENTRY*)p2;
    if (e1->isDir && !e2->isDir) return -1;
    else if (!e1->isDir && e2->isDir) return 1;
    else return strcasecmp(e1->long_filename, e2->long_filename);
}

char *get_filename_ext(char *filename) {
    char *dot = strrchr(filename, '.');
    if(!dot || dot == filename) return "";
    return dot + 1;
}

int is_valid_file(char *filename) {
    char *ext = get_filename_ext(filename);
    if (strcasecmp(ext, "CAR") == 0 || strcasecmp(ext, "ROM") == 0
            || strcasecmp(ext, "XEX") == 0 || strcasecmp(ext, "ATR") == 0)
        return 1;
    return 0;
}

FILINFO fno;
char search_fname[FF_LFN_BUF + 1];

char *stristr(const char *str, const char *strSearch) {
    char *sors, *subs, *res = NULL;
    if ((sors = strdup (str)) != NULL) {
        if ((subs = strdup (strSearch)) != NULL) {
            res = strstr (strlwr (sors), strlwr (subs));
            if (res != NULL)
                res = (char*)str + (res - sors);
            free (subs);
        }
        free (sors);
    }
    return res;
}

int scan_files(char *path, char *search)
{
    FRESULT res;
    DIR dir;
    UINT i;

    res = f_opendir(&dir, path);
    if (res == FR_OK) {
        for (;;) {
            if (num_dir_entries == 255) break;
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
            if (fno.fattrib & (AM_HID | AM_SYS)) continue;
            if (fno.fattrib & AM_DIR) {
                i = strlen(path);
                strcat(path, "/");
                if (fno.altname[0])
                    strcat(path, fno.altname);
                else
                    strcat(path, fno.fname);
                if (strlen(path) >= 210) continue;
                res = scan_files(path, search);
                if (res != FR_OK) break;
                path[i] = 0;
            }
            else if (is_valid_file(fno.fname))
            {
                char *match = stristr(fno.fname, search);
                if (match) {
                    DIR_ENTRY *dst = (DIR_ENTRY *)&cart_ram[0];
                    dst += num_dir_entries;
                    dst->isDir = (match == fno.fname) ? 1 : 0;
                    strncpy(dst->long_filename, fno.fname, 31);
                    dst->long_filename[31] = 0;
                    if (fno.altname[0])
                        strcpy(dst->filename, fno.altname);
                    else {
                        strncpy(dst->filename, fno.fname, 12);
                        dst->filename[12] = 0;
                    }
                    strcpy(dst->full_path, path);
                    num_dir_entries++;
                }
            }
        }
        f_closedir(&dir);
    }
    return res;
}

int search_directory(char *path, char *search) {
    char pathBuf[256];
    strcpy(pathBuf, path);
    num_dir_entries = 0;
    int i;
    FATFS FatFs;
    if (f_mount(&FatFs, "", 1) == FR_OK) {
        if (scan_files(pathBuf, search) == FR_OK) {
            qsort((DIR_ENTRY *)&cart_ram[0], num_dir_entries, sizeof(DIR_ENTRY), entry_compare);
            DIR_ENTRY *dst = (DIR_ENTRY *)&cart_ram[0];
            for (i=0; i<num_dir_entries; i++)
                dst[i].isDir = 0;
            return 1;
        }
    }
    strcpy(errorBuf, "Problem searching flash");
    return 0;
}

int read_directory(char *path) {
    int ret = 0;
    num_dir_entries = 0;
    DIR_ENTRY *dst = (DIR_ENTRY *)&cart_ram[0];

    if (!fatfs_is_mounted())
       mount_fatfs_disk();

    FATFS FatFs;
    if (f_mount(&FatFs, "", 1) == FR_OK) {
        DIR dir;
        if (f_opendir(&dir, path) == FR_OK) {
            while (num_dir_entries < 255) {
                if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == 0)
                    break;
                if (fno.fattrib & (AM_HID | AM_SYS))
                    continue;
                dst->isDir = fno.fattrib & AM_DIR ? 1 : 0;
                if (!dst->isDir)
                    if (!is_valid_file(fno.fname)) continue;
                strncpy(dst->long_filename, fno.fname, 31);
                dst->long_filename[31] = 0;
                if (fno.altname[0])
                    strcpy(dst->filename, fno.altname);
                else {
                    strncpy(dst->filename, fno.fname, 12);
                    dst->filename[12] = 0;
                }
                dst->full_path[0] = 0;
                dst++;
                num_dir_entries++;
            }
            f_closedir(&dir);
        }
        else
            strcpy(errorBuf, "Can't read directory");
        f_mount(0, "", 1);
        qsort((DIR_ENTRY *)&cart_ram[0], num_dir_entries, sizeof(DIR_ENTRY), entry_compare);
        ret = 1;
    }
    else
        strcpy(errorBuf, "Can't read flash memory");
    return ret;
}

/* ATR Handling */
#define ATR_HEADER_SIZE 16
#define ATR_SIGNATURE 0x0296
typedef struct {
  uint16_t signature;
  uint16_t pars;
  uint16_t secSize;
  uint16_t parsHigh;
  uint8_t flags;
  uint16_t protInfo;
  uint8_t unused[5];
} ATRHeader;

typedef struct {
    char path[256];
    ATRHeader atrHeader;
    int filesize;
    FIL fil;
} MountedATR;

MountedATR mountedATRs[1] = {0};
FATFS FatFs;
int doneFatFsInit = 0;

int mount_atr(char *filename) {
    if (!doneFatFsInit) {
        if (f_mount(&FatFs, "", 1) != FR_OK)
            return 1;
        doneFatFsInit = 1;
    }
    MountedATR *mountedATR = &mountedATRs[0];
    if (f_open(&mountedATR->fil, filename, FA_READ|FA_WRITE) != FR_OK)
        return 2;
    UINT br;
    if (f_read(&mountedATR->fil, &mountedATR->atrHeader, ATR_HEADER_SIZE, &br) != FR_OK || br != ATR_HEADER_SIZE ||
            mountedATR->atrHeader.signature != ATR_SIGNATURE) {
        f_close(&mountedATR->fil);
        return 3;
    }
    strcpy(mountedATR->path, filename);
    mountedATR->filesize = f_size(&mountedATR->fil);
    return 0;
}

int read_atr_sector(uint16_t sector, uint8_t page, uint8_t *buf) {
    MountedATR *mountedATR = &mountedATRs[0];
    if (!mountedATR->path[0]) return 1;
    if (sector == 0) return 2;
    int offset = ATR_HEADER_SIZE;
    if (sector <=3)
        offset += (sector - 1) * 128;
    else
        offset += (3 * 128) + ((sector - 4) * mountedATR->atrHeader.secSize) + (page * 128);
    if (offset > (mountedATR->filesize - 128)) {
        memset(buf, 0 , 128);
        return 0;
    }
    UINT br;
    if (f_lseek(&mountedATR->fil, offset) != FR_OK || f_read(&mountedATR->fil, buf, 128, &br) != FR_OK || br != 128)
        return 2;
    return 0;
}

int write_atr_sector(uint16_t sector, uint8_t page, uint8_t *buf) {
    MountedATR *mountedATR = &mountedATRs[0];
    if (!mountedATR->path[0]) return 1;
    if (sector == 0) return 2;
    int offset = ATR_HEADER_SIZE;
    if (sector <=3)
        offset += (sector - 1) * 128;
    else
        offset += (3 * 128) + ((sector - 4) * mountedATR->atrHeader.secSize) + (page * 128);
    if (offset > (mountedATR->filesize - 128))
        return 2;
    UINT bw;
    if (f_lseek(&mountedATR->fil, offset) != FR_OK || f_write(&mountedATR->fil, buf, 128, &bw) != FR_OK || f_sync(&mountedATR->fil) != FR_OK || bw != 128)
        return 2;
    return 0;
}

/* CARTRIDGE/XEX HANDLING */
int load_file(char *filename) {
    FATFS FatFs;
    int cart_type = CART_TYPE_NONE;
    int car_file = 0, xex_file = 0, expectedSize = 0;
    unsigned char carFileHeader[16];
    UINT br, size = 0;

    if (strncasecmp(filename+strlen(filename)-4, ".CAR", 4) == 0) car_file = 1;
    if (strncasecmp(filename+strlen(filename)-4, ".XEX", 4) == 0) xex_file = 1;

    if (f_mount(&FatFs, "", 1) != FR_OK) {
        strcpy(errorBuf, "Can't read flash memory");
        return 0;
    }
    FIL fil;
    if (f_open(&fil, filename, FA_READ) != FR_OK) {
        strcpy(errorBuf, "Can't open file");
        goto cleanup;
    }
    if (car_file) {
        if (f_read(&fil, carFileHeader, 16, &br) != FR_OK || br != 16) {
            strcpy(errorBuf, "Bad CAR file");
            goto closefile;
        }
        int car_type = carFileHeader[7];
        if (car_type == 1)       { cart_type = CART_TYPE_8K; expectedSize = 8192; }
        else if (car_type == 2)  { cart_type = CART_TYPE_16K; expectedSize = 16384; }
        else if (car_type == 3)  { cart_type = CART_TYPE_OSS_16K_034M; expectedSize = 16384; }
        else if (car_type == 8)  { cart_type = CART_TYPE_WILLIAMS_64K; expectedSize = 65536; }
        else if (car_type == 9)  { cart_type = CART_TYPE_EXPRESS_64K; expectedSize = 65536; }
        else if (car_type == 10) { cart_type = CART_TYPE_DIAMOND_64K; expectedSize = 65536; }
        else if (car_type == 11) { cart_type = CART_TYPE_SDX_64K; expectedSize = 65536; }
        else if (car_type == 12) { cart_type = CART_TYPE_XEGS_32K; expectedSize = 32768; }
        else if (car_type == 13) { cart_type = CART_TYPE_XEGS_64K; expectedSize = 65536; }
        else if (car_type == 14) { cart_type = CART_TYPE_XEGS_128K; expectedSize = 131072; }
        else if (car_type == 15) { cart_type = CART_TYPE_OSS_16K_TYPE_B; expectedSize = 16384; }
        else if (car_type == 17) { cart_type = CART_TYPE_ATRAX_128K; expectedSize = 131072; }
        else if (car_type == 18) { cart_type = CART_TYPE_BOUNTY_BOB; expectedSize = 40960; }
        else if (car_type == 22) { cart_type = CART_TYPE_WILLIAMS_64K; expectedSize = 32768; }
        else if (car_type == 26) { cart_type = CART_TYPE_MEGACART_16K; expectedSize = 16384; }
        else if (car_type == 27) { cart_type = CART_TYPE_MEGACART_32K; expectedSize = 32768; }
        else if (car_type == 28) { cart_type = CART_TYPE_MEGACART_64K; expectedSize = 65536; }
        else if (car_type == 29) { cart_type = CART_TYPE_MEGACART_128K; expectedSize = 131072; }
        else if (car_type == 33) { cart_type = CART_TYPE_SW_XEGS_32K; expectedSize = 32768; }
        else if (car_type == 34) { cart_type = CART_TYPE_SW_XEGS_64K; expectedSize = 65536; }
        else if (car_type == 35) { cart_type = CART_TYPE_SW_XEGS_128K; expectedSize = 131072; }
        else if (car_type == 39) { cart_type = CART_TYPE_PHOENIX_8K; expectedSize = 8192; }
        else if (car_type == 40) { cart_type = CART_TYPE_BLIZZARD_16K; expectedSize = 16384; }
        else if (car_type == 41) { cart_type = CART_TYPE_ATARIMAX_1MBIT; expectedSize = 131072; }
        else if (car_type == 43) { cart_type = CART_TYPE_SDX_128K; expectedSize = 131072; }
        else if (car_type == 44) { cart_type = CART_TYPE_OSS_8K; expectedSize = 8192; }
        else if (car_type == 45) { cart_type = CART_TYPE_OSS_16K_043M; expectedSize = 16384; }
        else if (car_type == 46) { cart_type = CART_TYPE_BLIZZARD_4K; expectedSize = 4096; }
        else if (car_type == 50) { cart_type = CART_TYPE_TURBOSOFT_64K; expectedSize = 65536; }
        else if (car_type == 51) { cart_type = CART_TYPE_TURBOSOFT_128K; expectedSize = 131072; }
        else if (car_type == 52) { cart_type = CART_TYPE_MICROCALC; expectedSize = 32768; }
        else if (car_type == 54) { cart_type = CART_TYPE_SIC_128K; expectedSize = 131072; }
        else if (car_type == 57) { cart_type = CART_TYPE_2K; expectedSize = 2048; }
        else if (car_type == 58) { cart_type = CART_TYPE_4K; expectedSize = 4096; }
        else {
            strcpy(errorBuf, "Unsupported CAR type");
            goto closefile;
        }
    }
    strcpy(errorBuf, "Can't read file");
    unsigned char *dst = &cart_ram[0];
    int bytes_to_read = 128 * 1024;
    if (xex_file) {
        dst += 4;
        bytes_to_read -= 4;
    }
    if (f_read(&fil, dst, bytes_to_read, &br) != FR_OK) {
        cart_type = CART_TYPE_NONE;
        goto closefile;
    }
    size += br;
    if (br == bytes_to_read) {
        if (f_read(&fil, carFileHeader, 1, &br) != FR_OK) {
            cart_type = CART_TYPE_NONE;
            goto closefile;
        }
        if (br == 1) {
            strcpy(errorBuf, "Cart file/XEX too big (>128k)");
            cart_type = CART_TYPE_NONE;
            goto closefile;
        }
    }
    if (car_file) {
        if (size != expectedSize) {
            strcpy(errorBuf, "CAR file is wrong size");
            cart_type = CART_TYPE_NONE;
            goto closefile;
        }
    }
    else if (xex_file) {
        cart_type = CART_TYPE_XEX;
        cart_ram[0] = size & 0xFF;
        cart_ram[1] = (size >> 8) & 0xFF;
        cart_ram[2] = (size >> 16) & 0xFF;
        cart_ram[3] = 0;
    }
    else {
        if (size == 8*1024) cart_type = CART_TYPE_8K;
        else if (size == 16*1024) cart_type = CART_TYPE_16K;
        else if (size == 32*1024) cart_type = CART_TYPE_XEGS_32K;
        else if (size == 64*1024) cart_type = CART_TYPE_XEGS_64K;
        else if (size == 128*1024) cart_type = CART_TYPE_XEGS_128K;
        else {
            strcpy(errorBuf, "Unsupported ROM size ");
            cart_type = CART_TYPE_NONE;
            goto closefile;
        }
    }
    if (cart_type == CART_TYPE_4K) {
        memcpy(&cart_ram[4096], &cart_ram[0], 4096);
        memset(&cart_ram[0], 0xFF, 4096);
    }
    if (cart_type == CART_TYPE_2K) {
        memcpy(&cart_ram[6144], &cart_ram[0], 6144);
        memset(&cart_ram[0], 0xFF, 6144);
    }
    if (cart_type == CART_TYPE_BLIZZARD_4K) {
        memcpy(&cart_ram[4096], &cart_ram[0], 4096);
    }
closefile:
    f_close(&fil);
cleanup:
    f_mount(0, "", 1);
    return cart_type;
}

/* ============================================================
 * CARTRIDGE EMULATION FUNCTIONS
 * All use gpio_get_all64() for RP2350B
