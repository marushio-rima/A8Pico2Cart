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
 *
 * Needs to be a release NOT debug build for the cartridge emulation to work
 */

#ifndef __FATFS_DISK_H__
#define __FATFS_DISK_H__

#include "flash_fs.h"

#define SECTOR_NUM 30716 //2044 //1800
#define SECTOR_SIZE 512

void create_fatfs_disk();
bool mount_fatfs_disk();
bool fatfs_is_mounted();
uint32_t fatfs_disk_read(uint8_t* buff, uint32_t sector, uint32_t count);
uint32_t fatfs_disk_write(const uint8_t* buff, uint32_t sector, uint32_t count);
void fatfs_disk_sync();

#endif
