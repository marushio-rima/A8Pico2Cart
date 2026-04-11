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
 * Boot detection: checks PHI2 (GP37) for 100ms
 * If PHI2 is high → plugged into Atari → cartridge emulation mode
 * If PHI2 is low  → powered from USB → USB Mass Storage mode
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "tusb.h"

#include "atari_cart.h"
#include "fatfs_disk.h"

void cdc_task(void);

int main(void)
{
    // Check if plugged into Atari 8-bit
    // by checking for high on PHI2 (GP37 on WeAct RP2350B) for 100ms
    gpio_init(ATARI_PHI2_PIN);
    gpio_set_dir(ATARI_PHI2_PIN, GPIO_IN);
    while (to_ms_since_boot(get_absolute_time()) < 100)
    {
        if (gpio_get(ATARI_PHI2_PIN))
            atari_cart_main();
    }

    // Powered from USB — enter USB Mass Storage mode
    stdio_init_all();
    printf("A8Pico2Cart - USB Mass Storage mode\n");

    // Init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    while (1)
    {
        tud_task();
        cdc_task();
    }

    return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

void tud_mount_cb(void)
{
    printf("Device mounted\n");
    if (!mount_fatfs_disk())
        create_fatfs_disk();
}

void tud_umount_cb(void)
{
    printf("Device unmounted\n");
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
}

void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
    if (tud_cdc_available())
    {
        char buf[64];
        uint32_t count = tud_cdc_read(buf, sizeof(buf));
        (void) count;
        tud_cdc_write(buf, count);
        tud_cdc_write_flush();
    }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void) itf;
    (void) rts;
}

void tud_cdc_rx_cb(uint8_t itf)
{
    (void) itf;
}
