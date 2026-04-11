/**
 *    _   ___ ___ _       ___    ___          _   
 *   /_\ ( _ ) _ (_)__ _ |_  )/ __|__ _ _ _| |_ 
 *  / _ \/ _ \  _/ / _/_\/ / | (__/ _` | '_|  _|
 * /_/ \_\___/_| |_\__\_/___| \___\__,_|_|  \__|
 *
 * Atari 8-bit cartridge for WeAct RP2350B
 * Based on A8PicoCart by Robin Edwards 2023
 * Port by marushio-rima 2025
 */

#ifndef __ATARI_CART_H__
#define __ATARI_CART_H__

// WeAct RP2350B: PHI2 remapped from GP22 to GP37
#define ATARI_PHI2_PIN   37

void atari_cart_main();

#endif
