/*
 * iap_protocol.h - Contrat PARTAGE entre l'application et le bootloader IAP.
 *
 * Ce header est inclus des DEUX cotes :
 *   - l'application (projet racine)         : declenche l'entree en IAP (0xA3)
 *   - le bootloader (projet bootloader/)    : implemente le protocole
 *
 * Il fige la carte Flash, l'adresse du flag RAM partage (qui survit au
 * NVIC_SystemReset) et le format des trames UART. Toute modif ici doit etre
 * reflashee des deux cotes.
 *
 *  Carte Flash STM32F103RCT6 (256 KB, pages de 2 KB) :
 *    0x08000000 +--------------------------+
 *               |  BOOTLOADER IAP  (16 KB) |  flashe 1x par BOOT0+RESET
 *    0x08004000 +--------------------------+
 *               |  APPLICATION    (240 KB) |  flashee par IAP (sans bouton)
 *    0x08040000 +--------------------------+
 *
 *  RAM 48 KB : les 32 derniers octets sont retires des DEUX linkers
 *  (LENGTH = 48K - 32) et reserves au flag partage ci-dessous.
 */
#ifndef __IAP_PROTOCOL_H__
#define __IAP_PROTOCOL_H__

#include <stdint.h>

/* --- Carte Flash --- */
#define IAP_FLASH_BASE        (0x08000000u)
#define IAP_BOOTLOADER_SIZE   (0x4000u)                              /* 16 KB  */
#define IAP_APP_BASE          (IAP_FLASH_BASE + IAP_BOOTLOADER_SIZE) /* 0x08004000 */
#define IAP_FLASH_SIZE        (256u * 1024u)
#define IAP_FLASH_END         (IAP_FLASH_BASE + IAP_FLASH_SIZE)      /* 0x08040000 */
#define IAP_APP_SIZE          (IAP_FLASH_END - IAP_APP_BASE)         /* 240 KB */
#define IAP_PAGE_SIZE         (2048u)                                /* high-density */

/* --- Flag partage app -> bootloader (RAM haute, hors des deux linkers) ---
 * 48 KB RAM se termine a 0x2000C000 ; on reserve les 32 derniers octets. */
#define IAP_SHARED_BASE       (0x2000BFE0u)
#define IAP_FLAG_ADDR         (IAP_SHARED_BASE)
#define IAP_FLAG              (*(volatile uint32_t *)(IAP_FLAG_ADDR))
#define IAP_FLAG_UPDATE       (0xB00710ADu)   /* "BOOTLOAD" : rester en IAP */

/* --- Protocole UART (USART1, 115200 8N1) ---
 * Trame hote -> bootloader :
 *   [SOF0=0xAA][SOF1=0x55][CMD][LEN_L][LEN_H][payload..][CRC32 x4 (LE)]
 *   CRC32 (zlib/IEEE) calcule sur CMD+LEN+payload.
 * Reponse bootloader -> hote : IAP_ACK ou IAP_NACK, suivi eventuellement
 * de donnees (HELLO, VERIFY).
 */
#define IAP_SOF0              (0xAAu)
#define IAP_SOF1              (0x55u)
#define IAP_ACK               (0x79u)
#define IAP_NACK              (0x1Fu)

#define IAP_MAX_PAYLOAD       (1024u)   /* WRITE : 1 KB de donnees max par trame */

/* Commandes */
#define IAP_CMD_HELLO         (0x01u)   /* -> ACK + info geometrie (voir ci-dessous) */
#define IAP_CMD_ERASE         (0x02u)   /* payload [nbytes u32] -> efface l'app, ACK */
#define IAP_CMD_WRITE         (0x03u)   /* payload [addr u32][data..] -> ACK */
#define IAP_CMD_VERIFY        (0x04u)   /* payload [addr u32][len u32] -> ACK + crc32 u32 */
#define IAP_CMD_GO            (0x05u)   /* -> ACK puis saut vers l'application */

/* Reponse a HELLO (apres IAP_ACK), tout en little-endian :
 *   [bl_ver_major u8][bl_ver_minor u8][app_base u32][flash_end u32][page_size u16] */
#define IAP_BL_VER_MAJOR      (1u)
#define IAP_BL_VER_MINOR      (0u)

#endif /* __IAP_PROTOCOL_H__ */
