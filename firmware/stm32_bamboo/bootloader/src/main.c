/*
 * Bootloader IAP resident - STM32F103RCT6 (Bambou 4WD).
 *
 * Vit dans les 16 premiers KB de la Flash (0x08000000). A chaque reset il
 * s'execute AVANT l'application (0x08004000) et decide :
 *   - flag partage arme (l'app a recu 0xA3)  -> mode IAP (reception UART)
 *   - application invalide (flash rate/vide)  -> mode IAP (securite anti-brique)
 *   - sinon                                    -> saut vers l'application
 *
 * En mode IAP il parle le protocole de lib/IAP/iap_protocol.h sur USART1
 * (PA9/PA10, 115200 8N1) et ecrit la Flash de l'application lui-meme.
 *
 * Autonome : accede aux registres directement (pas de FWlib / FreeRTOS) pour
 * tenir sous 16 KB. Tout en scrutation (polling), aucune interruption activee
 * -> PRIMASK reste a 0, etat propre pour le saut applicatif.
 */
#include "stm32f10x.h"
#include "iap_protocol.h"

#define IAP_BAUD          (115200u)
#define APB2_CLOCK        (72000000u)   /* PCLK2 apres SystemInit (HSE 8MHz x9) */

/* Cles FPEC (non definies dans stm32f10x.h). */
#define FLASH_KEY1        (0x45670123u)
#define FLASH_KEY2        (0xCDEF89ABu)

/* ------------------------------------------------------------------ LED PC13 */
static void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    /* PC13 : sortie push-pull 2 MHz -> MODE=10, CNF=00 (0x2) sur CRH[7:4] */
    GPIOC->CRH &= ~(0xFu << ((13 - 8) * 4));
    GPIOC->CRH |=  (0x2u << ((13 - 8) * 4));
}
static inline void led_toggle(void) { GPIOC->ODR ^= (1u << 13); }

/* ----------------------------------------------------------- Base de temps ms */
/* SysTick libre sur HCLK ; on compte les passages a zero (COUNTFLAG). */
static void tick_start_1ms(void)
{
    SysTick->LOAD = (APB2_CLOCK / 1000u) - 1u;   /* HCLK = 72 MHz */
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

/* ------------------------------------------------------------------- USART1 */
static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    /* PA9 (TX)  : AF push-pull 50 MHz -> MODE=11, CNF=10 = 0xB sur CRH[7:4]   */
    GPIOA->CRH &= ~(0xFu << ((9 - 8) * 4));
    GPIOA->CRH |=  (0xBu << ((9 - 8) * 4));
    /* PA10 (RX) : entree flottante -> MODE=00, CNF=01 = 0x4 sur CRH[11:8]     */
    GPIOA->CRH &= ~(0xFu << ((10 - 8) * 4));
    GPIOA->CRH |=  (0x4u << ((10 - 8) * 4));

    USART1->BRR = (APB2_CLOCK + IAP_BAUD / 2u) / IAP_BAUD;   /* arrondi */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void uart_put(uint8_t b)
{
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = b;
}

/* Recoit un octet, ou -1 apres timeout_ms. Utilise le SysTick lance ci-dessus. */
static int uart_get(uint32_t timeout_ms)
{
    tick_start_1ms();
    uint32_t elapsed = 0;
    for (;;)
    {
        if (USART1->SR & USART_SR_RXNE)
            return (int)(USART1->DR & 0xFFu);
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            if (++elapsed >= timeout_ms)
                return -1;
        }
    }
}

/* --------------------------------------------------------------- CRC32 (zlib) */
/* Identique a zlib.crc32 / Python : init 0, poly reflechi 0xEDB88320. */
static uint32_t crc32_buf(uint32_t crc, const uint8_t *data, uint32_t len)
{
    crc = ~crc;
    while (len--)
    {
        crc ^= *data++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return ~crc;
}

/* ----------------------------------------------------------------- Flash FPEC */
static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK)
    {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}
static inline void flash_wait(void) { while (FLASH->SR & FLASH_SR_BSY) { } }
static inline void flash_clear_err(void)
{
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
}

static int flash_erase_page(uint32_t addr)
{
    flash_wait();
    flash_clear_err();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = addr;
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait();
    FLASH->CR &= ~FLASH_CR_PER;
    return (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) ? -1 : 0;
}

static int flash_program_hw(uint32_t addr, uint16_t hw)
{
    flash_wait();
    flash_clear_err();
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint16_t *)addr = hw;
    flash_wait();
    FLASH->CR &= ~FLASH_CR_PG;
    if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR))
        return -1;
    return (*(volatile uint16_t *)addr == hw) ? 0 : -1;
}

/* Efface les pages de l'application couvrant nbytes (borne a la zone app). */
static int flash_erase_app(uint32_t nbytes)
{
    if (nbytes == 0 || nbytes > IAP_APP_SIZE)
        nbytes = IAP_APP_SIZE;
    uint32_t pages = (nbytes + IAP_PAGE_SIZE - 1u) / IAP_PAGE_SIZE;
    for (uint32_t i = 0; i < pages; i++)
    {
        if (flash_erase_page(IAP_APP_BASE + i * IAP_PAGE_SIZE) != 0)
            return -1;
        led_toggle();
    }
    return 0;
}

/* --------------------------------------------------------- Saut application */
static int app_valid(void)
{
    uint32_t sp = *(volatile uint32_t *)IAP_APP_BASE;
    uint32_t pc = *(volatile uint32_t *)(IAP_APP_BASE + 4u);
    /* SP doit pointer dans la SRAM (0x2000_xxxx) et le reset handler dans la
     * zone applicative. Flash vierge => 0xFFFFFFFF => invalide. */
    if ((sp & 0xFFFE0000u) != 0x20000000u)
        return 0;
    if (pc < IAP_APP_BASE || pc >= IAP_FLASH_END)
        return 0;
    return 1;
}

static void jump_to_app(void)
{
    uint32_t sp = *(volatile uint32_t *)IAP_APP_BASE;
    uint32_t pc = *(volatile uint32_t *)(IAP_APP_BASE + 4u);

    /* Rendre le peripherique muet et neutre avant de passer la main. */
    USART1->CR1 = 0;
    SysTick->CTRL = 0;
    SCB->VTOR = IAP_APP_BASE;    /* l'app le refera dans SystemInit, mais on est propre */
    __DSB();
    __ISB();

    /* msr msp puis bx sans toucher la pile entre les deux. PRIMASK reste a 0
     * (aucune IRQ activee ici) : etat identique a un reset pour l'application. */
    __asm volatile(
        "msr msp, %0\n\t"
        "bx  %1\n\t"
        :
        : "r"(sp), "r"(pc));
    while (1) { }
}

/* ------------------------------------------------------------- Protocole IAP */
static uint8_t s_frame[IAP_MAX_PAYLOAD + 8];   /* CMD+LEN+payload (sans SOF/CRC) */

/* Recoit une trame complete et validee. Retourne la longueur de s_frame
 * (>=3 : CMD,LEN_L,LEN_H,payload...) ou -1 (timeout / CRC / overflow).
 * Resync glissante sur SOF0/SOF1 pour tolerer des octets parasites. */
static int frame_recv(uint32_t timeout_ms)
{
    int c;
    /* --- SOF 0xAA 0x55 --- */
    for (;;)
    {
        c = uart_get(timeout_ms);
        if (c < 0) return -1;
        if (c != IAP_SOF0) continue;
        c = uart_get(timeout_ms);
        if (c < 0) return -1;
        if (c == IAP_SOF1) break;
        if (c == IAP_SOF0) continue;   /* 0xAA 0xAA .. : re-tente sur le 2e */
    }

    /* --- CMD, LEN --- */
    int cmd = uart_get(timeout_ms); if (cmd < 0) return -1;
    int lenl = uart_get(timeout_ms); if (lenl < 0) return -1;
    int lenh = uart_get(timeout_ms); if (lenh < 0) return -1;
    uint32_t len = (uint32_t)lenl | ((uint32_t)lenh << 8);
    if (len > IAP_MAX_PAYLOAD) return -1;

    s_frame[0] = (uint8_t)cmd;
    s_frame[1] = (uint8_t)lenl;
    s_frame[2] = (uint8_t)lenh;
    for (uint32_t i = 0; i < len; i++)
    {
        c = uart_get(timeout_ms);
        if (c < 0) return -1;
        s_frame[3 + i] = (uint8_t)c;
    }

    /* --- CRC32 (LE) sur CMD+LEN+payload --- */
    uint32_t rx_crc = 0;
    for (int i = 0; i < 4; i++)
    {
        c = uart_get(timeout_ms);
        if (c < 0) return -1;
        rx_crc |= (uint32_t)c << (8 * i);
    }
    uint32_t calc = crc32_buf(0, s_frame, 3 + len);
    if (calc != rx_crc) return -1;

    return (int)(3 + len);
}

static inline uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void reply_hello(void)
{
    uart_put(IAP_ACK);
    uart_put(IAP_BL_VER_MAJOR);
    uart_put(IAP_BL_VER_MINOR);
    uint32_t v = IAP_APP_BASE;   for (int i = 0; i < 4; i++) uart_put((v >> (8 * i)) & 0xFF);
    v = IAP_FLASH_END;           for (int i = 0; i < 4; i++) uart_put((v >> (8 * i)) & 0xFF);
    uint16_t ps = IAP_PAGE_SIZE; uart_put(ps & 0xFF); uart_put((ps >> 8) & 0xFF);
}

/* Boucle IAP : ne rend jamais la main sauf sur GO (saut app). */
static void iap_loop(void)
{
    flash_unlock();
    for (;;)
    {
        int n = frame_recv(3000);
        if (n < 0) { continue; }      /* mode IAP colle : on attend une trame valide */

        uint8_t cmd = s_frame[0];
        uint32_t len = (uint32_t)n - 3u;
        const uint8_t *pl = &s_frame[3];
        led_toggle();

        switch (cmd)
        {
        case IAP_CMD_HELLO:
            reply_hello();
            break;

        case IAP_CMD_ERASE:
            if (len >= 4 && flash_erase_app(rd_u32(pl)) == 0) uart_put(IAP_ACK);
            else                                              uart_put(IAP_NACK);
            break;

        case IAP_CMD_WRITE:
        {
            if (len < 5) { uart_put(IAP_NACK); break; }
            uint32_t addr = rd_u32(pl);
            const uint8_t *d = pl + 4;
            uint32_t dlen = len - 4u;
            if (addr < IAP_APP_BASE || (addr + dlen) > IAP_FLASH_END || (addr & 1u))
            { uart_put(IAP_NACK); break; }
            int ok = 0;
            for (uint32_t i = 0; i < dlen; i += 2)
            {
                uint16_t hw = d[i] | ((uint16_t)((i + 1 < dlen) ? d[i + 1] : 0xFF) << 8);
                if (flash_program_hw(addr + i, hw) != 0) { ok = -1; break; }
            }
            uart_put(ok == 0 ? IAP_ACK : IAP_NACK);
            break;
        }

        case IAP_CMD_VERIFY:
        {
            if (len < 8) { uart_put(IAP_NACK); break; }
            uint32_t addr = rd_u32(pl);
            uint32_t vlen = rd_u32(pl + 4);
            if (addr < IAP_APP_BASE || (addr + vlen) > IAP_FLASH_END)
            { uart_put(IAP_NACK); break; }
            uint32_t crc = crc32_buf(0, (const uint8_t *)addr, vlen);
            uart_put(IAP_ACK);
            for (int i = 0; i < 4; i++) uart_put((crc >> (8 * i)) & 0xFF);
            break;
        }

        case IAP_CMD_GO:
            uart_put(IAP_ACK);
            /* Laisser l'ACK partir completement avant de sauter. */
            while (!(USART1->SR & USART_SR_TC)) { }
            flash_wait();
            FLASH->CR |= FLASH_CR_LOCK;
            jump_to_app();
            break;                    /* jamais atteint */

        default:
            uart_put(IAP_NACK);
            break;
        }
    }
}

int main(void)
{
    /* SystemInit() a deja ete appele par le startup (horloge = 72 MHz). */
    uint32_t flag = IAP_FLAG;
    IAP_FLAG = 0;                     /* consomme : un seul passage force */

    led_init();

    int force_iap = (flag == IAP_FLAG_UPDATE) || !app_valid();
    if (!force_iap)
        jump_to_app();                /* cas nominal : demarrage rapide de l'app */

    /* Mode IAP */
    uart_init();
    iap_loop();                       /* ne revient jamais (sauf GO -> saut app) */

    /* Filet : si iap_loop rendait la main, tenter l'app. */
    jump_to_app();
    while (1) { }
}
