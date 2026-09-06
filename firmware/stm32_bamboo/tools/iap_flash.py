#!/usr/bin/env python3
"""iap_flash.py - outil hote de mise a jour applicative par UART (IAP).

Parle le protocole du bootloader resident (voir lib/IAP/iap_protocol.h) sur le
port CH340 de la carte Bambou 4WD. Envoie l'image applicative (firmware.bin)
et la fait ecrire par le bootloader a 0x08004000, sans BOOT0 ni RESET.

Sequence :
  1. (option --enter) envoie 0xA3 a l'application -> reset dans le bootloader
  2. HELLO  : poignee de main + geometrie flash (app_base, flash_end, page)
  3. ERASE  : efface juste assez de pages pour l'image
  4. WRITE  : ecrit l'image par blocs de <=IAP_MAX_PAYLOAD-4 octets
  5. VERIFY : compare le CRC32 calcule cote carte a celui de l'image
  6. GO     : verrouille la flash et saute a l'application

Recette d'ecriture CH340 (cf. memoire projet) : pas de flush(), pas de
DTR/RTS, write_timeout court -> le port ne se fige jamais.

Usage :
  python tools/iap_flash.py                       # port/bin par defaut, --enter
  python tools/iap_flash.py --port COM4 --bin xxx.bin
  python tools/iap_flash.py --no-enter            # deja dans le bootloader
"""
import argparse
import os
import sys
import time
import zlib

try:
    import serial
except ImportError:
    sys.exit("pyserial manquant (lancez avec le Python de PlatformIO).")

# ------------------------------------------------- protocole (iap_protocol.h)
SOF0, SOF1 = 0xAA, 0x55
ACK, NACK = 0x79, 0x1F
CMD_HELLO, CMD_ERASE, CMD_WRITE, CMD_VERIFY, CMD_GO = 0x01, 0x02, 0x03, 0x04, 0x05
MAX_PAYLOAD = 1024
APP_BASE_DEFAULT = 0x08004000
FLASH_END_DEFAULT = 0x08040000

# ------------------------------------------------- commande applicative 0xA3
PTO_HEAD, PTO_ID_RX = 0xFF, 0xFC
FUNC_ENTER_BOOTLOADER, SAVE_VERIFY = 0xA3, 0x5F

DEFAULT_BIN = os.path.join(".pio", "build", "genericSTM32F103RC", "firmware.bin")


def log(*a):
    print("[iap]", *a, flush=True)


def app_enter_frame():
    """Trame hote->carte demandant le saut bootloader : [FF FC LEN A3 5F CHK]."""
    body = bytearray([PTO_HEAD, PTO_ID_RX, 3 + 1, FUNC_ENTER_BOOTLOADER, SAVE_VERIFY])
    body.append(sum(body[2:]) & 0xFF)
    return bytes(body)


def frame(cmd, payload=b""):
    """[AA 55][CMD][LEN_L LEN_H][payload][CRC32 LE] ; CRC sur CMD+LEN+payload."""
    body = bytearray([cmd, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    body += payload
    crc = zlib.crc32(body) & 0xFFFFFFFF
    return bytes([SOF0, SOF1]) + bytes(body) + crc.to_bytes(4, "little")


class Link:
    """Port serie + primitives lecture/ecriture non bloquantes (recette CH340)."""

    def __init__(self, port, baud):
        # timeout court en lecture, write_timeout borne : jamais de blocage dur.
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=2)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, data):
        self.ser.write(data)               # pas de flush() : CH340 se fige sinon

    def read_exact(self, n, timeout=3.0):
        """Lit exactement n octets ou None si timeout global depasse."""
        out = bytearray()
        end = time.time() + timeout
        while len(out) < n and time.time() < end:
            chunk = self.ser.read(n - len(out))
            if chunk:
                out += chunk
        return bytes(out) if len(out) == n else None

    def read_ack(self, timeout=3.0):
        """Attend le prochain ACK/NACK en ignorant d'eventuels octets parasites."""
        end = time.time() + timeout
        while time.time() < end:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] == ACK:
                return True
            if b[0] == NACK:
                return False
        return None


def do_hello(link, retries=40):
    """Repete HELLO jusqu'a reponse. Renvoie (app_base, flash_end, page) ou None."""
    f = frame(CMD_HELLO)
    for _ in range(retries):
        link.send(f)
        b = link.ser.read(1)
        if not b or b[0] != ACK:
            time.sleep(0.1)
            continue
        rest = link.read_exact(12, timeout=1.0)     # ver(2)+base(4)+end(4)+page(2)
        if rest is None:
            continue
        vmaj, vmin = rest[0], rest[1]
        app_base = int.from_bytes(rest[2:6], "little")
        flash_end = int.from_bytes(rest[6:10], "little")
        page = int.from_bytes(rest[10:12], "little")
        log(f"bootloader v{vmaj}.{vmin}  app=0x{app_base:08X}  "
            f"end=0x{flash_end:08X}  page={page}")
        return app_base, flash_end, page
    return None


def main():
    ap = argparse.ArgumentParser(description="Flash IAP par UART (bootloader resident).")
    ap.add_argument("--port", default=os.environ.get("BAMBOU_PORT", "COM4"))
    ap.add_argument("--baud", type=int, default=int(os.environ.get("BAMBOU_BAUD", "115200")))
    ap.add_argument("--bin", default=DEFAULT_BIN, help="image applicative a flasher")
    ap.add_argument("--enter", dest="enter", action="store_true", default=True,
                    help="envoie 0xA3 avant de commencer (defaut)")
    ap.add_argument("--no-enter", dest="enter", action="store_false",
                    help="la carte est deja dans le bootloader")
    ap.add_argument("--chunk", type=int, default=MAX_PAYLOAD - 4,
                    help="octets de donnees par trame WRITE")
    args = ap.parse_args()

    if not os.path.isfile(args.bin):
        sys.exit(f"image introuvable : {args.bin} (compilez avec `pio run`)")
    image = open(args.bin, "rb").read()
    if len(image) & 1:                              # flash STM32F1 = demi-mots
        image += b"\xFF"
    log(f"image {args.bin} : {len(image)} octets")

    # 1. faire basculer l'application dans le bootloader (0xA3), puis rouvrir.
    if args.enter:
        log("envoi 0xA3 (saut bootloader)...")
        try:
            pre = Link(args.port, args.baud)
            pre.send(app_enter_frame())
            time.sleep(0.2)
            pre.close()
        except Exception as e:
            log(f"(0xA3 ignore : {e})")
        time.sleep(1.0)                             # reset + reenumeration CH340

    # 2. HELLO
    link = Link(args.port, args.baud)
    try:
        geo = do_hello(link)
        if geo is None:
            sys.exit("pas de reponse HELLO : la carte est-elle dans le bootloader ?")
        app_base, flash_end, page = geo
        if len(image) > (flash_end - app_base):
            sys.exit(f"image trop grande : {len(image)} > {flash_end - app_base}")

        # 3. ERASE (nombre d'octets a effacer)
        log("effacement...")
        if not _cmd_ack(link, CMD_ERASE, len(image).to_bytes(4, "little"), 20.0):
            sys.exit("ERASE refuse (NACK).")

        # 4. WRITE bloc par bloc
        log("ecriture...")
        off = 0
        t0 = time.time()
        while off < len(image):
            piece = image[off:off + args.chunk]
            addr = app_base + off
            payload = addr.to_bytes(4, "little") + piece
            if not _cmd_ack(link, CMD_WRITE, payload, 5.0):
                sys.exit(f"WRITE refuse a 0x{addr:08X} (NACK).")
            off += len(piece)
            pct = 100 * off // len(image)
            print(f"\r  {off}/{len(image)} ({pct}%)", end="", flush=True)
        print()
        log(f"ecrit en {time.time() - t0:.1f}s")

        # 5. VERIFY (CRC32 cote carte vs image)
        log("verification CRC32...")
        payload = app_base.to_bytes(4, "little") + len(image).to_bytes(4, "little")
        link.send(frame(CMD_VERIFY, payload))
        # La carte calcule le CRC sur toute l'image AVANT de repondre : l'ACK peut
        # tarder (~0,2 s pour 160+ Ko). read_ack boucle jusqu'a 5 s.
        if link.read_ack(timeout=5.0) is not True:
            sys.exit("VERIFY refuse (NACK).")
        crc_rx = link.read_exact(4, timeout=3.0)
        if crc_rx is None:
            sys.exit("VERIFY : CRC non recu.")
        crc_board = int.from_bytes(crc_rx, "little")
        crc_host = zlib.crc32(image) & 0xFFFFFFFF
        if crc_board != crc_host:
            sys.exit(f"CRC different : carte=0x{crc_board:08X} hote=0x{crc_host:08X}")
        log(f"CRC OK : 0x{crc_host:08X}")

        # 6. GO
        log("saut application (GO)...")
        if not _cmd_ack(link, CMD_GO, b"", 3.0):
            sys.exit("GO refuse (NACK).")
        log("termine : l'application demarre.")
    finally:
        link.close()


def _cmd_ack(link, cmd, payload, timeout):
    """Envoie une commande et attend ACK(True)/NACK(False)."""
    link.send(frame(cmd, payload))
    r = link.read_ack(timeout=timeout)
    return r is True


if __name__ == "__main__":
    main()
