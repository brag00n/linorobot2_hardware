#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""beep_battery_test.py - Test SANS moteur : tension batterie + bip.

But : isoler si l'instabilite (port qui meurt, carte qui plante) est liee au
BRUIT/COURANT moteur. Ici AUCUN moteur : on lit la tension batterie, on envoie
UN bip (300 ms, auto-terminant, prouve la reception d'une commande sans bruit
moteur), puis on verifie que les trames continuent d'arriver (=> port survit).

Methode serie fiable : un handle, timeout=0.1, write_timeout=2.0, pas de flush,
pas de DTR/RTS, une seule ecriture, fermeture naturelle (pas d'os._exit).

Usage : python tools/beep_battery_test.py [COMx]
"""
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_monitor import FrameParser, build_frame, decode_speed

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200
FUNC_BEEP = 0x02


def read_window(s, p, duration):
    """Lit `duration` s. Renvoie (nb_frames_ok, derniere_tension_ou_None)."""
    n = 0
    volt = None
    t0 = time.time()
    while time.time() - t0 < duration:
        for func, data, ok, raw in p.feed(s.read(256)):
            if not ok:
                continue
            n += 1
            if func == 0x0A and len(data) >= 7:
                volt = decode_speed(data)["Batterie (V)"]
    return n, volt


def main():
    beep = build_frame(FUNC_BEEP, bytes([0x2C, 0x01]))  # 300 ms
    print("Trame bip :", " ".join(f"{b:02X}" for b in beep), flush=True)

    try:
        s = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=2.0)
    except Exception as e:
        sys.exit(f"Ouverture {PORT} impossible : {e!r} (repluger l'USB ?)")

    p = FrameParser()
    n1, volt = read_window(s, p, 1.2)
    print(f"Avant bip : {n1} trames, batterie = {volt} V", flush=True)
    if n1 == 0:
        s.close()
        sys.exit("Carte muette : non alimentee / LED eteinte ?")

    try:
        w = s.write(beep)
        print(f"bip envoye ({w} o) -> tu dois entendre un bip de ~0,3 s.", flush=True)
    except Exception as e:
        s.close()
        sys.exit(f"ECHEC write bip : {e!r}")

    n2, volt2 = read_window(s, p, 1.5)
    s.close()

    print(f"Apres bip : {n2} trames, batterie = {volt2} V", flush=True)
    print("\n=== VERDICT ===", flush=True)
    if n2 > 0:
        print("Le port a SURVECU a la commande (trames toujours recues).", flush=True)
        print("=> la couche comms est stable SANS moteur. Si un test moteur, lui,", flush=True)
        print("   tue le port, l'instabilite vient bien du moteur (bruit/brownout).", flush=True)
    else:
        print("Plus aucune trame apres le bip : la carte/port a lache MEME sans moteur", flush=True)
        print("   => instabilite comms/alim independante du moteur.", flush=True)


if __name__ == "__main__":
    main()
