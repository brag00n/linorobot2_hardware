#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""write_repeat_probe.py - Valide que PLUSIEURS ecritures successives passent.

Critique pour la securite : le firmware n'a pas de timeout moteur (une commande
FUNC_MOTOR tourne jusqu'au STOP). Il faut donc etre certain qu'apres un 1er
write (drive) un 2e write (stop) part de facon fiable. On teste ca sans danger
en envoyant la requete VERSION 3 fois, entrecoupees de lectures, sur UN handle,
avec les reglages qui marchent (write_timeout=2, pas de flush, pas de DTR/RTS).

Usage : python tools/write_repeat_probe.py [COMx]
"""
import os
import sys
import threading
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_monitor import FrameParser, build_frame

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200
FUNC_REQUEST_DATA = 0x50
FUNC_VERSION = 0x51


def main():
    req = build_frame(FUNC_REQUEST_DATA, bytes([FUNC_VERSION, 0x00]))

    def watchdog():
        time.sleep(12.0)
        print("WATCHDOG: sortie forcee", flush=True)
        os._exit(3)
    threading.Thread(target=watchdog, daemon=True).start()

    try:
        s = serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=2.0)
    except Exception as e:
        sys.exit(f"Ouverture {PORT} impossible : {e!r}")

    p = FrameParser()
    versions = 0
    N = 3
    for k in range(N):
        # lecture ~0.6 s (comme un cycle read baseline / read pendant drive)
        t0 = time.time()
        while time.time() - t0 < 0.6:
            for func, data, ok, raw in p.feed(s.read(256)):
                if ok and func == FUNC_VERSION:
                    versions += 1
        # puis ECRITURE (write-after-read : le cas qui figeait avant)
        try:
            n = s.write(req)
            print(f"write #{k + 1} OK ({n} o) apres lecture.", flush=True)
        except serial.SerialTimeoutException:
            s.close()
            sys.exit(f"ECHEC : write #{k + 1} bloquee (timeout). "
                     f"Les ecritures repetees NE sont PAS fiables -> pas de test moteur.")
        except Exception as e:
            s.close()
            sys.exit(f"ECHEC write #{k + 1} : {e!r}")

    # derniere fenetre de lecture pour capter la reponse du dernier write
    t0 = time.time()
    while time.time() - t0 < 0.8:
        for func, data, ok, raw in p.feed(s.read(256)):
            if ok and func == FUNC_VERSION:
                versions += 1
    s.close()

    print(f"\nReponses version recues : {versions} (pour {N} requetes).", flush=True)
    print("\n=== VERDICT ===", flush=True)
    if versions >= N:
        print("*** ECRITURES REPETEES FIABLES *** (write-after-read OK, aucun gel).", flush=True)
        print("On peut piloter+arreter un moteur en securite.", flush=True)
    elif versions > 0:
        print(f"Partiel : {versions}/{N} reponses. Ecritures passent mais des reponses", flush=True)
        print("ont pu etre ratees par le fenetrage. write-after-read ne fige plus.", flush=True)
    else:
        print("Aucune reponse : a investiguer avant tout test moteur.", flush=True)


if __name__ == "__main__":
    main()
