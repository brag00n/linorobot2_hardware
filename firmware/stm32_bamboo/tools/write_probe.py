#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""write_probe.py - Teste si une ecriture serie BLOQUANTE passe sur COM4.

Ouvre le port comme le serveur MCP (pas de write_timeout => ecriture bloquante),
lance un chien de garde qui force la sortie si une ecriture depasse 3 s
(=> aucun moteur ne reste alimente), ecrit une trame STOP moteur, mesure le temps.
"""
import os
import threading
import time

import serial

STOP = bytes([0xFF, 0xFC, 0x07, 0x10, 0, 0, 0, 0, 0x23])  # FUNC_MOTOR 0,0,0,0

done = threading.Event()


def watchdog():
    if not done.wait(3.0):
        print("WATCHDOG: ecriture bloquee >3s -> kill", flush=True)
        os._exit(2)


def main():
    for i in range(8):
        try:
            s = serial.Serial("COM4", 115200, timeout=0.2)  # write_timeout=None
            break
        except Exception as e:
            print("open try", i, repr(e)[:60], flush=True)
            time.sleep(1)
    else:
        print("ECHEC OUVERTURE", flush=True)
        return
    print("open OK", flush=True)
    threading.Thread(target=watchdog, daemon=True).start()
    t = time.time()
    n = s.write(STOP)
    s.flush()
    dt = time.time() - t
    done.set()
    print(f"write OK : {n} octets en {dt*1000:.0f} ms", flush=True)
    s.close()


if __name__ == "__main__":
    main()
