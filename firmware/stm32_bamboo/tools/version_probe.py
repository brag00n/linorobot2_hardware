#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""version_probe.py - Teste si une ECRITURE PC->carte arrive vraiment.

Principe : on demande la VERSION firmware (FUNC_REQUEST_DATA 0x50, sous-code
FUNC_VERSION 0x51). Cette reponse n'est JAMAIS auto-diffusee : la carte ne
l'envoie QUE si elle a recu notre requete. Donc voir revenir une trame func=0x51
= preuve que l'ecriture est arrivee et a ete traitee (aller-retour complet).
Inoffensif : aucun moteur, fonctionne meme en mode batterie-faible.

Anti-blocage CH340/Windows :
  - write_timeout=2s  -> une ecriture coincee LEVE une exception (pas de gel)
  - pas de flush()    -> flush() bloque salement sur ce CH340
  - UNE seule ecriture, faite juste apres l'ouverture, AVANT toute lecture
    (l'ordre "write puis read" n'est pas celui qui figeait le port)
  - chien de garde : sortie forcee sans jamais reecrire

Usage : python tools/version_probe.py
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
    # requete version : FF FC 05 50 51 00 <ck>
    req = build_frame(FUNC_REQUEST_DATA, bytes([FUNC_VERSION, 0x00]))
    print("Trame requete version :", " ".join(f"{b:02X}" for b in req), flush=True)

    # chien de garde : sortie dure apres 8 s, SANS ecrire (jamais de deadlock)
    def watchdog():
        time.sleep(8.0)
        print("WATCHDOG: sortie forcee", flush=True)
        os._exit(3)
    threading.Thread(target=watchdog, daemon=True).start()

    try:
        s = serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=2.0)
    except Exception as e:
        sys.exit(f"Ouverture {PORT} impossible : {e!r} (replugger l'USB ?)")

    # 1) ECRITURE unique, immediate, avant toute lecture
    try:
        n = s.write(req)
        print(f"write() a rendu {n} octets (pas de gel).", flush=True)
    except serial.SerialTimeoutException:
        s.close()
        sys.exit("ECHEC : write bloquee (timeout 2s) -> le CH340 n'ecoule pas. "
                 "Probleme couche USB/CH340, pas firmware.")
    except Exception as e:
        s.close()
        sys.exit(f"ECHEC write : {e!r}")

    # 2) LECTURE des reponses pendant 2.5 s
    p = FrameParser()
    seen_any = 0
    version = None
    funcs = {}
    t0 = time.time()
    while time.time() - t0 < 2.5:
        chunk = s.read(256)
        for func, data, ok, raw in p.feed(chunk):
            if not ok:
                continue
            seen_any += 1
            funcs[func] = funcs.get(func, 0) + 1
            if func == FUNC_VERSION and len(data) >= 2:
                version = (data[0], data[1])
    s.close()

    print(f"\nTrames recues : {seen_any}", flush=True)
    if funcs:
        print("  par type :", ", ".join(f"0x{f:02X}:{c}" for f, c in sorted(funcs.items())), flush=True)

    print("\n=== VERDICT ===", flush=True)
    if version is not None:
        print(f"*** ECRITURE OK *** la carte a repondu VERSION {version[0]}.{version[1]}.", flush=True)
        print("Le chemin PC->carte fonctionne : on peut commander le robot.", flush=True)
    elif seen_any > 0:
        print("La carte EMET (trames recues) mais N'A PAS repondu a la requete version.", flush=True)
        print("=> l'ecriture part du PC mais n'atteint/ne declenche pas la carte :", flush=True)
        print("   suspecter le cablage TXD(CH340)->RXD(PA10) ou l'etage RX.", flush=True)
    else:
        print("Aucune trame recue : carte muette (non alimentee / LED eteinte ?).", flush=True)


if __name__ == "__main__":
    main()
