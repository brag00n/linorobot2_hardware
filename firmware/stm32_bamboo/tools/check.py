#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""check.py - Verification unitaire de la carte STM32 Bamboo v4.

Test MONO-CARTE (voir tools/README.md : firmware/<carte>/tools/ = validation des
fonctions de CETTE carte ; tools/robot_control/ = prototypes multi-cartes). Valide
la chaine de commande PC <-> STM32 sans faire tourner de moteur (inoffensif, OK
meme batterie faible) :

  1. CONNEXION    : le port serie s'ouvre et une ecriture passe (pas de gel CH340) ;
  2. VERSION      : la carte repond a la requete version (FUNC 0x50/0x51) -> le
                    chemin PC->carte fonctionne (reponse jamais auto-diffusee) ;
  3. AUTO-REPORT  : des trames de mesure spontanees (vitesse/IMU/encodeur) sont vues
                    -> la carte tourne et emet (chemin carte->PC fonctionne).

Reutilise le protocole de ros_monitor.py (build_frame/FrameParser) et la methode
anti-blocage CH340 de version_probe.py (write_timeout, une seule ecriture avant
lecture, chien de garde qui sort sans jamais reecrire).

/!\ Couper le serveur MCP bambou-board avant (il tient le port COM4). Le defaut
ci-dessous est COM5 (enumeration STM32 cote outils) : adapter selon le montage.

Usage :
  python firmware/stm32_bamboo/tools/check.py [PORT]      # defaut COM5
  -> code de sortie 0 si tout PASS, 1 sinon.
"""
import os
import sys
import threading
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_monitor import FrameParser, build_frame  # noqa: E402

BAUD = 115200
FUNC_REQUEST_DATA = 0x50
FUNC_VERSION = 0x51
# trames de mesure auto-diffusees par la carte (preuve qu'elle tourne)
AUTO_REPORT_FUNCS = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E}   # SPEED/MPU/IMU_ATT/ENCODER/ICM
WATCHDOG_S = 10.0


def check(port="COM5", listen_s=2.5, verbose=True):
    """Verifie connexion + version + auto-report de la carte STM32.

    Retourne True si les 3 etapes passent, False sinon. N'ecrit qu'UNE trame
    (requete version), avant toute lecture, pour ne pas figer le CH340.
    """
    def say(*a):
        if verbose:
            print(*a, flush=True)

    results = {"connexion": False, "version": False, "auto_report": False}
    version = None
    seen_any = 0
    funcs = {}

    # chien de garde : sortie dure si un appel serie fige tout (jamais de reecriture)
    def watchdog():
        time.sleep(WATCHDOG_S)
        say("WATCHDOG: sortie forcee (port fige ?)")
        os._exit(3)
    threading.Thread(target=watchdog, daemon=True).start()

    say(f"=== Check STM32 Bamboo v4 sur {port} @ {BAUD} ===")

    # --- 1) CONNEXION : ouverture + une ecriture qui passe -------------------
    req = build_frame(FUNC_REQUEST_DATA, bytes([FUNC_VERSION, 0x00]))
    try:
        s = serial.Serial(port, BAUD, timeout=0.2, write_timeout=2.0)
    except Exception as e:
        say(f"[1] CONNEXION : FAIL - ouverture {port} impossible : {e!r}")
        say("    (port occupe par le MCP bambou-board ? mauvais COM ? USB debranche ?)")
        return _verdict(results, say)
    try:
        n = s.write(req)                      # ecriture unique, immediate, avant lecture
        results["connexion"] = True
        say(f"[1] CONNEXION : PASS - port ouvert, write() a rendu {n} octets.")
    except serial.SerialTimeoutException:
        s.close()
        say("[1] CONNEXION : FAIL - write bloquee (timeout 2s), le CH340 n'ecoule pas.")
        return _verdict(results, say)
    except Exception as e:
        s.close()
        say(f"[1] CONNEXION : FAIL - erreur write : {e!r}")
        return _verdict(results, say)

    # --- 2+3) LECTURE : version (round-trip) + trames auto-report -------------
    p = FrameParser()
    t0 = time.time()
    while time.time() - t0 < listen_s:
        chunk = s.read(256)
        for func, data, ok, raw in p.feed(chunk):
            if not ok:
                continue
            seen_any += 1
            funcs[func] = funcs.get(func, 0) + 1
            if func == FUNC_VERSION and len(data) >= 2:
                version = (data[0], data[1])
    s.close()

    if funcs:
        say("    trames recues :",
            ", ".join(f"0x{f:02X}:{c}" for f, c in sorted(funcs.items())))

    # 2) VERSION microcode : reponse a la requete (jamais auto-diffusee)
    if version is not None:
        results["version"] = True
        say(f"[2] VERSION    : PASS - la carte a repondu VERSION {version[0]}.{version[1]}.")
    else:
        say("[2] VERSION    : FAIL - pas de trame 0x51 (l'ordre PC->carte n'aboutit pas).")

    # 3) AUTO-REPORT : trames de mesure spontanees vues
    reports = sorted(f for f in funcs if f in AUTO_REPORT_FUNCS)
    if reports:
        results["auto_report"] = True
        say("[3] AUTO-REPORT: PASS - trames de mesure vues : "
            + ", ".join(f"0x{f:02X}" for f in reports) + ".")
    elif seen_any > 0:
        say("[3] AUTO-REPORT: FAIL - la carte emet mais aucune trame de mesure "
            "(auto-report desactive ? RESET_STATE ?).")
    else:
        say("[3] AUTO-REPORT: FAIL - aucune trame recue (carte muette / non alimentee ?).")

    return _verdict(results, say)


def _verdict(results, say):
    ok = all(results.values())
    say("=== VERDICT : " + ("PASS" if ok else "FAIL")
        + " (" + ", ".join(f"{k}={'OK' if v else 'KO'}" for k, v in results.items()) + ") ===")
    return ok


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    sys.exit(0 if check(port) else 1)
