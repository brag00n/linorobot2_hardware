#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
phased_drive.py - Test moteur+encodeur en PHASES a handles serie separes.

Contrainte observee sur ce CH340/Windows : une ECRITURE apres des LECTURES sur
le meme handle fige le port ("device not functioning"). Une ecriture sur un
handle NEUF, sans lecture prealable, passe. Le firmware ne repete pas / ne coupe
pas la commande FUNC_MOTOR (g_start_ctrl=0 => pas d'ecrasement, pas de timeout) :
UNE seule ecriture suffit a lancer le moteur, une autre a l'arreter.

Phases :
  1) handle A : lecture baseline encodeur, puis fermeture
  2) handle B : NEUF, aucune lecture -> ecrit DRIVE, attend, ecrit STOP, ferme
  3) handle C : lecture comptage final

SECURITE : roues en l'air. Le firmware n'ayant pas de timeout, si le STOP
echouait le moteur resterait alimente -> COUPER l'alim. PWM et duree bornes.

Usage : python tools/phased_drive.py --motor 2 --pwm 25 --seconds 1.0
"""
import argparse
import os
import sys
import threading
import time

import serial

from ros_monitor import FrameParser, build_frame, decode_encoder

FUNC_MOTOR = 0x10
PORT = "COM4"
BAUD = 115200


def read_counts(duration):
    """Ouvre un handle, lit `duration` s, renvoie le dernier [M1..M4], ferme."""
    s = serial.Serial(PORT, BAUD, timeout=0.1)
    p = FrameParser()
    last = None
    t = time.time()
    while time.time() - t < duration:
        for func, data, ok, raw in p.feed(s.read(256)):
            if ok and func == 0x0D and len(data) >= 16:
                d = decode_encoder(data)
                last = [int(round(d[f"M{i + 1}"])) for i in range(4)]
    s.close()
    return last


def drive_phase(drive, stop, seconds):
    """Handle NEUF, AUCUNE lecture : ecrit drive, attend, ecrit stop. Ferme."""
    s = serial.Serial(PORT, BAUD, timeout=0.1)     # write bloquant
    try:
        s.write(drive)                             # 1 seule ecriture -> demarre
        s.flush()
        time.sleep(seconds)                        # le moteur tourne (PWM maintenue)
    finally:
        for _ in range(3):                         # STOP, espacees (pas de rafale)
            s.write(stop)
            s.flush()
            time.sleep(0.1)
        s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", type=int, default=2, choices=[1, 2, 3, 4])
    ap.add_argument("--pwm", type=int, default=25)
    ap.add_argument("--seconds", type=float, default=1.0)
    args = ap.parse_args()
    motor = args.motor
    pwm = max(-50, min(50, args.pwm))
    seconds = max(0.1, min(2.0, args.seconds))

    # chien de garde : sortie forcee sans ecrire (jamais de deadlock)
    def watchdog():
        time.sleep(seconds + 8.0)
        print("WATCHDOG: sortie forcee (COUPER l'alim si une roue tourne encore)",
              flush=True)
        os._exit(2)
    threading.Thread(target=watchdog, daemon=True).start()

    print(f"Phase 1 : lecture baseline...", flush=True)
    base = read_counts(0.9)
    if base is None:
        sys.exit("Aucune trame encodeur : carte muette ?")
    print(f"  baseline M1={base[0]} M2={base[1]} M3={base[2]} M4={base[3]}", flush=True)

    vals = [0, 0, 0, 0]
    vals[motor - 1] = pwm
    drive = build_frame(FUNC_MOTOR, bytes((v & 0xFF) for v in vals))
    stop = build_frame(FUNC_MOTOR, bytes([0, 0, 0, 0]))

    print(f"Phase 2 : M{motor} a {pwm}% pendant {seconds:g}s (handle neuf, write-only)...",
          flush=True)
    drive_phase(drive, stop, seconds)
    print("  drive+stop envoyes.", flush=True)

    print("Phase 3 : lecture comptage final...", flush=True)
    final = read_counts(0.9) or base
    print(f"  final    M1={final[0]} M2={final[1]} M3={final[2]} M4={final[3]}", flush=True)

    print("\n=== RESULTAT ===", flush=True)
    for i in range(4):
        dl = final[i] - base[i]
        tag = "   <-- pilote" if i == motor - 1 else ""
        print(f"  M{i + 1} : delta {dl:+5d}{tag}", flush=True)
    dd = final[motor - 1] - base[motor - 1]
    print(flush=True)
    if abs(dd) > 20:
        print(f"OK : M{motor} a compte {dd:+d} tics sous tension -> ENCODEUR OK.",
              flush=True)
    else:
        print(f"PROBLEME : M{motor} n'a quasi rien compte ({dd:+d}). "
              f"La roue a-t-elle tourne ? sinon defaut chaine encodeur.", flush=True)


if __name__ == "__main__":
    main()
