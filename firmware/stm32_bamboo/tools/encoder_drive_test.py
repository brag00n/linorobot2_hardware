#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""encoder_drive_test.py - Pilote UN moteur a faible PWM et mesure l'encodeur.

Methode serie fiable (cf memoire serial-ch340-write-recipe) : UN handle,
timeout=0.1, write_timeout=2.0, PAS de flush(), PAS de DTR/RTS, et SURTOUT
PAS de rafale d'ecritures (le martelage sature le CH340 et fige le write) ni
d'os._exit (c'est lui qui met le port en "device not functioning").

Le firmware n'a PAS de timeout moteur : UNE ecriture drive suffit a lancer, la
commande tient jusqu'au STOP. Structure calquee sur les probes qui marchent :
lecture / une ecriture / lecture. Le STOP (writes espacees, entrecoupees de
lectures) est dans un finally toujours execute. Fermeture naturelle du port.

SECURITE : roues EN L'AIR. Si le script se fige malgre tout : COUPER L'ALIM.

Usage : python tools/encoder_drive_test.py --port COM5 --motor 2 --pwm 35 --seconds 1.0
"""
import argparse
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_monitor import FrameParser, build_frame, decode_encoder

FUNC_MOTOR = 0x10
BAUD = 115200


def read_window(s, p, duration, base, peak, state):
    """Lit `duration` s ; met a jour state['last'] et peak (ecart max /base)."""
    t0 = time.time()
    while time.time() - t0 < duration:
        for func, data, ok, raw in p.feed(s.read(128)):
            if ok and func == 0x0D and len(data) >= 16:
                d = decode_encoder(data)
                cur = [int(round(d[f"M{i + 1}"])) for i in range(4)]
                state["last"] = cur
                if base is not None and peak is not None:
                    for i in range(4):
                        if abs(cur[i] - base[i]) > abs(peak[i] - base[i]):
                            peak[i] = cur[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--motor", type=int, default=2, choices=[1, 2, 3, 4])
    ap.add_argument("--pwm", type=int, default=35)
    ap.add_argument("--seconds", type=float, default=1.0)
    args = ap.parse_args()
    motor = args.motor
    pwm = max(-50, min(50, args.pwm))
    seconds = max(0.1, min(2.0, args.seconds))

    vals = [0, 0, 0, 0]
    vals[motor - 1] = pwm
    drive = build_frame(FUNC_MOTOR, bytes((v & 0xFF) for v in vals))
    stop = build_frame(FUNC_MOTOR, bytes([0, 0, 0, 0]))

    try:
        s = serial.Serial(args.port, BAUD, timeout=0.1, write_timeout=2.0)
    except Exception as e:
        sys.exit(f"Ouverture {args.port} impossible : {e!r}")

    p = FrameParser()
    state = {"last": None}
    print(f"Port {args.port}. M{motor} a {pwm}% pendant {seconds:g}s (roues en l'air).",
          flush=True)

    read_window(s, p, 1.0, None, None, state)
    base = state["last"]
    if base is None:
        s.close()
        sys.exit("Aucune trame encodeur : carte muette ?")
    print(f"Baseline : M1={base[0]} M2={base[1]} M3={base[2]} M4={base[3]}", flush=True)

    peak = list(base)
    try:
        try:
            n = s.write(drive)                  # UNE seule ecriture -> demarre
            print(f"drive envoye ({n} o).", flush=True)
        except Exception as e:
            print(f"write drive echec : {e!r} -> STOP", flush=True)
        read_window(s, p, seconds, base, peak, state)   # le moteur tourne
    finally:
        for _ in range(3):                      # STOP espace, entrecoupe de lecture
            try:
                s.write(stop)
            except Exception:
                pass
            read_window(s, p, 0.12, base, peak, state)

    read_window(s, p, 0.8, base, peak, state)
    final = state["last"] or base
    s.close()

    print("\n=== RESULTAT ===", flush=True)
    for i in range(4):
        dl = final[i] - base[i]
        pk = peak[i] - base[i]
        tag = "   <-- pilote" if i == motor - 1 else ""
        print(f"  M{i + 1} : delta {dl:+6d}   (pic {pk:+6d}){tag}", flush=True)
    pkd = peak[motor - 1] - base[motor - 1]
    dd = final[motor - 1] - base[motor - 1]
    print(flush=True)
    if abs(dd) > 20 or abs(pkd) > 20:
        print(f"OK : M{motor} a compte (delta {dd:+d}, pic {pkd:+d}) sous tension "
              f"-> CHAINE ENCODEUR FONCTIONNELLE.", flush=True)
    else:
        print(f"PROBLEME : M{motor} n'a quasi rien compte (delta {dd:+d}, pic {pkd:+d}).", flush=True)
        print("  Soit la roue n'a pas tourne (moteur), soit defaut chaine encodeur.", flush=True)


if __name__ == "__main__":
    main()
