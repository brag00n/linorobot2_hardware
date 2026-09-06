#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drive_test.py - Test moteur+encodeur ponctuel (roues en l'air).

Architecture robuste (comme le serveur MCP) : un THREAD LECTEUR lit/parse les
trames en continu, le THREAD PRINCIPAL ecrit les commandes moteur. Ne JAMAIS
lire et ecrire sur le meme handle depuis le meme thread : sur Windows, le CH340
se bloque ("device not functioning"). Ici read (thread) et write (principal)
sont separes -> ecritures fiables.

Pilote UN moteur a faible PWM une courte duree (commande repetee anti-watchdog),
puis STOP garanti, et affiche le delta de comptage encodeur.

Usage :
    python tools/drive_test.py --motor 2 --pwm 30 --seconds 1.5
    (--motor 1..4, --pwm borne a +-50, --seconds borne a 2)
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

_counts = None           # dernier [M1..M4] vu par le lecteur
_lock = threading.Lock()
_stop = threading.Event()


def reader(ser):
    parser = FrameParser()
    global _counts
    while not _stop.is_set():
        try:
            chunk = ser.read(256)
        except Exception:
            break
        if not chunk:
            continue
        for func, data, ok, raw in parser.feed(chunk):
            if ok and func == 0x0D and len(data) >= 16:
                d = decode_encoder(data)
                with _lock:
                    _counts = [int(round(d[f"M{i + 1}"])) for i in range(4)]


def get_counts():
    with _lock:
        return list(_counts) if _counts is not None else None


def wait_counts(timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        c = get_counts()
        if c is not None:
            return c
        time.sleep(0.02)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", type=int, default=2, choices=[1, 2, 3, 4])
    ap.add_argument("--pwm", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=1.5)
    args = ap.parse_args()
    pwm = max(-50, min(50, args.pwm))          # securite
    seconds = max(0.1, min(2.0, args.seconds))

    ser = serial.Serial(PORT, BAUD, timeout=0.1)   # write bloquant (write_timeout=None)
    threading.Thread(target=reader, args=(ser,), daemon=True).start()
    print(f"Port {PORT} ouvert. M{args.motor}, PWM {pwm}%, {seconds}s.", flush=True)

    base = wait_counts(1.5)
    if base is None:
        _stop.set(); ser.close()
        sys.exit("Aucune trame encodeur : carte muette ?")
    print(f"Baseline : M1={base[0]} M2={base[1]} M3={base[2]} M4={base[3]}", flush=True)

    vals = [0, 0, 0, 0]
    vals[args.motor - 1] = pwm
    frame = build_frame(FUNC_MOTOR, bytes((v & 0xFF) for v in vals))
    stop = build_frame(FUNC_MOTOR, bytes([0, 0, 0, 0]))

    # chien de garde : sortie forcee si tout se fige (SANS ecrire -> pas de deadlock)
    def watchdog():
        if not _stop.wait(seconds + 4.0):
            print("WATCHDOG: sortie forcee", flush=True)
            os._exit(2)
    threading.Thread(target=watchdog, daemon=True).start()

    peak = list(base)
    try:
        t0 = time.time()
        while time.time() - t0 < seconds:
            ser.write(frame)                   # ecriture sur le thread principal
            time.sleep(0.05)
            cur = get_counts()
            if cur:
                for i in range(4):
                    if abs(cur[i] - base[i]) > abs(peak[i] - base[i]):
                        peak[i] = cur[i]
    finally:
        for _ in range(6):                     # STOP franc, toujours execute
            ser.write(stop)
            time.sleep(0.03)

    final = wait_counts(0.8) or peak
    _stop.set()
    time.sleep(0.15)
    ser.close()

    print("\n=== RESULTAT ===", flush=True)
    for i in range(4):
        dl = final[i] - base[i]
        pk = peak[i] - base[i]
        flag = "   <-- pilote" if i == args.motor - 1 else ""
        print(f"  M{i + 1} : delta final {dl:+5d}   (pic {pk:+5d}){flag}", flush=True)
    dd = final[args.motor - 1] - base[args.motor - 1]
    print(flush=True)
    if abs(dd) > 20:
        print(f"OK : M{args.motor} a compte {dd:+d} tics sous tension "
              f"-> ENCODEUR FONCTIONNEL.", flush=True)
    else:
        print(f"PROBLEME : M{args.motor} n'a quasiment pas compte ({dd:+d}). "
              f"Roue reellement en rotation ? sinon defaut chaine encodeur.", flush=True)


if __name__ == "__main__":
    main()
