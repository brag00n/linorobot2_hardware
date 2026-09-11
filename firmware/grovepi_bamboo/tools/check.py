#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""check.py - Verification unitaire de la carte capteurs GrovePi+ Bambou.

Test MONO-CARTE (cf. tools/README.md du projet). Valide, cote PC, la chaine
PC <-> ATmega328P via l'adaptateur USB<->serie (PL2303TA) :

  1. CONNEXION   : le port serie s'ouvre et une ecriture passe ;
  2. VERSION     : la carte repond a la requete version (REQUEST_DATA 0x50/0x51)
                   -> le chemin PC->carte fonctionne ;
  3. AUTO-REPORT : des trames IMU (0x60) et ultrason (0x61) arrivent spontanement
                   -> le firmware tourne et les capteurs sont echantillonnes.

Affiche aussi le dernier angle fusionne (roll/pitch) et les 4 distances lues.

Protocole (miroir de ../../stm32_bamboo/tools/ros_monitor.py) :
    [0xFF][ID][LEN][FUNC][donnees...][CHK]   ID: 0xFC hote->carte, 0xFB carte->hote

/!\ A LANCER SUR LE PC (adaptateur USB<->serie branche), PAS sur un Pi. Couper
tout moniteur serie qui tiendrait le port. Le PL2303TA doit monter un COMx SANS
erreur (code 10) sous Windows 11 -- voir platformio.ini "ETAPE 0".

Usage :
  python firmware/grovepi_bamboo/tools/check.py [PORT]     # defaut COM6
  -> code de sortie 0 si tout PASS, 1 sinon.
"""
import os
import sys
import threading
import time

import serial

BAUD = 115200
PTO_HEAD, PTO_ID_RX, PTO_ID_TX = 0xFF, 0xFC, 0xFB
FUNC_REQUEST_DATA = 0x50
FUNC_VERSION      = 0x51
FUNC_REPORT_IMU   = 0x60
FUNC_REPORT_ULTRA = 0x61
WATCHDOG_S = 10.0


def build_frame(func, params=b""):
    """Trame hote -> carte : [0xFF][0xFC][LEN][FUNC][params][CHK]."""
    length = 3 + len(params)
    frame = bytearray([PTO_HEAD, PTO_ID_RX, length, func]) + bytearray(params)
    frame.append(sum(frame[2:]) & 0xFF)
    return bytes(frame)


class FrameParser:
    """Machine a etats : accepte les octets recus, rend les trames completes."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk):
        self.buf.extend(chunk)
        out = []
        while True:
            start = self._find_header()
            if start is None:
                break
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < 3:
                break
            length = self.buf[2]
            total = length + 2
            if length < 2 or total > 64:
                del self.buf[:2]
                continue
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            func, data, chk = raw[3], raw[4:total - 1], raw[total - 1]
            out.append((func, data, (sum(raw[2:total - 1]) & 0xFF) == chk))
        return out

    def _find_header(self):
        b = self.buf
        for i in range(len(b) - 1):
            if b[i] == PTO_HEAD and b[i + 1] == PTO_ID_TX:
                return i
        return len(b) - 1 if b and b[-1] == PTO_HEAD else None


def s16(lo, hi):
    v = lo | (hi << 8)
    return v - 0x10000 if v & 0x8000 else v


def check(port="COM6", listen_s=2.5, verbose=True):
    """Verifie connexion + version + auto-report IMU/ultrason. Retourne True/False."""
    def say(*a):
        if verbose:
            print(*a, flush=True)

    results = {"connexion": False, "version": False, "auto_report": False}
    version = None
    last_imu = None
    last_ultra = None
    funcs = {}

    def watchdog():
        time.sleep(WATCHDOG_S)
        say("WATCHDOG: sortie forcee (port fige ?)")
        os._exit(3)
    threading.Thread(target=watchdog, daemon=True).start()

    say(f"=== Check GrovePi+ Bambou sur {port} @ {BAUD} ===")

    req = build_frame(FUNC_REQUEST_DATA, bytes([FUNC_VERSION]))
    try:
        s = serial.Serial(port, BAUD, timeout=0.2, write_timeout=2.0)
    except Exception as e:
        say(f"[1] CONNEXION : FAIL - ouverture {port} impossible : {e!r}")
        say("    (PL2303 en erreur code 10 ? mauvais COM ? adaptateur debranche ?)")
        return _verdict(results, say)

    # L'ATmega se reset sur ouverture du port (DTR) -> laisser booter puis vider.
    time.sleep(2.0)
    s.reset_input_buffer()
    try:
        n = s.write(req)
        results["connexion"] = True
        say(f"[1] CONNEXION : PASS - port ouvert, write() a rendu {n} octets.")
    except serial.SerialTimeoutException:
        s.close()
        say("[1] CONNEXION : FAIL - write bloquee (timeout 2s).")
        return _verdict(results, say)
    except Exception as e:
        s.close()
        say(f"[1] CONNEXION : FAIL - erreur write : {e!r}")
        return _verdict(results, say)

    p = FrameParser()
    t0 = time.time()
    while time.time() - t0 < listen_s:
        for func, data, ok in p.feed(s.read(256)):
            if not ok:
                continue
            funcs[func] = funcs.get(func, 0) + 1
            if func == FUNC_VERSION and len(data) >= 2:
                version = (data[0], data[1])
            elif func == FUNC_REPORT_IMU and len(data) >= 4:
                last_imu = (s16(data[0], data[1]) / 100.0, s16(data[2], data[3]) / 100.0)
            elif func == FUNC_REPORT_ULTRA and len(data) >= 8:
                last_ultra = [data[2 * i] | (data[2 * i + 1] << 8) for i in range(4)]
    s.close()

    if funcs:
        say("    trames recues :",
            ", ".join(f"0x{f:02X}:{c}" for f, c in sorted(funcs.items())))

    if version is not None:
        results["version"] = True
        say(f"[2] VERSION    : PASS - firmware {version[0]}.{version[1]}.")
    else:
        say("[2] VERSION    : FAIL - pas de trame 0x51 (l'ordre PC->carte n'aboutit pas).")

    saw_imu = FUNC_REPORT_IMU in funcs
    saw_ultra = FUNC_REPORT_ULTRA in funcs
    if saw_imu and saw_ultra:
        results["auto_report"] = True
        say("[3] AUTO-REPORT: PASS - IMU (0x60) et ultrason (0x61) vus.")
    elif funcs:
        say(f"[3] AUTO-REPORT: FAIL - manque {'IMU ' if not saw_imu else ''}"
            f"{'ULTRA' if not saw_ultra else ''} (capteur non cable / firmware ?).")
    else:
        say("[3] AUTO-REPORT: FAIL - aucune trame (carte muette / non alimentee ?).")

    if last_imu:
        say(f"    IMU     : roll={last_imu[0]:+.1f} deg  pitch={last_imu[1]:+.1f} deg")
    if last_ultra:
        say("    Ultrason: " + "  ".join(
            f"S{i}={'--' if d == 0xFFFF else d} mm" for i, d in enumerate(last_ultra)))

    return _verdict(results, say)


def _verdict(results, say):
    ok = all(results.values())
    say("=== VERDICT : " + ("PASS" if ok else "FAIL")
        + " (" + ", ".join(f"{k}={'OK' if v else 'KO'}" for k, v in results.items()) + ") ===")
    return ok


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
    sys.exit(0 if check(port) else 1)
