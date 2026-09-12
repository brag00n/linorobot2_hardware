#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""check.py - Banc de test unitaire de la carte capteurs GrovePi+ Bambou.

Test MONO-CARTE (cf. tools/README.md du projet). Valide, cote PC, la chaine
PC <-> ATmega328P via l'adaptateur USB<->serie, en exercant TOUTES les fonctions
du protocole (version, statut, get/set config, save EEPROM protegee, auto-report).

Protocole (miroir de ../../stm32_bamboo/tools/ros_monitor.py) :
    [0xFF][ID][LEN][FUNC][donnees...][CHK]   ID: 0xFC hote->carte, 0xFB carte->hote

/!\ A LANCER SUR LE PC (adaptateur USB<->serie branche), PAS sur un Pi. Couper
tout moniteur serie qui tiendrait le port.

Les tests des trames config/statut/version ne passent qu'APRES avoir reflashe la
carte avec le firmware >= 0.2.0.

Usage :
  python firmware/grovepi_bamboo/tools/check.py [PORT] [TEST|all]
    PORT : defaut COM6.   TEST : nom d'un test ci-dessous, ou 'all' (defaut).
  Tests : connexion version status get_config set_config enable_disable
          ultra_pins eeprom_save eeprom_throttle timesync timestamp auto_report
  -> code de sortie 0 si tout PASS, 1 sinon.

  /!\ eeprom_save et eeprom_throttle ECRIVENT l'EEPROM (usure). Ils sont exclus
      de 'all' par defaut ; les lancer nommement ou via 'all+eeprom'.
"""
import os
import sys
import threading
import time

import serial

BAUD = 115200
PTO_HEAD, PTO_ID_RX, PTO_ID_TX = 0xFF, 0xFC, 0xFB
FUNC_REQUEST_DATA  = 0x50
FUNC_VERSION       = 0x51
FUNC_TIME_SYNC     = 0x52
FUNC_STATUS        = 0x53
FUNC_CONFIG        = 0x54
FUNC_SET_CONFIG    = 0x55
FUNC_CONFIG_ACTION = 0x56
FUNC_REPORT_IMU    = 0x60
FUNC_REPORT_ULTRA  = 0x61
FUNC_REPORT_IR     = 0x62

SAVE_VERIFY      = 0x5F
CFG_ACT_SAVE     = 0x01
CFG_ACT_DEFAULTS = 0x02
CFG_ACT_RELOAD   = 0x03
CFG_RES = {0: "done", 1: "unchanged", 2: "throttled", 3: "bad-guard"}

DEV_IMU, DEV_ULTRA, DEV_IR = 0x00, 0x01, 0x20
DEV_ULTRA0 = 0x10
DEV_ALL = 0xFF
DEV_IDS = [DEV_IMU, DEV_ULTRA, 0x10, 0x11, 0x12, 0x13, DEV_IR]
HEALTH = {0x00: "OK", 0x01: "off", 0x10: "err", 0x11: "i2c-nack",
          0x12: "i2c-read", 0x21: "no-echo", 0x31: "adc-low", 0x32: "adc-high"}
BOARD_STATE = {0: "OK", 1: "DEGRADED", 2: "FAULT"}

WATCHDOG_S = 90.0    # 'all'+eeprom = ~40 s (ask() attend sa fenetre pleine) ; marge large


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


def u16(lo, hi):
    return lo | (hi << 8)


def u32(d, off=0):
    return d[off] | (d[off + 1] << 8) | (d[off + 2] << 16) | (d[off + 3] << 24)


class Board:
    """Connexion serie + primitives protocole partagees par les tests."""
    def __init__(self, port):
        self.s = serial.Serial(port, BAUD, timeout=0.2, write_timeout=2.0)
        # L'ATmega se reset sur ouverture du port (DTR) -> laisser booter puis vider.
        time.sleep(2.0)
        self.s.reset_input_buffer()
        self.p = FrameParser()

    def send(self, frame):
        return self.s.write(frame)

    def request(self, sub, param=DEV_ALL):
        self.send(build_frame(FUNC_REQUEST_DATA, bytes([sub, param])))

    def set_config(self, dev, enabled, pin_a, pin_b, period):
        self.send(build_frame(FUNC_SET_CONFIG,
                  bytes([dev, enabled, pin_a, pin_b, period & 0xFF, (period >> 8) & 0xFF])))

    def action(self, act, guard=SAVE_VERIFY):
        self.send(build_frame(FUNC_CONFIG_ACTION, bytes([act, guard])))

    def time_sync(self, seq, timeout=1.0):
        """Echange TIME_SYNC : renvoie (t_board_ms, rtt_ms, offset_ms) ou None.

        offset tel que ros_time ~ device_time + offset (referentiel host ici).
        /!\ t2 est capture a l'INSTANT d'arrivee de la trame (pas apres une fenetre
        fixe) -> RTT reel. On lit le port en direct sans passer par collect().
        """
        self.s.reset_input_buffer()
        self.p = FrameParser()
        t1 = time.time()
        self.send(build_frame(FUNC_TIME_SYNC, bytes([seq])))
        deadline = t1 + timeout
        while time.time() < deadline:
            for func, data, ok in self.p.feed(self.s.read(64)):
                if ok and func == FUNC_TIME_SYNC and len(data) >= 5 and data[0] == seq:
                    t2 = time.time()
                    t_board = u32(data, 1)
                    rtt = (t2 - t1) * 1000.0
                    host_mid = (t1 + t2) / 2.0 * 1000.0    # ms hote au milieu de l'echange
                    offset = host_mid - t_board
                    return t_board, rtt, offset
        return None

    def collect(self, duration):
        """Ecoute pendant duration s, renvoie [(func, data)] (checksum OK)."""
        out, t0 = [], time.time()
        while time.time() - t0 < duration:
            for func, data, ok in self.p.feed(self.s.read(256)):
                if ok:
                    out.append((func, data))
        return out

    def ask(self, sub, param=DEV_ALL, want=None, duration=1.5):
        """Envoie une requete, renvoie la 1re trame `want` (defaut = sub)."""
        want = sub if want is None else want
        self.s.reset_input_buffer()
        self.p = FrameParser()
        self.request(sub, param)
        for func, data in self.collect(duration):
            if func == want:
                return data
        return None

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


# --- Tests -------------------------------------------------------------------
# Chaque test : fn(board, say) -> (bool, detail). say(*a) pour les logs.

def t_connexion(b, say):
    n = b.send(build_frame(FUNC_REQUEST_DATA, bytes([FUNC_VERSION, DEV_ALL])))
    return True, f"port ouvert, write() a rendu {n} octets"


def t_version(b, say):
    d = b.ask(FUNC_VERSION)
    if d is None or len(d) < 2:
        return False, "pas de trame 0x51 (l'ordre PC->carte n'aboutit pas)"
    ver = f"{d[0]}.{d[1]}" + (f".{d[2]}" if len(d) >= 3 else "")
    return True, f"firmware {ver} (test de contact OK)"


def t_status(b, say):
    d = b.ask(FUNC_STATUS)
    if d is None or len(d) < 2:
        return False, "pas de trame 0x53"
    state, n = d[0], d[1]
    devs = [(d[2 + 2 * i], d[3 + 2 * i]) for i in range(n) if 3 + 2 * i < len(d)]
    txt = " ".join(f"0x{i:02X}:{HEALTH.get(h, hex(h))}" for i, h in devs)
    say(f"    board={BOARD_STATE.get(state, state)}  devices: {txt}")
    return state in BOARD_STATE, f"board={BOARD_STATE.get(state, state)}, {n} devices"


def _get_config(b, dev):
    d = b.ask(FUNC_CONFIG, param=dev, want=FUNC_CONFIG)
    if d is None or len(d) < 7 or d[0] != dev:
        return None
    return {"dev": d[0], "enabled": d[1], "pinA": d[2], "pinB": d[3],
            "period": u16(d[4], d[5]), "health": d[6]}


def t_get_config(b, say):
    miss = []
    for dev in DEV_IDS:
        c = _get_config(b, dev)
        if c is None:
            miss.append(f"0x{dev:02X}")
        else:
            say(f"    0x{dev:02X}: en={c['enabled']} pinA={c['pinA']} "
                f"pinB={c['pinB']} per={c['period']}ms sante={HEALTH.get(c['health'], hex(c['health']))}")
    if miss:
        return False, "config absente pour " + ", ".join(miss)
    return True, f"{len(DEV_IDS)} configs lues"


def t_set_config(b, say):
    orig = _get_config(b, DEV_IR)
    if orig is None:
        return False, "IR (0x20) illisible"
    new = 137 if orig["period"] != 137 else 149
    b.set_config(DEV_IR, 0xFF, 0xFF, 0xFF, new)      # 0xFF/0 = garder sauf period
    time.sleep(0.2)
    got = _get_config(b, DEV_IR)
    ok = got is not None and got["period"] == new
    b.set_config(DEV_IR, 0xFF, 0xFF, 0xFF, orig["period"])   # restaure la RAM
    detail = f"period IR {orig['period']}->{new} relue={got['period'] if got else '?'}"
    return ok, detail


def t_enable_disable(b, say):
    b.set_config(DEV_IR, 0, 0xFF, 0xFF, 0)           # desactive IR
    time.sleep(0.5)                                  # laisse le dernier 0x62 en vol s'ecouler
    b.s.reset_input_buffer(); b.p = FrameParser()    # purge les 0x62 anterieurs a la desactivation
    self_seen = any(f == FUNC_REPORT_IR for f, _ in b.collect(1.0))
    b.set_config(DEV_IR, 1, 0xFF, 0xFF, 0)           # reactive IR
    time.sleep(0.3)
    b.s.reset_input_buffer(); b.p = FrameParser()
    re_seen = any(f == FUNC_REPORT_IR for f, _ in b.collect(1.0))
    ok = (not self_seen) and re_seen
    return ok, f"0x62 apres off={self_seen} (attendu False), apres on={re_seen} (attendu True)"


def t_ultra_pins(b, say):
    orig = _get_config(b, DEV_ULTRA0)
    if orig is None:
        return False, "canal ultra 0 (0x10) illisible"
    ta, ea = 2, 3
    b.set_config(DEV_ULTRA0, 0xFF, ta, ea, 0)
    time.sleep(0.2)
    got = _get_config(b, DEV_ULTRA0)
    ok = got is not None and got["pinA"] == ta and got["pinB"] == ea
    b.set_config(DEV_ULTRA0, 0xFF, orig["pinA"], orig["pinB"], 0)   # restaure
    relus = (got["pinA"], got["pinB"]) if got else "?"
    return ok, f"trig/echo relus={relus} (attendu {(ta, ea)})"


def t_eeprom_save(b, say):
    b.request(0)  # noop pour vider
    b.s.reset_input_buffer(); b.p = FrameParser()
    b.action(CFG_ACT_SAVE)
    ack = next((d for f, d in b.collect(1.5) if f == FUNC_CONFIG_ACTION), None)
    if ack is None or len(ack) < 2:
        return False, "pas d'ack 0x56"
    res = ack[1]
    say(f"    save -> result={CFG_RES.get(res, res)}")
    return res in (0, 1), f"save result={CFG_RES.get(res, res)} (done/unchanged attendu)"


def t_eeprom_throttle(b, say):
    time.sleep(2.2)                                  # purge un eventuel throttle anterieur
    # 1) force une ecriture physique reelle (config qui differe du stocke).
    b.set_config(DEV_IR, 0xFF, 0xFF, 0xFF, 111)
    b.s.reset_input_buffer(); b.p = FrameParser()
    b.action(CFG_ACT_SAVE)
    a1 = next((d for f, d in b.collect(1.5) if f == FUNC_CONFIG_ACTION), None)
    if a1 and len(a1) >= 2 and a1[1] == 1:           # unchanged -> re-tente autre valeur
        b.set_config(DEV_IR, 0xFF, 0xFF, 0xFF, 112)
        b.s.reset_input_buffer(); b.p = FrameParser()
        b.action(CFG_ACT_SAVE)
        a1 = next((d for f, d in b.collect(1.5) if f == FUNC_CONFIG_ACTION), None)
    r1 = a1[1] if a1 and len(a1) >= 2 else None
    # 2) save immediate d'une NOUVELLE config -> doit etre throttlee (result=2).
    b.set_config(DEV_IR, 0xFF, 0xFF, 0xFF, 222)
    b.s.reset_input_buffer(); b.p = FrameParser()
    b.action(CFG_ACT_SAVE)
    a2 = next((d for f, d in b.collect(1.5) if f == FUNC_CONFIG_ACTION), None)
    r2 = a2[1] if a2 and len(a2) >= 2 else None
    say(f"    save#1 result={CFG_RES.get(r1, r1)}  save#2(rapide) result={CFG_RES.get(r2, r2)}")
    # restaure une config saine en EEPROM apres expiration du throttle.
    time.sleep(2.2)
    b.action(CFG_ACT_DEFAULTS)
    b.collect(1.0)
    ok = (r1 == 0 and r2 == 2)
    return ok, f"1er save={CFG_RES.get(r1, r1)} (done), 2e rapproche={CFG_RES.get(r2, r2)} (throttled attendu)"


def t_auto_report(b, say):
    # Chaque trame report porte [ts u32 ms] en tete (horloge monotone) -> +4 o.
    b.s.reset_input_buffer(); b.p = FrameParser()
    funcs, last, ts = {}, {}, {}
    for func, data in b.collect(2.5):
        funcs[func] = funcs.get(func, 0) + 1
        if func == FUNC_REPORT_IMU and len(data) >= 8:
            ts["imu"] = u32(data, 0)
            last["imu"] = (s16(data[4], data[5]) / 100.0, s16(data[6], data[7]) / 100.0)
        elif func == FUNC_REPORT_ULTRA and len(data) >= 12:
            ts["ultra"] = u32(data, 0)
            last["ultra"] = [u16(data[4 + 2 * i], data[5 + 2 * i]) for i in range(4)]
        elif func == FUNC_REPORT_IR and len(data) >= 8:
            ts["ir"] = u32(data, 0)
            last["ir"] = (u16(data[4], data[5]), u16(data[6], data[7]))
    say("    trames :", ", ".join(f"0x{f:02X}:{c}" for f, c in sorted(funcs.items())) or "aucune")
    if "imu" in last:
        say(f"    IMU     : ts={ts['imu']}ms roll={last['imu'][0]:+.1f} pitch={last['imu'][1]:+.1f}")
    if "ultra" in last:
        say(f"    Ultrason: ts={ts['ultra']}ms  " + "  ".join(
            f"S{i}={'--' if d == 0xFFFF else d} mm" for i, d in enumerate(last["ultra"])))
    if "ir" in last:
        dist, adc = last["ir"]
        say(f"    IR Sharp: ts={ts['ir']}ms dist={'--' if dist == 0xFFFF else str(dist) + ' mm'} (adc={adc})")
    saw = FUNC_REPORT_IMU in funcs and FUNC_REPORT_ULTRA in funcs
    return saw, "IMU 0x60 + ultra 0x61 vus" if saw else "manque IMU et/ou ultra"


def t_timesync(b, say):
    # Echange aller-retour TIME_SYNC : la carte echoe seq + son horloge monotone.
    r1 = b.time_sync(0x11)
    if r1 is None:
        return False, "pas de reponse 0x52 (firmware < 0.3.0 ?)"
    tb1, rtt1, off1 = r1
    time.sleep(0.3)
    r2 = b.time_sync(0x22)
    if r2 is None:
        return False, "2e echange 0x52 sans reponse"
    tb2, rtt2, off2 = r2
    say(f"    sync#1 t_board={tb1}ms RTT={rtt1:.1f}ms offset={off1:.0f}ms")
    say(f"    sync#2 t_board={tb2}ms RTT={rtt2:.1f}ms offset={off2:.0f}ms")
    # PASS : horloge croit (~ +300 ms attendu), RTT plausible (< 200 ms).
    grew = tb2 > tb1
    dt = tb2 - tb1
    rtt_ok = 0.0 <= rtt2 < 200.0
    ok = grew and 150 < dt < 900 and rtt_ok
    return ok, f"t_board {tb1}->{tb2} (dt={dt}ms attendu ~300), RTT={rtt2:.1f}ms"


def t_timestamp(b, say):
    # Deux trames IMU successives : le ts doit croitre de ~ la periode configuree.
    c = _get_config(b, DEV_IMU)
    period = c["period"] if c else 50
    b.s.reset_input_buffer(); b.p = FrameParser()
    stamps = []
    for func, data in b.collect(2.5):
        if func == FUNC_REPORT_IMU and len(data) >= 8:
            stamps.append(u32(data, 0))
    if len(stamps) < 2:
        return False, f"moins de 2 trames IMU horodatees ({len(stamps)})"
    deltas = [b_ - a_ for a_, b_ in zip(stamps, stamps[1:])]
    mono = all(d > 0 for d in deltas)
    avg = sum(deltas) / len(deltas)
    say(f"    {len(stamps)} trames IMU, ts {stamps[0]}..{stamps[-1]}ms, "
        f"dt moyen={avg:.0f}ms (periode={period}ms)")
    # tolerance large : le jitter sous-boucle et la charge peuvent etirer dt.
    ok = mono and 0.4 * period <= avg <= 3.0 * period
    return ok, f"ts croissant={mono}, dt moyen={avg:.0f}ms (periode {period}ms)"


# Ordre logique ; eeprom_* exclus de 'all' (usure EEPROM) sauf 'all+eeprom'.
TESTS = [
    ("connexion", t_connexion),
    ("version", t_version),
    ("status", t_status),
    ("get_config", t_get_config),
    ("set_config", t_set_config),
    ("enable_disable", t_enable_disable),
    ("ultra_pins", t_ultra_pins),
    ("eeprom_save", t_eeprom_save),
    ("eeprom_throttle", t_eeprom_throttle),
    ("timesync", t_timesync),
    ("timestamp", t_timestamp),
    ("auto_report", t_auto_report),
]
EEPROM_TESTS = {"eeprom_save", "eeprom_throttle"}


def run(port="COM6", which="all", verbose=True):
    def say(*a):
        if verbose:
            print(*a, flush=True)

    def watchdog():
        time.sleep(WATCHDOG_S)
        say("WATCHDOG: sortie forcee (port fige ?)")
        os._exit(3)
    threading.Thread(target=watchdog, daemon=True).start()

    if which in ("all", "all+eeprom"):
        selected = [(n, f) for n, f in TESTS
                    if which == "all+eeprom" or n not in EEPROM_TESTS]
    else:
        selected = [(n, f) for n, f in TESTS if n == which]
        if not selected:
            say(f"Test inconnu : {which}. Dispo : " + ", ".join(n for n, _ in TESTS))
            return False

    say(f"=== Banc GrovePi+ Bambou sur {port} @ {BAUD} ({which}) ===")
    try:
        b = Board(port)
    except Exception as e:
        say(f"[connexion] FAIL - ouverture {port} impossible : {e!r}")
        say("    (PL2303 en erreur code 10 ? mauvais COM ? adaptateur debranche ?)")
        return False

    results = {}
    try:
        for name, fn in selected:
            try:
                ok, detail = fn(b, say)
            except Exception as e:
                ok, detail = False, f"exception : {e!r}"
            results[name] = ok
            say(f"[{name:15}] {'PASS' if ok else 'FAIL'} - {detail}")
    finally:
        b.close()

    allok = all(results.values())
    say("=== VERDICT : " + ("PASS" if allok else "FAIL") + " ("
        + ", ".join(f"{k}={'OK' if v else 'KO'}" for k, v in results.items()) + ") ===")
    return allok


if __name__ == "__main__":
    args = [a for a in sys.argv[1:]]
    port = "COM6"
    which = "all"
    for a in args:
        if a.lower().startswith("com") or a.startswith("/dev"):
            port = a
        else:
            which = a
    sys.exit(0 if run(port, which) else 1)
