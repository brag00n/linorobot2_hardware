#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""check.py - Verification unitaire de la camera UnitV2 (M5Stack) Bamboo v4.

Test MONO-CARTE (voir tools/README.md). Valide le flux video de l'UnitV2, exposee
en MJPEG sur son reseau USB-Ethernet (driver SR9900, hote 10.254.239.1) :

  1. FLUX        : le flux MJPEG http://10.254.239.1/video_feed est joignable
                   (connexion HTTP etablie, en-tetes recus) ;
  2. IMAGE       : au moins une image JPEG est extraite du flux multipart ET
                   decodee (dimensions non nulles) -> la camera delivre bien.

N'importe rien des autres cartes. numpy/OpenCV servent seulement a decoder l'image
(le flux est lu en direct via urllib, sans dependre du support MJPEG-sur-HTTP de
cv2). L'UnitV2 doit etre sous tension et son interface reseau montee cote PC.

Usage :
  python firmware/unitv2_bamboo/tools/check.py [--url http://10.254.239.1/video_feed]
  -> code de sortie 0 si tout PASS, 1 sinon.
"""
import argparse
import sys
import urllib.request

import numpy as np
import cv2

DEFAULT_URL = "http://10.254.239.1/video_feed"


def _read_one_jpeg(stream, max_bytes=2_000_000):
    """Extrait la 1re image JPEG complete d'un flux multipart MJPEG.

    Cherche le marqueur de debut (SOI 0xFFD8) puis de fin (EOI 0xFFD9).
    Retourne les octets JPEG, ou None si rien de complet dans max_bytes.
    """
    buf = bytearray()
    while len(buf) < max_bytes:
        chunk = stream.read(4096)
        if not chunk:
            break
        buf += chunk
        start = buf.find(b"\xff\xd8")
        end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
        if start != -1 and end != -1:
            return bytes(buf[start:end + 2])
    return None


def check(url=DEFAULT_URL, timeout=5.0, verbose=True):
    """Verifie joignabilite du flux + decodage d'une image. Retourne True si PASS."""
    def say(*a):
        if verbose:
            print(*a, flush=True)

    results = {"flux": False, "image": False}
    say(f"=== Check camera UnitV2 Bamboo v4 sur {url} ===")

    # --- 1) FLUX : le flux MJPEG repond ---------------------------------------
    try:
        stream = urllib.request.urlopen(url, timeout=timeout)  # noqa: S310 (URL locale connue)
    except Exception as e:
        say(f"[1] FLUX  : FAIL - flux injoignable : {e!r}")
        say("    (UnitV2 sous tension ? interface reseau 10.254.239.x montee cote PC ?)")
        say("=== VERDICT : FAIL (flux=KO, image=KO) ===")
        return False
    ctype = stream.headers.get("Content-Type", "?")
    results["flux"] = True
    say(f"[1] FLUX  : PASS - flux joignable (Content-Type: {ctype}).")

    # --- 2) IMAGE : extraire + decoder une image JPEG -------------------------
    try:
        jpeg = _read_one_jpeg(stream)
    finally:
        stream.close()
    if jpeg is None:
        say("[2] IMAGE : FAIL - aucune image JPEG complete dans le flux.")
    else:
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            say(f"[2] IMAGE : FAIL - image recue ({len(jpeg)} octets) mais non decodable.")
        else:
            h, w = frame.shape[:2]
            results["image"] = True
            say(f"[2] IMAGE : PASS - image {w}x{h} decodee ({len(jpeg)} octets).")

    ok = all(results.values())
    say("=== VERDICT : " + ("PASS" if ok else "FAIL")
        + " (" + ", ".join(f"{k}={'OK' if v else 'KO'}" for k, v in results.items()) + ") ===")
    return ok


def parse_args():
    ap = argparse.ArgumentParser(description="Check camera UnitV2 Bamboo v4")
    ap.add_argument("--url", default=DEFAULT_URL,
                    help=f"URL du flux MJPEG (defaut {DEFAULT_URL})")
    ap.add_argument("--timeout", type=float, default=5.0)
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    sys.exit(0 if check(args.url, args.timeout) else 1)
