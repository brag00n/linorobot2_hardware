#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""check.py - Verification unitaire de la camera USB (webcam pan-tilt) Bamboo v4.

Test MONO-CARTE (voir tools/README.md). N'a besoin que d'OpenCV ; n'importe rien
des autres cartes. Valide la chaine de capture video et la presence des modeles de
vision utilises par le suivi de visage :

  1. OUVERTURE   : une camera s'ouvre (backend MSMF, scan des index 0..N) ET
                   delivre reellement une image (un cap peut s'ouvrir a vide) ;
  2. LECTURE     : quelques images consecutives sont lues sans echec ;
  3. RESOLUTION  : resolution et fps effectifs negocies (MJPG 1280x720@30 vise) ;
  4. MODELES     : cascade Haar + modeles DNN/YuNet/VitTrack presents sur disque.

Usage :
  python firmware/usbcam_bamboo/tools/check.py [--index auto] [--backend msmf]
  -> code de sortie 0 si tout PASS, 1 sinon.
"""
import argparse
import os
import sys

import cv2

BACKENDS = {"msmf": cv2.CAP_MSMF, "dshow": cv2.CAP_DSHOW, "any": cv2.CAP_ANY}

# Racine du repo (ce fichier : firmware/usbcam_bamboo/tools/check.py -> 3 niveaux)
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
_MODEL_DIR = os.path.join(_REPO, "firmware", "usbcam_bamboo", "Bambou4WD_python",
                          "src", "resources", "Other", "face_detection_model")
# modeles attendus (nom -> fichier), pour le suivi de visage multi-cartes
MODELS = {
    "Haar (frontal)": None,                                    # resolu a part
    "DNN proto (res10)": "deploy.prototxt",
    "DNN model (res10)": "res10_300x300_ssd_iter_140000.caffemodel",
    "YuNet (detecteur)": "face_detection_yunet_2023mar.onnx",
    "VitTrack (tracker)": "object_tracking_vittrack_2023sep.onnx",
}


def find_cascade():
    """Localise haarcascade_frontalface_default.xml : d'abord tools/ (racine du
    repo), sinon les cascades livrees avec OpenCV. Retourne le chemin ou None."""
    local = os.path.join(_REPO, "tools", "haarcascade_frontalface_default.xml")
    if os.path.isfile(local):
        return local
    packaged = os.path.join(cv2.data.haarcascades,
                            "haarcascade_frontalface_default.xml")
    return packaged if os.path.isfile(packaged) else None


def _try_open(index, backend, w, h, fps):
    """Ouvre + configure + LIT une image. Retourne le cap si valide, sinon None."""
    cap = cv2.VideoCapture(index, BACKENDS[backend])
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    return cap


def check(index="auto", backend="msmf", width=1280, height=720, fps=30,
          scan_max=6, verbose=True):
    """Verifie ouverture + lecture + resolution + modeles. Retourne True si PASS."""
    def say(*a):
        if verbose:
            print(*a, flush=True)

    results = {"ouverture": False, "lecture": False,
               "resolution": False, "modeles": False}
    say("=== Check camera USB Bamboo v4 ===")

    # --- 1) OUVERTURE : index demande, sinon scan sur les backends -----------
    backends = [backend] + [b for b in ("msmf", "dshow", "any") if b != backend]
    cap = used_index = used_backend = None
    if str(index).lower() != "auto":
        cap = _try_open(int(index), backend, width, height, fps)
        if cap is not None:
            used_index, used_backend = int(index), backend
    if cap is None:
        for b in backends:
            for i in range(scan_max):
                cap = _try_open(i, b, width, height, fps)
                if cap is not None:
                    used_index, used_backend = i, b
                    break
            if cap is not None:
                break

    if cap is not None:
        results["ouverture"] = True
        say(f"[1] OUVERTURE  : PASS - camera index {used_index} (backend {used_backend}).")
    else:
        say(f"[1] OUVERTURE  : FAIL - aucune camera exploitable (scan index 0..{scan_max - 1}, "
            "backends msmf/dshow/any). Verifier le branchement USB.")

    # --- 2) LECTURE : quelques images consecutives ---------------------------
    if cap is not None:
        good = 0
        for _ in range(10):
            ok, frame = cap.read()
            if ok and frame is not None:
                good += 1
        if good >= 8:
            results["lecture"] = True
            say(f"[2] LECTURE    : PASS - {good}/10 images lues.")
        else:
            say(f"[2] LECTURE    : FAIL - seulement {good}/10 images lues (flux instable).")

        # --- 3) RESOLUTION : effectifs negocies ------------------------------
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        afps = cap.get(cv2.CAP_PROP_FPS)
        cap.release()
        if aw > 0 and ah > 0:
            results["resolution"] = True
            say(f"[3] RESOLUTION : PASS - {aw}x{ah} @ {afps:.0f}fps effectifs.")
        else:
            say("[3] RESOLUTION : FAIL - resolution nulle rapportee par le driver.")

    # --- 4) MODELES : cascade Haar + modeles DNN/YuNet/Vit sur disque --------
    missing = []
    cascade = find_cascade()
    say(f"    Haar (frontal)      : {cascade or 'INTROUVABLE'}")
    if cascade is None:
        missing.append("Haar (frontal)")
    for label, fname in MODELS.items():
        if fname is None:
            continue
        path = os.path.join(_MODEL_DIR, fname)
        present = os.path.isfile(path)
        say(f"    {label:<20}: {'OK' if present else 'MANQUANT'}  ({fname})")
        if not present:
            missing.append(label)
    if not missing:
        results["modeles"] = True
        say("[4] MODELES    : PASS - cascade + modeles DNN/YuNet/Vit presents.")
    else:
        say("[4] MODELES    : FAIL - manquant(s) : " + ", ".join(missing) + ".")

    ok = all(results.values())
    say("=== VERDICT : " + ("PASS" if ok else "FAIL")
        + " (" + ", ".join(f"{k}={'OK' if v else 'KO'}" for k, v in results.items()) + ") ===")
    return ok


def parse_args():
    ap = argparse.ArgumentParser(description="Check camera USB Bamboo v4")
    ap.add_argument("--index", default="auto", help="index camera ou 'auto' (defaut auto)")
    ap.add_argument("--backend", default="msmf", choices=list(BACKENDS))
    ap.add_argument("--size", default="1280x720", help="resolution visee (defaut 1280x720)")
    ap.add_argument("--fps", type=int, default=30)
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        w, h = (int(v) for v in args.size.lower().split("x"))
    except ValueError:
        sys.exit(f"--size invalide : {args.size!r}")
    sys.exit(0 if check(args.index, args.backend, w, h, args.fps) else 1)
