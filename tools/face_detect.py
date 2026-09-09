#!/usr/bin/env python3
"""Detection de visage en direct sur la camera USB (OpenCV, Windows/DirectShow).

Architecture a DEUX pipelines decouples par threads, pour que l'affichage
reste fluide meme quand la detection est lente :

  P1 - AFFICHAGE (thread principal) : lit la camera en pleine resolution et
       affiche la fenetre a la cadence camera. Ne fait AUCUNE detection.
  P2 - DETECTION (thread separe)    : recupere la derniere image, la reduit
       fortement (largeur --det-width, defaut 320 px), applique la cascade de
       Haar, et publie les boites trouvees. Tourne a sa propre cadence.

Les boites detectees en P2 (basse resolution) sont REMAPPEES a l'echelle de
P1 (pleine resolution) via le facteur disp_w/det_w avant d'etre dessinees.
L'affichage montre toujours la derniere boite connue -> pas de gel de la
fenetre pendant un cycle de detection.

Position du visage principal remontee (repere P1) :
  - centre en pixels (cx, cy),
  - position normalisee / CENTRE image dans [-1, +1] (nx<0 gauche, ny<0 haut),
  - aire relative (visage/image), proxy de distance.

Exemples :
  python face_detect.py                    # camera externe, det 320px
  python face_detect.py --det-width 240    # detection encore plus rapide
  python face_detect.py --index 0          # webcam integree
  python face_detect.py --flip none        # image telle quelle
"""
import argparse
import os
import sys
import threading
import time

import cv2


def find_cascade():
    here = os.path.dirname(os.path.abspath(__file__))
    local = os.path.join(here, "haarcascade_frontalface_default.xml")
    if os.path.exists(local):
        return local
    builtin = os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
    return builtin if os.path.exists(builtin) else None


def apply_flip(frame, mode):
    if mode == "v":
        return cv2.flip(frame, 0)
    if mode == "h":
        return cv2.flip(frame, 1)
    if mode == "180":
        return cv2.flip(frame, -1)
    return frame


class Shared:
    """Etat partage P1 <-> P2, protege par un verrou."""
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None          # derniere image P1 (pleine res, deja flippee)
        self.frame_id = 0
        self.faces = []            # boites REMAPPEES en coords P1 : [(x,y,w,h),...]
        self.faces_frame_id = -1   # id de l'image sur laquelle ces boites datent
        self.det_fps = 0.0
        self.stop = False


def detection_worker(shared, cascade_path, det_width, min_size):
    cascade = cv2.CascadeClassifier(cascade_path)
    if cascade.empty():
        print(f"[P2] echec chargement cascade : {cascade_path}")
        return
    t0 = time.time()
    n = 0
    last_id = -1
    while True:
        with shared.lock:
            if shared.stop:
                break
            frame = shared.frame
            fid = shared.frame_id
        if frame is None or fid == last_id:
            time.sleep(0.002)      # rien de neuf, on laisse la main
            continue
        last_id = fid

        fh, fw = frame.shape[:2]
        scale = fw / float(det_width)           # facteur de remapping P2 -> P1
        det_h = int(round(fh / scale))
        small = cv2.resize(frame, (det_width, det_h), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        dets = cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5,
            minSize=(min_size, min_size))

        # Remapping des boites basse-res vers la pleine res P1
        remapped = [(int(x * scale), int(y * scale),
                     int(w * scale), int(h * scale)) for (x, y, w, h) in dets]

        n += 1
        now = time.time()
        det_fps = shared.det_fps
        if now - t0 >= 1.0:
            det_fps = n / (now - t0)
            t0 = now
            n = 0
        with shared.lock:
            shared.faces = remapped
            shared.faces_frame_id = fid
            shared.det_fps = det_fps


def main():
    ap = argparse.ArgumentParser(description="Detection visage 2 pipelines (affichage / detection)")
    ap.add_argument("--index", type=int, default=1,
                    help="index camera (0 = webcam PC, 1 = HD Web Camera externe)")
    ap.add_argument("--size", default="1280x720", help="resolution P1 affichage, ex 640x480")
    ap.add_argument("--fps", type=int, default=30, help="cadence demandee a la camera (defaut 30)")
    ap.add_argument("--backend", default="msmf", choices=["msmf", "dshow", "any"],
                    help="backend capture (msmf=30fps MJPG ; dshow reste en YUY2 ~4fps)")
    ap.add_argument("--det-width", type=int, default=320,
                    help="largeur P2 detection en px (defaut 320 ; plus petit = plus rapide)")
    ap.add_argument("--flip", default="v", choices=["none", "v", "h", "180"],
                    help="retournement image (defaut v)")
    ap.add_argument("--min-size", type=int, default=24,
                    help="taille min visage en px DANS L'IMAGE REDUITE P2 (defaut 24)")
    ap.add_argument("--no-display", action="store_true",
                    help="pas de fenetre, positions en console (P2 tourne quand meme)")
    args = ap.parse_args()

    try:
        w, h = (int(x) for x in args.size.lower().split("x"))
    except ValueError:
        sys.exit(f"--size invalide : {args.size!r} (attendu ex 1280x720)")

    cascade_path = find_cascade()
    if not cascade_path:
        sys.exit("Cascade haarcascade_frontalface_default.xml introuvable.")

    # MSMF (Media Foundation) negocie le flux MJPG compresse -> 30fps en 720p.
    # DSHOW restait bloque en YUY2 brut (~4fps, plafond bande passante USB2).
    backend = {"msmf": cv2.CAP_MSMF, "dshow": cv2.CAP_DSHOW,
               "any": cv2.CAP_ANY}[args.backend]
    cap = cv2.VideoCapture(args.index, backend)
    if not cap.isOpened():
        sys.exit(f"Impossible d'ouvrir la camera index {args.index} (backend {args.backend}).")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    afps = cap.get(cv2.CAP_PROP_FPS)
    print(f"P1 affichage : camera index {args.index}, {aw}x{ah} @ {afps:.0f}fps demande, flip={args.flip}")
    print(f"P2 detection : largeur {args.det_width}px (remapping x{aw / args.det_width:.2f})")
    print(f"Cascade : {cascade_path}")
    if args.no_display:
        print("Mode console. cx,cy (px) | nx,ny [-1..1] | aire% | fps P1/P2")
    else:
        print("Fenetre ouverte. 'q' ou Echap pour quitter.")

    shared = Shared()
    worker = threading.Thread(
        target=detection_worker,
        args=(shared, cascade_path, args.det_width, args.min_size),
        daemon=True)
    worker.start()

    disp_t0 = time.time()
    disp_n = 0
    disp_fps = 0.0
    last_report = 0.0
    read_fail = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                # MSMF emet parfois un grab rate transitoire : on tolere.
                read_fail += 1
                if read_fail > 30:
                    print("Lecture image echouee de facon repetee, arret.")
                    break
                time.sleep(0.005)
                continue
            read_fail = 0
            frame = apply_flip(frame, args.flip)
            fh, fw = frame.shape[:2]

            # Publie l'image pour P2
            with shared.lock:
                shared.frame = frame
                shared.frame_id += 1
                faces = list(shared.faces)
                det_fps = shared.det_fps

            disp_n += 1
            now = time.time()
            if now - disp_t0 >= 1.0:
                disp_fps = disp_n / (now - disp_t0)
                disp_t0 = now
                disp_n = 0

            main_face = max(faces, key=lambda r: r[2] * r[3]) if faces else None

            if main_face is not None:
                x, y, ww, hh = main_face
                cx, cy = x + ww / 2.0, y + hh / 2.0
                nx = (cx - fw / 2.0) / (fw / 2.0)
                ny = (cy - fh / 2.0) / (fh / 2.0)
                area_pct = 100.0 * (ww * hh) / (fw * fh)
                if now - last_report >= 0.2:
                    print(f"visage: cx={cx:6.1f} cy={cy:6.1f} | "
                          f"nx={nx:+.2f} ny={ny:+.2f} | aire={area_pct:4.1f}% | "
                          f"P1={disp_fps:4.1f}fps P2={det_fps:4.1f}fps")
                    last_report = now
            elif args.no_display and now - last_report >= 1.0:
                print(f"(aucun visage) P1={disp_fps:4.1f}fps P2={det_fps:4.1f}fps")
                last_report = now

            if not args.no_display:
                for (x, y, ww, hh) in faces:
                    is_main = main_face is not None and (x, y, ww, hh) == main_face
                    color = (0, 255, 0) if is_main else (0, 180, 255)
                    cv2.rectangle(frame, (x, y), (x + ww, y + hh), color, 2)
                if main_face is not None:
                    x, y, ww, hh = main_face
                    cx, cy = int(x + ww / 2), int(y + hh / 2)
                    nx = (cx - fw / 2.0) / (fw / 2.0)
                    ny = (cy - fh / 2.0) / (fh / 2.0)
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                    cv2.putText(frame, f"nx={nx:+.2f} ny={ny:+.2f}",
                                (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)
                cv2.line(frame, (fw // 2, 0), (fw // 2, fh), (80, 80, 80), 1)
                cv2.line(frame, (0, fh // 2), (fw, fh // 2), (80, 80, 80), 1)
                cv2.putText(frame,
                            f"P1(affichage)={disp_fps:.0f}fps  P2(detect)={det_fps:.0f}fps  visages={len(faces)}",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
                cv2.imshow("face_detect  P1 affichage / P2 detection (q=quitter)", frame)
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        with shared.lock:
            shared.stop = True
        worker.join(timeout=1.0)
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
