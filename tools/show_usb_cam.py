#!/usr/bin/env python3
"""Affiche en direct le flux d'une camera USB UVC via ffplay (dshow, Windows).

Ouvre une fenetre temps reel. Par defaut : "HD Web Camera" en MJPEG 1280x720@30
(format compresse -> faible charge). Fermer la fenetre (ou Ctrl+C) pour quitter.

Usage :
  python show_usb_cam.py                       # HD Web Camera, MJPEG 720p30
  python show_usb_cam.py --name "USB 2.0 Camera" --size 640x480 --codec mjpeg
  python show_usb_cam.py --list                # liste les cameras DirectShow
"""
import argparse
import glob
import os
import subprocess
import sys


def find_tool(name):
    """Localise ffplay/ffmpeg (winget) sans dependre du PATH de la session."""
    # PATH d'abord
    from shutil import which
    p = which(name)
    if p:
        return p
    # Installation winget typique
    base = os.path.expandvars(r"%LOCALAPPDATA%\Microsoft\WinGet\Packages")
    hits = glob.glob(os.path.join(base, "**", name + ".exe"), recursive=True)
    return hits[0] if hits else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="HD Web Camera", help="nom DirectShow de la camera")
    ap.add_argument("--size", default="1280x720", help="resolution, ex 640x480")
    ap.add_argument("--framerate", default="30")
    ap.add_argument("--codec", default="mjpeg", choices=["mjpeg", "yuyv422", "auto"],
                    help="format demande a la camera (mjpeg = compresse)")
    ap.add_argument("--flip", default="none", choices=["none", "v", "h", "180"],
                    help="retourne l'image : v=vertical, h=horizontal, 180=les deux")
    ap.add_argument("--list", action="store_true", help="liste les cameras et quitte")
    args = ap.parse_args()

    ffplay = find_tool("ffplay")
    ffmpeg = find_tool("ffmpeg")
    if not ffplay:
        sys.exit("ffplay introuvable (installe via: winget install Gyan.FFmpeg).")

    if args.list:
        subprocess.run([ffmpeg or "ffmpeg", "-hide_banner", "-f", "dshow",
                        "-list_devices", "true", "-i", "dummy"])
        return

    cmd = [ffplay, "-hide_banner", "-f", "dshow"]
    if args.codec != "auto":
        cmd += ["-vcodec", args.codec]
    cmd += ["-video_size", args.size, "-framerate", args.framerate,
            "-i", f"video={args.name}"]
    vf = {"v": "vflip", "h": "hflip", "180": "vflip,hflip"}.get(args.flip)
    if vf:
        cmd += ["-vf", vf]

    print("Lancement :", " ".join(f'"{c}"' if " " in c else c for c in cmd), flush=True)
    print("Ferme la fenetre video (ou Ctrl+C) pour quitter.", flush=True)
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
