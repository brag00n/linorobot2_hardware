r"""telemetry - Journal structure du comportement de l'outil robot_control.

L'app (robot_control.main) detient COM4 ET la camera : aucun autre process ne
peut les ouvrir. Pour permettre une ANALYSE externe (ex : serveur MCP), l'app
ecrit son comportement sur disque, et l'observateur se contente de lire :

  logs/robot_control.jsonl : un enregistrement JSON par ligne, horodate (t).
      types : event, tx_motor, tx_servo, rx (sub=speed|imu|encoder),
              detect (nx/ny/aire/fps), track (erreur->pas->angle), heartbeat.
  logs/state.json          : dernier enregistrement de chaque type + compteurs
                             (photo instantanee, pour un status rapide).
  logs/latest.jpg          : derniere image annotee (throttlee), pour analyse.

Ecritures thread-safe (verrou). Rotation par taille en FENETRE GLISSANTE :
le fichier vif est plafonne a max_bytes ; a la bascule il devient
robot_control.jsonl.1 (backup unique, l'ancien .1 est ecrase). Empreinte
disque bornee = fichier vif + 1 backup <= 2 x max_bytes (budget total). Tout
est desactivable (enabled=False -> no-op). Le serveur MCP NE DEPEND PAS de ce
module : il lit seulement les fichiers ci-dessus (et .1 pour l'historique).
"""
import json
import os
import threading
import time


class Telemetry:
    def __init__(self, log_dir=None, enabled=True, snapshot_period=0.5,
                 rx_min_period=0.2, max_bytes=5_000_000):
        self.enabled = enabled
        if log_dir is None:
            here = os.path.dirname(os.path.abspath(__file__))
            log_dir = os.path.join(here, "logs")
        self.log_dir = log_dir
        self.jsonl_path = os.path.join(log_dir, "robot_control.jsonl")
        self.state_path = os.path.join(log_dir, "state.json")
        self.snapshot_path = os.path.join(log_dir, "latest.jpg")
        self.snapshot_period = snapshot_period
        self.rx_min_period = rx_min_period
        self.max_bytes = max_bytes

        self._lock = threading.RLock()
        self._fh = None
        self._state = {}                 # cle -> dernier enregistrement
        self._counts = {}                # type -> nombre
        self._last_rx = {}               # sub -> derniere ecriture (throttle)
        self._last_snap = 0.0
        self._last_state_flush = 0.0
        self.start_t = time.time()

        if self.enabled:
            os.makedirs(log_dir, exist_ok=True)
            self._fh = open(self.jsonl_path, "a", encoding="utf-8")

    # --- ecriture bas niveau ------------------------------------------------
    def _write(self, rec):
        self._fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
        self._fh.flush()

    def _maybe_rotate(self):
        try:
            if self._fh.tell() < self.max_bytes:
                return
        except Exception:
            return
        try:
            self._fh.close()
            bak = self.jsonl_path + ".1"
            if os.path.exists(bak):
                os.remove(bak)
            os.replace(self.jsonl_path, bak)
        except Exception:
            pass
        self._fh = open(self.jsonl_path, "a", encoding="utf-8")

    def _flush_state(self, t):
        snap = {"updated": round(t, 3),
                "uptime": round(t - self.start_t, 1),
                "pid": os.getpid(),
                "counts": dict(self._counts),
                "last": self._state}
        try:
            tmp = self.state_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(snap, f, separators=(",", ":"))
            os.replace(tmp, self.state_path)
        except Exception:
            pass
        self._last_state_flush = t

    # --- API publique -------------------------------------------------------
    def log(self, type, **fields):
        if not self.enabled:
            return
        t = time.time()
        rec = {"t": round(t, 4), "type": type}
        rec.update(fields)
        key = type if "sub" not in fields else f"{type}:{fields['sub']}"
        with self._lock:
            self._counts[type] = self._counts.get(type, 0) + 1
            self._state[key] = rec
            try:
                self._write(rec)
                self._maybe_rotate()
            except Exception:
                pass
            if t - self._last_state_flush >= 1.0:
                self._flush_state(t)

    def log_rx(self, sub, **fields):
        """Telemetrie carte haut debit : etat toujours a jour, ecriture throttlee."""
        if not self.enabled:
            return
        t = time.time()
        with self._lock:
            # etat toujours frais (meme si on n'ecrit pas la ligne)
            self._state[f"rx:{sub}"] = {"t": round(t, 4), "type": "rx",
                                        "sub": sub, **fields}
            if t - self._last_rx.get(sub, 0.0) < self.rx_min_period:
                if t - self._last_state_flush >= 1.0:
                    self._flush_state(t)
                return
            self._last_rx[sub] = t
        self.log("rx", sub=sub, **fields)

    def snapshot(self, frame):
        """Ecrit la derniere image annotee (throttlee a snapshot_period)."""
        if not self.enabled:
            return
        t = time.time()
        if t - self._last_snap < self.snapshot_period:
            return
        self._last_snap = t
        try:
            import cv2
            tmp = self.snapshot_path + ".tmp"
            if cv2.imwrite(tmp, frame):
                os.replace(tmp, self.snapshot_path)
        except Exception:
            pass

    def close(self):
        if not self.enabled:
            return
        try:
            self.log("event", msg="telemetry_stop")
            with self._lock:
                self._flush_state(time.time())
                if self._fh:
                    self._fh.close()
        except Exception:
            pass
