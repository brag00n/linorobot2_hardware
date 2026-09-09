r"""_rpc.py - Briques partagees des deux serveurs MCP de robot_control.

- Boucle JSON-RPC 2.0 sur stdin/stdout (messages delimites par des sauts de ligne),
  stdlib seule, identique a l'ancienne implementation manuelle (rc_mcp_server.py /
  ros_mcp_server.py) : initialize / notifications-initialized / ping / tools-list /
  tools-call. Factorisee ici pour que analysis.py et action.py la partagent.
- Resolution du dossier de logs (RC_LOG_DIR ou robot_control/logs) et lecture des
  fichiers de telemetrie ecrits par l'app (classe Logs). AUCUNE ouverture de port
  ni de camera : uniquement de la lecture de fichiers.

stdout est reserve au JSON-RPC : tout message de service part sur stderr (log()).
"""
import json
import os
import sys
import time

SERVER_VERSION = "2.0.0"
DEFAULT_PROTOCOL = "2025-06-18"

# tools/robot_control/mcp/_rpc.py -> dossier robot_control (deux niveaux au-dessus).
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def resolve_log_dir():
    """Dossier de telemetrie : RC_LOG_DIR sinon robot_control/logs (comme avant)."""
    return os.environ.get("RC_LOG_DIR", os.path.join(_PKG_DIR, "logs"))


def make_log(tag):
    """Fabrique une fonction de log prefixee vers stderr (stdout = JSON-RPC)."""
    def _log(*a):
        print("[%s]" % tag, *a, file=sys.stderr, flush=True)
    return _log


# ---------------------------------------------------------------------------
# Lecture des fichiers de telemetrie (robuste aux ecritures concurrentes)
# ---------------------------------------------------------------------------
class Logs:
    """Acces en LECTURE SEULE aux fichiers ecrits par l'app dans log_dir."""

    def __init__(self, log_dir):
        self.dir = log_dir
        self.jsonl = os.path.join(log_dir, "robot_control.jsonl")
        self.state = os.path.join(log_dir, "state.json")
        self.snapshot = os.path.join(log_dir, "latest.jpg")
        self.cursor = os.path.join(log_dir, "mcp_cursor.json")

    def read_state(self):
        try:
            with open(self.state, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None

    def read_records(self, limit=None, types=None, since=None):
        """Enregistrements JSONL (recents en dernier). types : set a conserver ;
        since : t >= since. Lignes non parsables (ecriture en cours) ignorees."""
        recs = []
        try:
            with open(self.jsonl, encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        r = json.loads(line)
                    except Exception:
                        continue
                    if types is not None and r.get("type") not in types:
                        continue
                    if since is not None and r.get("t", 0) < since:
                        continue
                    recs.append(r)
        except FileNotFoundError:
            return []
        except Exception:
            return recs
        if limit is not None and len(recs) > limit:
            recs = recs[-limit:]
        return recs

    def load_cursor(self):
        try:
            with open(self.cursor, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def fmt_age(self, path):
        try:
            return round(time.time() - os.path.getmtime(path), 1)
        except Exception:
            return None

    @staticmethod
    def running(state):
        """L'app est-elle vivante ? (state.json mis a jour il y a < 5 s)."""
        if not state:
            return False
        try:
            return (time.time() - state.get("updated", 0)) < 5.0
        except Exception:
            return False


# ---------------------------------------------------------------------------
# Boucle JSON-RPC (stdio, messages delimites par \n)
# ---------------------------------------------------------------------------
def _send(obj):
    data = json.dumps(obj, separators=(",", ":")).encode("utf-8") + b"\n"
    sys.stdout.buffer.write(data)                     # bytes -> pas de conversion \r\n
    sys.stdout.buffer.flush()


def serve(server_name, tools, handlers):
    """Boucle JSON-RPC MCP. handlers : {nom -> callable(args) -> texte}."""
    def result_text(mid, text, is_error=False):
        _send({"jsonrpc": "2.0", "id": mid,
               "result": {"content": [{"type": "text", "text": text}],
                          "isError": is_error}})

    for raw in sys.stdin.buffer:
        line = raw.decode("utf-8", "replace").strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except Exception:
            continue
        mid = msg.get("id")
        method = msg.get("method")
        if method == "initialize":
            pv = (msg.get("params") or {}).get("protocolVersion", DEFAULT_PROTOCOL)
            _send({"jsonrpc": "2.0", "id": mid,
                   "result": {"protocolVersion": pv,
                              "capabilities": {"tools": {}},
                              "serverInfo": {"name": server_name,
                                             "version": SERVER_VERSION}}})
        elif method == "notifications/initialized":
            pass                                       # notification : pas de reponse
        elif method == "ping":
            _send({"jsonrpc": "2.0", "id": mid, "result": {}})
        elif method == "tools/list":
            _send({"jsonrpc": "2.0", "id": mid, "result": {"tools": tools}})
        elif method == "tools/call":
            p = msg.get("params") or {}
            name = p.get("name")
            args = p.get("arguments") or {}
            handler = handlers.get(name)
            if handler is None:
                _send({"jsonrpc": "2.0", "id": mid,
                       "error": {"code": -32602, "message": "outil inconnu: %s" % name}})
                continue
            try:
                result_text(mid, handler(args))
            except Exception as e:
                result_text(mid, "erreur outil %s: %s" % (name, e), is_error=True)
        elif mid is not None:                          # requete de methode inconnue
            _send({"jsonrpc": "2.0", "id": mid,
                   "error": {"code": -32601, "message": "methode inconnue: %s" % method}})
        # sinon : notification inconnue -> ignore
