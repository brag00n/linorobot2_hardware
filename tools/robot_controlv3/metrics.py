r"""MetricsConfig - activation centralisee des metriques (HMI / MCP / log).

Objectif perf : le FPS de la boucle d'affichage P1 depend surtout du DESSIN HUD et
des ecritures de telemetrie faites sur le thread P1. On veut donc pouvoir COUPER une
metrique non indispensable, par groupe ou par type, et par cible (HMI, MCP, log).

Modele (volontairement SANS agregation : moyenner/fenetrer sur le thread P1 couterait
du FPS, a contre-emploi ; l'agregation vit cote consommateur - encSpeed, MCP analyze) :

  - NOMMAGE  : chaque metrique appartient a un GROUPE `<source>_<domaine>` (ex.
               stm32_rpm, grove_irdist) lui-meme TYPE (perf/quality/sensor/state/
               actuator/link/event). Le nom journalise est `<groupe>_<type>`.
  - NIVEAU   : off < info < debug. Chaque groupe a un niveau requis (info par defaut) ;
               une cible sous ce niveau n'affiche/ne journalise pas.
  - CIBLE    : hmi (dessin), mcp (state.json/status), log (ligne jsonl). Independantes.

Pilotage : `--no-metric <spec>` au lancement + commande MCP `set_metrics <spec> on|off`
drainee sur le thread principal. `spec` = `<group|type>` (toutes cibles) ou
`<target>:<group|type>` (une cible). `enabled(group, target)` est le seul test appele
sur le chemin chaud (dict lookups, O(1)).
"""

LEVELS = {"off": 0, "info": 1, "debug": 2}
TARGETS = ("hmi", "mcp", "log")

# Registre des groupes connus : nom -> (type, niveau_requis). Couvre l'existant migre
# + les nouveautes (stm32_rpm, grove_irdist). Un groupe absent d'ici est traite en
# `info`/type inconnu (jamais bloquant : enabled() renvoie True par defaut).
REGISTRY = {
    "cam_fps":       ("perf",     "info"),
    "cam_detect":    ("state",    "info"),
    "stm32_motor":   ("state",    "info"),
    "stm32_servo":   ("actuator", "info"),
    "stm32_batt":    ("link",     "info"),
    "stm32_imu":     ("sensor",   "info"),
    "stm32_rpm":     ("sensor",   "info"),
    "esp32_motor":   ("state",    "info"),
    "esp32_batt":    ("link",     "info"),
    "esp32_imu":     ("sensor",   "info"),
    "esp32_mag":     ("sensor",   "info"),
    "esp32_rpm":     ("sensor",   "info"),
    "teensy_motor":  ("state",    "info"),
    "teensy_batt":   ("link",     "info"),
    "teensy_imu":    ("sensor",   "info"),
    "teensy_rpm":    ("sensor",   "info"),
    "grove_ultra":   ("sensor",   "info"),
    "grove_imu":     ("sensor",   "info"),
    "grove_irdist":  ("sensor",   "info"),
    "recog_badge":   ("quality",  "info"),
    "train_log":     ("event",    "info"),
    "gamepad_input": ("actuator", "info"),
}


class MetricsConfig:
    """Etat d'activation des metriques, interrogeable par groupe + cible.

    Par defaut tout est actif (niveau `info` sur les 3 cibles). On DESACTIVE via
    apply(spec, on=False) : `spec` designe un groupe ou un type, eventuellement
    prefixe d'une cible. Seule une exclusion explicite coupe une metrique.
    """

    def __init__(self):
        # exclusions : cible -> ensemble de groupes/types coupes (chaine)
        self._off = {t: set() for t in TARGETS}
        # niveau courant par cible (une metrique de niveau > seuil n'est pas emise)
        self._level = {t: LEVELS["debug"] for t in TARGETS}

    # --- chemin chaud -------------------------------------------------------
    def enabled(self, group, target):
        """La metrique `group` doit-elle etre produite pour `target` (hmi/mcp/log) ?"""
        off = self._off.get(target)
        if off is None:
            return True
        if group in off:
            return False
        gtype, glevel = REGISTRY.get(group, (None, "info"))
        if gtype is not None and gtype in off:
            return False
        return LEVELS.get(glevel, 1) <= self._level.get(target, 2)

    # --- pilotage -----------------------------------------------------------
    @staticmethod
    def _split(spec):
        """`<target>:<key>` -> (target, key) ; `<key>` -> (None, key)."""
        spec = str(spec).strip()
        if ":" in spec:
            head, _, tail = spec.partition(":")
            if head in TARGETS:
                return head, tail.strip()
        return None, spec

    def apply(self, spec, on):
        """Active (on=True) ou coupe (on=False) un groupe/type, sur 1 ou toutes les cibles.

        Retourne (ok, message) pour le retour MCP/CLI.
        """
        target, key = self._split(spec)
        if not key:
            return False, "spec vide"
        known = set(REGISTRY) | {t for (t, _) in REGISTRY.values()}
        if key not in known:
            return False, ("clef inconnue '%s' (groupes/types : %s)"
                           % (key, ", ".join(sorted(known))))
        tgts = (target,) if target else TARGETS
        for t in tgts:
            if on:
                self._off[t].discard(key)
            else:
                self._off[t].add(key)
        return True, ("%s %s pour %s" % (key, "actif" if on else "coupe",
                                         target or "toutes cibles"))

    def parse_no_metric(self, specs):
        """Applique une liste de `--no-metric <spec>` (coupures). Ignore les invalides."""
        for spec in (specs or []):
            self.apply(spec, on=False)
        return self

    def summary(self):
        """Etat lisible (pour status/CLI) : cibles ayant des coupures."""
        parts = []
        for t in TARGETS:
            if self._off[t]:
                parts.append("%s:{%s}" % (t, ",".join(sorted(self._off[t]))))
        return "  ".join(parts) if parts else "tout actif"
