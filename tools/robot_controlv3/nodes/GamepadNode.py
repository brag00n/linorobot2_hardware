r"""GamepadNode - node manette de jeu (pygame) : publie /joy.

Source de teleop portable : lit une manette via **pygame/SDL** (Windows ET
Linux/RPi, contrairement a l'API XInput ctypes Windows-only qui a servi aux
essais materiels) et publie une image brute de son etat (axes, boutons, croix)
sur /joy. Toute la traduction en actions robot (deplacement, camera, modes)
se fait cote Core, la ou vit deja le dispatch clavier -- ce node reste un
publieur d'entree pur, comme un node teleop qui publierait sensor_msgs/Joy.

Manette de reference : **Under Control 2919** (manette sans fil tierce pour
Nintendo Switch, PAS une officielle Nintendo), ou toute manette XInput/Xbox.
Appairage Bluetooth (mode XInput OBLIGATOIRE pour etre vue du PC) :
  1. eteindre la manette (Home appui long) ;
  2. rallumer en maintenant **Home + X** jusqu'au clignotement (= mode XInput /
     emulation Xbox 360 ; Home+Y = DirectInput, Home+A = Switch, Home+B = Android) ;
  3. Windows -> Bluetooth -> Ajouter un appareil (apparait en `VID_045E&PID_02E0`).
  /!\ En mode Switch, Windows ne voit RIEN au scan Bluetooth. Sous Linux/RPi :
  idem via bluetoothctl, puis lue par pygame/SDL (aucun code specifique OS).

Portabilite RPi headless : `SDL_VIDEODRIVER=dummy` est pose avant l'init pygame
(aucun ecran requis). `pygame` est importe en OPTIONNEL : absent -> le node
logge un avertissement, publie `connected=False`, et l'app tourne normalement
(la manette est simplement inactive). Manette debranchee a chaud -> reouverture
tentee periodiquement (hot-plug SDL).

Sous ROS2 : publisher d'un sensor_msgs/Joy a cadence fixe (comme joy_node).
"""
import os

from ..roslite import Node
from ..msgs import JoyMsg

# --- Mapping XInput/SDL2 standard (manette Xbox 360, ce qu'emule la 2919) ------
# Indices d'axes (floats [-1, 1]). Surchargeables via le bloc profil si un autre
# modele differe ; `--gamepad-probe` affiche les indices reellement vus.
AX_LX = 0            # stick gauche X (- gauche / + droite)
AX_LY = 1            # stick gauche Y (- haut  / + bas)   -> avant = -LY
AX_RX = 2            # stick droit  X
AX_RY = 3            # stick droit  Y
AX_LT = 4            # gachette gauche ZL/LT (repos -1 -> +1 enfoncee)
AX_RT = 5            # gachette droite ZR/RT (repos -1 -> +1 enfoncee)

# Indices de boutons (0/1).
BTN_A = 0            # A  (bas)
BTN_B = 1            # B  (droite)
BTN_X = 2            # X  (gauche)
BTN_Y = 3            # Y  (haut)
BTN_LB = 4           # gachette haute gauche (L / LB)
BTN_RB = 5           # gachette haute droite (R / RB)
BTN_BACK = 6         # - (Back / View / Select)
BTN_START = 7        # + (Start / Menu)
BTN_L3 = 8           # clic stick gauche
BTN_R3 = 9           # clic stick droit

HAT_DPAD = 0         # croix directionnelle -> get_hat(0) = (x, y), y+ = haut


class GamepadNode(Node):
    """Publie l'etat brut d'une manette (pygame) sur /joy. Possede sa ressource SDL."""

    #: periode min entre deux tentatives de (re)ouverture de la manette (s)
    REOPEN_PERIOD_S = 2.0

    def __init__(self, index=0, deadzone=0.12, expo=0.35, probe=False):
        super().__init__("gamepad")
        self.index = index
        # deadzone/expo sont portes ici pour info + `--gamepad-probe` ; l'application
        # reelle (zone morte, expo) se fait cote Core sur les floats publies.
        self.deadzone = deadzone
        self.expo = expo
        self.probe = probe
        self._out = self.create_output("/joy", JoyMsg)
        self._pygame = None       # module pygame (None si import KO)
        self._js = None           # objet Joystick ouvert (None si absent)
        self._name = ""
        self._seq = 0
        self._last_open = 0.0     # horodatage derniere tentative d'ouverture
        self._warned = False      # avertissement "pas de pygame" logge une fois

    # --- cycle de vie -------------------------------------------------------
    def on_start(self):
        """Import optionnel de pygame + init SDL headless, puis 1re ouverture."""
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")   # RPi/serveur sans ecran
        os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
        try:
            import pygame  # optionnel : absent -> manette inactive, app OK
            pygame.init()
            pygame.joystick.init()
            self._pygame = pygame
        except Exception as e:                              # noqa: BLE001 (non fatal)
            print(f"[gamepad] pygame indisponible ({e}) -> manette desactivee")
            self._pygame = None
            return
        self._open()

    def process(self):
        """Lit la manette (non bloquant) et publie un JoyMsg sur /joy."""
        pg = self._pygame
        self._seq += 1
        if pg is None:
            self._out.set(JoyMsg(seq=self._seq, connected=False))
            return

        pg.event.pump()   # draine les evenements SDL (hot-plug + etats a jour)

        # (Re)ouverture paresseuse si la manette est absente ou vient d'arriver.
        if self._js is None:
            self._open()

        if self._js is None:
            self._out.set(JoyMsg(seq=self._seq, connected=False))
            return

        try:
            axes = tuple(self._js.get_axis(i) for i in range(self._js.get_numaxes()))
            buttons = tuple(int(self._js.get_button(i))
                            for i in range(self._js.get_numbuttons()))
            hats = tuple(self._js.get_hat(i) for i in range(self._js.get_numhats()))
        except Exception:                                   # noqa: BLE001
            # Manette arrachee en cours de lecture -> on la lache proprement.
            self._close_js()
            self._out.set(JoyMsg(seq=self._seq, connected=False))
            return

        if self.probe:
            print(f"[gamepad] axes={['%.2f' % a for a in axes]} "
                  f"buttons={buttons} hats={hats}")

        self._out.set(JoyMsg(seq=self._seq, connected=True, name=self._name,
                             axes=axes, buttons=buttons, hats=hats))

    def on_stop(self):
        self._close_js()
        if self._pygame is not None:
            try:
                self._pygame.joystick.quit()
                self._pygame.quit()
            except Exception:                               # noqa: BLE001
                pass

    # --- internes -----------------------------------------------------------
    def _open(self):
        """Tente d'ouvrir la manette `index` (throttle REOPEN_PERIOD_S)."""
        pg = self._pygame
        if pg is None:
            return
        import time
        now = time.monotonic()
        if now - self._last_open < self.REOPEN_PERIOD_S:
            return
        self._last_open = now
        try:
            # Re-init du sous-systeme pour rafraichir le comptage (hot-plug).
            pg.joystick.quit()
            pg.joystick.init()
            if pg.joystick.get_count() <= self.index:
                return
            js = pg.joystick.Joystick(self.index)
            js.init()
            self._js = js
            self._name = js.get_name()
            print(f"[gamepad] manette connectee : {self._name} "
                  f"(axes={js.get_numaxes()} boutons={js.get_numbuttons()} "
                  f"croix={js.get_numhats()})")
        except Exception as e:                              # noqa: BLE001
            self._js = None
            if not self._warned:
                print(f"[gamepad] ouverture impossible ({e})")
                self._warned = True

    def _close_js(self):
        if self._js is not None:
            try:
                self._js.quit()
            except Exception:                               # noqa: BLE001
                pass
            self._js = None
            self._name = ""
