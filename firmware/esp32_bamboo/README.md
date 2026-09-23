# Firmware `esp32_bamboo` — carte WaveShare « General Driver for Robots » (Rev1.2)

Firmware de la carte de contrôle **WaveShare General Driver for Robots**. Ce document décrit la
**carte** et son **microcode**, indépendamment de tout robot : un robot qui embarque cette carte la
référence par l'identifiant de version ci-dessous.

> ### Identité du microcode
> ```
> ESP32-WROOM-32UE_bamboo v0.1.0
> ```
> Définie dans [`include/fw_version.h`](include/fw_version.h) et **retournée par le microcode**
> (§ 5.1). C'est le seul identifiant à citer pour désigner une révision de ce firmware.

- **Nom constructeur** : WaveShare *General Driver for Robots*, révision **Rev1.2**.
- **Contrôleur** : **ESP32-WROOM-32UE** (Wi-Fi 2,4 GHz + BLE + ESP-NOW, antenne externe IPEX1).
- **Rôle** : pont télémétrie / actionnement **MAVLink v2** (dialecte `bamboo`) vers le RPi4, qui
  porte la pile ROS2. Développement sous **PlatformIO** (Arduino-ESP32).
- **Sources matériel** : wiki officiel WaveShare + lecture du **schéma officiel Rev1.2**
  (sept. 2026). Les brochages marqués **(schéma)** viennent du schéma ; le wiki ne publie pas le
  détail broche à broche.

> ⚠️ **Deux points structurants**, hérités du choix du pont TB6612FNG :
> - **4 moteurs entraînés, 2 voies d'encodeur seulement** (une par côté) → l'odométrie ne
>   dispose que de 2 mesures réelles (gauche / droite). Voir § 4.2.
> - **2× CP2102 avec le même `VID:PID = 10C4:EA60`** → désambiguïsation par **numéro de série**
>   (règle udev), jamais par VID:PID. Voir § 4.4.

---

## 1. Caractéristiques générales

| Élément | Valeur |
|---|---|
| Contrôleur | ESP32-WROOM-32UE (Wi-Fi 2,4 GHz, Bluetooth, ESP-NOW) |
| Antenne | connecteur **IPEX1** (repère n° 2), antenne externe |
| Alimentation d'entrée | **DC 7–13 V** (batterie **2S / 3S** Li-ion) |
| Régulateur 5 V | **DC-DC MP8759**, rail principal `NL5V`, **5 V / 5 A** (schéma) |
| Régulateur 3,3 V | **AMS1117-3.3** (schéma) |
| Pont moteur | **TB6612FNG** — double pont-H, **2 canaux** (A et B) |
| Servos série | bus **ST3215** (série demi-duplex, jusqu'à 253 servos, ~5 A) |
| Monitoring alim | **INA219** (tension **et** courant), I2C **0x42** |
| IMU | **QMI8658** (6 axes) + **AK09918C** (magnétomètre 3 axes) |
| Interfaces USB | **2× CP2102** (`10C4:EA60`) : un pour l'UART ESP32, un pour les données lidar |
| Extension hôte | **header 40 broches** (Raspberry Pi / Jetson / Sunrise X3) |
| Stockage | slot **carte TF/SD** |
| Circuit de flash | **auto-download** (EN/BOOT automatiques, aucune manipulation de bouton) |
| Dimensions | **65 × 65 mm**, trous de fixation 49 × 58 mm (Ø 3 mm) |

---

## 2. Cartographie des connecteurs (repères silkscreen)

Numéros = repères de la nomenclature « onboard resource » du wiki constructeur.

| N° | Nom / fonction | Type de connecteur | § détail |
|---|---|---|---|
| 2 | Connecteur antenne Wi-Fi | IPEX1 | — |
| 3 | Interface **LIDAR** | PH2.0 **4P** (= **H7**, schéma) | 4.3 |
| 4 | Extension **I2C** (OLED / capteurs) | header 4 broches | 4.7 |
| 5 | Bouton **Reset** (redémarre l'ESP32) | bouton | — |
| 6 | Bouton **Download** (mode flash au boot) | bouton | — |
| 7 | Circuit régulateur **5 V** (alim hôte) | — | — |
| 8 | Type-C **« LIDAR »** (données lidar → hôte) | USB Type-C | 4.4 |
| 9 | Type-C **« USB »** (UART ESP32 + upload) | USB Type-C | 4.4 |
| 10 | **Entrée alimentation** DC 7–13 V | **XH2.54** 2P | 4.1 |
| 11 | INA219 (monitoring V + I) | puce I2C | 4.7 |
| 12 | Interrupteur **Power ON/OFF** | switch | 4.1 |
| 13 | Interface **servo bus ST3215** | connecteur série servo | 4.5 |
| **14** | **Moteur groupe B**, sans encodeur | PH2.0 **2P** | 4.2 |
| **15** | **Moteur groupe A**, avec encodeur | PH2.0 **6P** | 4.2 |
| **16** | **Moteur groupe A**, sans encodeur | PH2.0 **2P** | 4.2 |
| **17** | **Moteur groupe B**, avec encodeur | PH2.0 **6P** | 4.2 |
| 18 | AK09918C (compas 3 axes) | puce I2C | 4.7 |
| 19 | QMI8658 (IMU 6 axes) | puce I2C | 4.7 |
| 20 | TB6612FNG (pilote moteurs) | puce | 4.2 |
| 21 | Circuit de contrôle servo série | — | 4.5 |
| 22 | Slot **carte TF/SD** | micro-SD | — |
| 23 | **Header 40 broches** (extension hôte) | 2×20 pas 2,54 mm | 4.8 |
| 24 | Header 40 broches (accès broches hôte) | 2×20 pas 2,54 mm | 4.8 |
| 25 | CP2102 — UART↔USB **données lidar** | puce | 4.3 |
| 26 | CP2102 — UART↔USB **comm ESP32** | puce | 4.4 |
| 27 | Circuit **auto-download** (EN/BOOT auto) | — | 6 |

> ℹ️ **Il n'y a pas de connecteur caméra** sur cette carte (ni CSI, ni DVP) — voir § 4.6.

---

## 3. Vue d'ensemble des chaînes

```
  Batterie 7-13 V --XH2.54(10)--[SW(12)]--+--> TB6612FNG (20) --> ports moteurs (14/15/16/17)
                                          +--> bus servo ST3215 (13)      [non pilote]
                                          +--> MP8759 --> NL5V 5V/5A --+--> header 40P (23/24) -> RPi4
                                          |                            +--> port LIDAR H7 (3), broche 5V
                                          |                            +--> 2x CP2102 (25/26)
                                          +--> AMS1117 --> 3,3 V logique + encodeurs

  LIDAR --H7(3).CP_RX--> CP2102 U3 (25) --> Type-C « LIDAR » (8) --USB--> RPi4  [l'ESP32 ne lit pas]
  ESP32 --UART---------> CP2102 U2 (26) --> Type-C « USB »   (9) --USB--> RPi4  [MAVLink 921600]
```

---

## 4. Brochage détaillé, connecteur par connecteur

### 4.1 Entrée alimentation — n° 10 (XH2.54 2P)

| Broche | Signal | Détail |
|---|---|---|
| 1 | **V+** | **7–13 V** non régulé |
| 2 | **GND** | masse |

- Commutée par l'interrupteur **Power ON/OFF** (n° 12).
- Alimente **directement** (non régulé) le TB6612FNG et le bus servo ST3215 ; et via **MP8759**
  le rail **5 V** (`NL5V`, hôte + lidar + CP2102), via **AMS1117** le **3,3 V** logique.
- Alimenter la carte hôte depuis ce rail est possible (header 40P, § 4.8) mais l'expose au bruit
  moteur ; une alimentation séparée de l'hôte est préférable — voir la fiche UPS en § 8.

### 4.2 Ports moteurs — n° 14/15/16/17 — M1 … M4

Le TB6612FNG n'a que **2 canaux** (A et B). Les 4 connecteurs moteur forment donc **2 groupes**,
chaque groupe partageant un canal :

| Groupe | Connecteur **6P** (avec encodeur) | Connecteur **2P** (sans encodeur) | Canal TB6612 |
|---|---|---|---|
| **A** | n° **15** | n° **16** | un canal |
| **B** | n° **17** | n° **14** | l'autre canal |

**Brochage du connecteur 6P (n° 15 et 17)** — PH2.0 6 points *(schéma, à confirmer carte en main)* :

| Broche | Signal | Rôle |
|---|---|---|
| 1 | **M+** | phase moteur A |
| 2 | **M−** | phase moteur B |
| 3 | **VCC** | alimentation encodeur (3,3 V) |
| 4 | **GND** | masse encodeur |
| 5 | **C1** | voie encodeur A |
| 6 | **C2** | voie encodeur B |

**Brochage du connecteur 2P (n° 14 et 16)** — PH2.0 2 points :

| Broche | Signal | Rôle |
|---|---|---|
| 1 | **M+** | phase moteur A |
| 2 | **M−** | phase moteur B |

**Correspondance avec les voies M1…M4 du firmware.** Les quatre voies logiques du firmware
(`config/custom/bamboov310_config.h`) se replient **deux par deux** sur les deux canaux physiques :
M3 réutilise **exactement** les broches de M1, M4 celles de M2 — commande *et* encodeur.

| Voie firmware | Groupe physique | Encodeur propre | Conséquence |
|---|---|---|---|
| **M1** | groupe A, connecteur **6P** (n° 15) | **oui** (voie gauche réelle) | pilote le canal A |
| **M3** | groupe A, connecteur **2P** (n° 16) | **non** — recopie de M1 | même canal, même PWM que M1 |
| **M2** | groupe B, connecteur **6P** (n° 17) | **oui** (voie droite réelle) | pilote le canal B |
| **M4** | groupe B, connecteur **2P** (n° 14) | **non** — recopie de M2 | même canal, même PWM que M2 |

Ce n'est pas une convention logicielle mais un fait électrique : **écrire sur M3 écrit sur le même
canal TB6612 que M1**. Les deux moteurs d'un groupe suivent donc toujours la même consigne, et
`encoders[2]` / `encoders[3]` publiés par le firmware sont des **duplicatas** de `encoders[0]` /
`encoders[1]` — ne jamais les calibrer ni les moyenner.

> ⚠️ L'affectation **groupe A ↔ côté gauche** (et B ↔ droit) reste **à vérifier à l'observation** :
> le schéma ne nomme pas les canaux « gauche / droite ». Procédure, **roues surélevées** :
> piloter M1 seul à faible PWM et relever quel côté tourne. Toute inversion se corrige par
> `MOTORn_INV` / `MOTORn_ENCODER_INV` dans la configuration, **jamais** côté ROS (cela
> désynchroniserait PID et odométrie).

### 4.3 Port LIDAR — n° 3 = connecteur **H7** (PH2.0 4P)

Brochage *(schéma)* :

| Broche | Signal | Détail |
|---|---|---|
| 1 | **GND** | masse |
| 2 | **5V** | rail `NL5V` (MP8759, 5 V/5 A), **toujours actif** — EN tiré au + par **R4 1,5 MΩ**, non commuté par GPIO |
| 3 | **GND** | masse |
| 4 | **CP_RX** | données lidar → carte, via **R15 1 kΩ** série → **RXD du CP2102 U3** (n° 25) |

- **Chaîne mono-directionnelle** : le CP2102 U3 n'utilise que **RXD**, pas de TX →
  `H7.CP_RX → CP2102 U3 → Type-C « LIDAR » (n° 8) → USB hôte`. La carte ne peut donc **rien
  envoyer** au lidar : capteur auto-tournant à trame sortante seule uniquement.
- **L'ESP32 ne lit pas ce port.** Dans ce firmware `initLidar()` est un **stub vide**
  (`lib/lidar/lidar.cpp` : tout le chemin série est sous `#ifdef USE_LIDAR_UDP`, non défini ; le
  contrôle d'alimentation est sous `LIDAR_POWEROFF`, non défini non plus). Le lidar est un
  **passthrough matériel** vers l'hôte, invisible du microcode.
- **Pas de fusible par port** : la seule limite est les **5 A** internes du DC-DC, partagés avec
  le header 40P et les CP2102.
- **Câbler par fonction**, le connecteur du lidar n'étant pas un H7 : `lidar VCC → 5V`,
  `GND → GND`, `lidar Tx → CP_RX`. Données en **TTL 3,3 V**.
- Le port est **passif vis-à-vis du microcode** : rien à configurer côté firmware pour mettre un
  lidar en service, tout se passe côté hôte sur le Type-C n° 8.

### 4.4 Les deux Type-C — n° 8 (« LIDAR ») et n° 9 (« USB »)

| N° | Sérigraphie | Puce | Usage |
|---|---|---|---|
| **9** | **USB** | CP2102 **U2** (n° 26) | **UART de l'ESP32** : upload firmware **et** liaison MAVLink vers le RPi4 (**921600 bauds**) |
| **8** | **LIDAR** | CP2102 **U3** (n° 25) | sortie des trames lidar vers l'hôte (voir § 4.3) |

- Les deux sont des ports **USB device** vers le RPi4, et portent le **même `10C4:EA60`** →
  les distinguer par **numéro de série** dans la règle udev.
- Le circuit **auto-download** (n° 27) est actif sur le n° 9 : **DTR/RTS doivent rester
  désassertés** à l'ouverture du port, sinon l'ESP32 se réinitialise.

### 4.5 Servos — bus ST3215 (n° 13) — et l'absence de servo PWM

| Broche | Signal | Détail |
|---|---|---|
| 1 | **VIN** | **7–13 V non régulé** (directement l'entrée n° 10) |
| 2 | **GND** | masse |
| 3 | **Signal** | série **demi-duplex** |

- Jusqu'à **253 servos chaînés**, ~**5 A** au total (≈ 5 servos typiques).
- **La carte ne pilote pas de servo PWM** de type MG996R / MG90S ; le servo PWM recommandé par le
  constructeur est le WP90. Tout le pilotage servo passe par ce bus série.
- **Non exploité par ce firmware** : `MAV_CMD_DO_SET_SERVO` renvoie `MAV_RESULT_UNSUPPORTED`
  (`lib/_Connectors/ConnectorMavlink.cpp`), et le bloc `ENABLE_DEVICE_SERVO_MOTOR` compile
  `lib/pwm/` qui pilote un **PCA9685** externe (`PCA_BASE`, non défini ici) — donc `initPwm()`
  est inerte.

### 4.6 Caméra

**Cette carte n'a aucun connecteur caméra** : pas de nappe CSI, pas de bus DVP, aucune broche
caméra sur le header. L'ESP32-WROOM-32UE n'est pas un ESP32-CAM.

Toute imagerie relève donc de la **carte hôte** (caméra USB ou CSI côté hôte) : cette carte de
contrôle n'est pas sur ce chemin et son microcode n'a aucune fonction vidéo.

### 4.7 Extension I2C — n° 4

| Broche | Signal |
|---|---|
| 1 | **VCC** 3,3 V |
| 2 | **GND** |
| 3 | **SDA** |
| 4 | **SCL** |

Bus partagé avec les trois puces embarquées. Côté firmware, `SDA = GPIO 32`, `SCL = GPIO 33`
(`src/firmware.cpp`) :

| Composant | N° | Adresse I2C |
|---|---|---|
| INA219 (tension + courant) | 11 | **0x42** |
| QMI8658 (IMU 6 axes) | 19 | par défaut du pilote |
| AK09918C (magnétomètre) | 18 | par défaut du pilote |
| OLED SSD1306 (option, sur n° 4) | — | 0x3C typique |

### 4.8 Header 40 broches — n° 23/24

- Brochage **compatible Raspberry Pi** (2×20, pas 2,54 mm) : 5 V, 3,3 V, GND, GPIO, I2C, SPI, UART.
- Fournit le **5 V** à la carte hôte depuis `NL5V` — **même net que la broche 5V du port lidar**.
- Accepte un RPi, une Jetson Nano ou une Sunrise X3 Pi.
- Le contrôle passe par le **câble USB** du Type-C n° 9 (§ 4.4) et non par ce header. Mais le header
  **partage le net de l'UART de l'ESP32** (GPIO14/15 côté hôte), et c'est un piège coûteux, constaté
  sur ce robot le 2026-09-22.

  ⚠️ **Le RPi doit laisser son UART au repos, sinon l'hôte ne peut plus rien écrire dans la carte.**
  Avec `enable_uart=1` dans `/boot/config.txt`, GPIO14 est en fonction **UART TXD**, donc en sortie
  push-pull **maintenue à l'état haut** — même quand rien n'émet. Cette sortie écrase le CP2102 du
  Type-C n° 9, qui ne peut plus tirer le RXD de l'ESP32 vers le bas.

  | Observation | Interprétation |
  |---|---|
  | `esptool` : « Download mode successfully detected, but getting no sync reply: The serial TX path seems to be down. » | DTR/RTS passent (ils voyagent par l'USB), la voie **TX** est écrasée |
  | La carte reste parfaitement bavarde (télémétrie à 10 Hz lisible) | le sens carte → hôte n'emprunte pas ce net |
  | Aucun `cmd_vel` ni `PARAM_SET` n'aboutit | même cause : tout l'aller hôte → carte est muet |

  **D'où vient ce réglage** : ni du firmware RPi, ni de la console série — de **DietPi**. Sur un Pi 4
  (WiFi/BT intégré) l'UART primaire est le mini-UART `ttyS0`, **désactivé par défaut** côté firmware ;
  c'est `dietpi-set_hardware` qui injecte `enable_uart=1` sur les modèles RPi. Et le piège est que
  DietPi dissocie les deux réglages : la **console** série peut être coupée
  (`CONFIG_SERIAL_CONSOLE_ENABLE=0`, `console=tty1` seul, `serial-getty@*` masqués) alors que le
  **périphérique** UART reste actif. « Rien n'émet sur le port série » ne veut donc pas dire
  « la ligne est libre ».

  Diagnostic et remède, dans cet ordre :

  ```bash
  # 1. Prouver la panne sans rien écrire en flash (script du dépôt ROS) :
  docker compose --profile tools run --rm -e CHECK_ONLY=1 flash.esp32

  # 2. Libérer la ligne tout de suite, sans reboot (NON persistant) : exporter GPIO14 en
  #    entrée le retire de la fonction UART.
  sudo sh -c 'echo 14 > /sys/class/gpio/export'
  sudo sh -c 'echo in > /sys/class/gpio/gpio14/direction'
  # `chip_id` répond alors normalement (ESP32-D0WD-V3). Retour arrière : echo 14 > .../unexport

  # 3. Remède durable, par le toggle DietPi pour qu'il ne réinjecte pas la valeur :
  #    dietpi-config > Advanced Options > Serial/UART, puis reboot.
  ```

  Rien sur ce robot n'utilise l'UART du RPi : l'ESP32 **et** la carte capteurs passent par USB. Si un
  jour on voulait piloter la carte par l'UART de ce header plutôt que par l'USB, il faudrait faire
  l'inverse **et débrancher le Type-C n° 9** : les deux ne peuvent pas coexister sur ce net.

---

## 5. Fonctions du microcode

Transport de contrôle sélectionné à la compilation, trois options exclusives sur le **même UART
à 921600 bauds** (`src/firmware.cpp`) : micro-ROS (défaut), trames binaires maison
(`ENABLE_CONNECTOR_SERIAL_FRAME`), ou **MAVLink v2** (`ENABLE_MAVLINK`). La suite
décrit le mode **MAVLink**, celui que porte l'identifiant de version de cette carte.

### 5.1 Identité et cadences

| Élément | Valeur |
|---|---|
| **Identité microcode** | **`ESP32-WROOM-32UE_bamboo v0.1.0`** (`include/fw_version.h`) |
| Dialecte | MAVLink v2 `bamboo` (header-only, `firmware/_common/protocol/mavlink/generated/c`) |
| `sysid` | **2** (le RPi distingue les cartes par ce champ) |
| `compid` | **1** (`MAV_COMP_ID_AUTOPILOT1`) |
| Heartbeat | **1 Hz** |
| Boucle de contrôle | **10 Hz** (`ConnectorMavlink.cpp`) — et non les 50 Hz de `UPDATE_FREQ`, qui ne concernent que micro-ROS |
| Concurrence | aucune : ESP32 mono-thread (`loop()`), état TX/RX en portée fichier |

**Comment l'hôte obtient la version.** Le microcode la retourne sur deux messages complémentaires :

| Message | Contenu | Quand |
|---|---|---|
| `STATUSTEXT` (#253), sévérité `INFO` | la chaîne lisible `ESP32-WROOM-32UE_bamboo v0.1.0` | **une fois**, juste après le premier heartbeat (donc récupérée par un hôte branché à chaud, et visible au simple sniff de la liaison) |
| `AUTOPILOT_VERSION` (#148) | `flight_sw_version` = `(major<<24)|(minor<<16)|(patch<<8)`, `flight_custom_version` = 8 premiers octets du nom de carte, `capabilities` = `MAVLINK2 \| PARAM_FLOAT` | à la demande |

La demande se fait par `COMMAND_LONG` :

- `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) — demande standard, `ACK ACCEPTED` puis les deux messages ;
- `MAV_CMD_REQUEST_MESSAGE` (512) avec `param1 = 148` — même effet ; toute autre valeur de `param1`
  renvoie `UNSUPPORTED`, les autres messages étant déjà émis d'office par le tick.

**La version n'est pas un paramètre.** L'ordre des index de la table `PARAM` (§ 5.4) est un contrat de
fil identique à celui des autres cartes ; y ajouter une entrée le casserait.

**Règle de version** (semantique, détaillée dans `include/fw_version.h`) : `MAJOR` = rupture du
contrat de fil (ordre des index `PARAM`, dialecte, `sysid`) ; `MINOR` = fonction nouvelle compatible ;
`PATCH` = correction sans effet sur le fil.

### 5.2 Messages émis

`heartbeat`, `bamboo_wheel_state`, `attitude`, `sys_status`, `bamboo_encoders`, `bamboo_mag`,
`param_value`, `command_ack`, `statustext` + `autopilot_version` (identité, § 5.1).

### 5.3 Messages traités

`BAMBOO_CMD_VEL`, `BAMBOO_MOTOR_PWM`, `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET`,
`COMMAND_LONG`.

| Commande `COMMAND_LONG` | Statut |
|---|---|
| `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` | **supportée** |
| `MAV_CMD_PREFLIGHT_CALIBRATION` | **supportée** (recalibration gyro à la demande) |
| `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` | **supportée** — renvoie l'identité du microcode (§ 5.1) |
| `MAV_CMD_REQUEST_MESSAGE` | **supportée pour `param1 = 148`** (`AUTOPILOT_VERSION`) ; `UNSUPPORTED` sinon |
| `MAV_CMD_DO_SET_SERVO` | `UNSUPPORTED` (§ 4.5) |
| `MAV_CMD_PREFLIGHT_STORAGE` | `UNSUPPORTED` — **aucune persistance** (ni NVS, ni EEPROM) |
| `MAV_CMD_USER_1` / `USER_2` | `UNSUPPORTED` |

### 5.4 Table de paramètres — 19 entrées, **l'ordre est le contrat de fil**

`PARAM_REQUEST_READ` se fait **par index, `param_id` vide** : l'ordre ci-dessous est normatif et
**volontairement identique à celui du STM32**, pour que l'hôte n'ait qu'un seul codec.

| Index | Nom | Type |
|---|---|---|
| 0–11 | `MOT1_KP` … `MOT4_KD` | `REAL32` |
| 12–14 | `YAW_KP` / `YAW_KI` / `YAW_KD` | `REAL32` |
| 15 | `WHEEL_CPR` | `REAL32` (tics/tour) |
| 16 | `WHEEL_CIRC` | `REAL32` — **millimètres bruts** |
| 17 | `WHEEL_APB` | `REAL32` — **millimètres bruts** (demi-voie) |
| 18 | `CAR_TYPE` | `REAL32` (4 = FOURWHEEL) |

> ⚠️ Les 12 entrées `MOTn_*` **ne donnent pas 4 jeux de gains** : l'ESP32 replie l'index par
> `idx % 3` et n'a qu'**un seul triplet PID partagé par les 4 moteurs**. Écrire `MOT3_KP` revient
> à écrire `MOT1_KP`. Les unités **millimètres bruts** des index 16/17 ne doivent pas être
> confondues avec le ×10 fixe du protocole Yahboom.

### 5.5 Valeurs d'amorçage de la géométrie

Au démarrage, l'état MAVLink est amorcé depuis les macros de compilation
(`config/custom/bamboov310_config.h`) :

| Grandeur | Macro | Valeur de repli |
|---|---|---|
| Tics/tour | `COUNTS_PER_REV1` | `2114` |
| Ø roue | `WHEEL_DIAMETER` | `0.8` (m) |
| Voie | `LR_WHEELS_DISTANCE` | `1.3` (m) |
| Type de châssis | *littéral* | `4.0` (FOURWHEEL) |

> ⚠️ Ce sont des **valeurs de repli non mesurées** (un Ø de roue de 0,8 m est absurde) : la
> géométrie réelle est poussée depuis ROS2. Voir § 7.

---

## 6. Compiler et flasher

L'environnement PlatformIO doit être **nommé explicitement** : `default_envs` pointe ailleurs
parmi les 22 environnements du fichier.

```bash
cd firmware/esp32_bamboo
pio run -e bamboov3-wirshare_bamboo_mavlink             # compiler
pio run -e bamboov3-wirshare_bamboo_mavlink -t upload   # flasher (esptool)
```

- **Flash par le Type-C n° 9** (« USB »). Le circuit **auto-download** (n° 27) gère EN/BOOT :
  **aucun bouton à manipuler**.
- **Conserver le binaire précédent** : un mauvais flash immobilise le robot.
- `monitor_speed = 921600`.

---

## 7. Ce que le firmware n'exploite pas sur cette carte

Utile pour ne pas chercher des fonctions absentes :

| Fonction | État | Raison |
|---|---|---|
| Port **LIDAR** (n° 3) | non lu | `USE_LIDAR_UDP` non défini → `initLidar()` vide ; passthrough matériel (§ 4.3) |
| **Servos** (n° 13) | non pilotés | `DO_SET_SERVO` = `UNSUPPORTED`, `PCA_BASE` non défini (§ 4.5) |
| **Ultrason** HC-SR04 | inerte | `TRIG_PIN` / `ECHO_PIN` non définis → `Range::getRange()` ne mesure rien |
| **Carte TF/SD** (n° 22) | non utilisée | aucun accès dans ce firmware |
| **Persistance** des paramètres | absente | `PREFLIGHT_STORAGE` = `UNSUPPORTED` ; la configuration est repoussée par l'hôte à chaque connexion |
| Gains PID **par moteur** | non | un seul triplet partagé (§ 5.4) |
| Voies encodeur **3 et 4** | duplicatas | recopies de 1 et 2 (§ 4.2) |
| `stop()` (`BAMBOO_MOTOR_PWM`) | **sans effet** | le `case` est vide → le **seul frein réel** est `BAMBOO_CMD_VEL(0, 0)` |

---

## 8. Voir aussi

- [`../../docs/hardware_waveshare_ups_module_3s.md`](../../docs/hardware_waveshare_ups_module_3s.md)
  — carte d'alimentation 5 V utilisable avec cette carte (UPS Module 3S).
- [`../../../linorobot2/docs/bamboo4WD_V4_WSEsp32.md`](../../../linorobot2/docs/bamboo4WD_V4_WSEsp32.md)
  — exemple d'intégration ROS2 d'un robot bâti sur cette carte (dépôt frère). Le côté robot y
    référence la version de microcode ci-dessus.
