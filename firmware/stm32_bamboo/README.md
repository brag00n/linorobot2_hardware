# Bambou4WD_V3.02 — Firmware STM32 pour carte de contrôle robot ROS

Firmware embarqué (bare-metal + FreeRTOS) pour la carte **Yahboom YB-ERF01-V3.0**
(« STM32 ROS Robot Control Panel »), portée pour être compilée avec la
**chaîne GCC / PlatformIO** (à l'origine projet Keil MDK / ARMCC).

Version firmware : **V3.5.1** — MCU **STM32F103RCT6**.

---

## 1. Présentation

### 1.1 Rôle du firmware

La carte est à la fois un **contrôleur bas niveau de robot mobile** et une carte
de développement STM32. Ce firmware :

- pilote **4 moteurs à encodeur 12 V** en boucle fermée **PID** (vitesse) ;
- gère plusieurs **cinématiques de châssis** (Mecanum, 4 roues, Ackermann…) ;
- lit une **centrale inertielle 9 axes** (ICM20948, ou MPU9250) et calcule
  l'**attitude** (roll / pitch / yaw), avec correction de cap (**yaw PID**) ;
- expose tous les périphériques (servos, RGB, buzzer, OLED, SBUS, CAN) ;
- dialogue avec un calculateur hôte (Raspberry Pi / Jetson…) via un
  **protocole série binaire** à 115200 bauds, et **auto-publie** sa télémétrie.

L'ordonnancement est assuré par **FreeRTOS** (kernel V10.4.3). Tâches principales :

| Tâche | Priorité | Rôle |
|---|---|---|
| `vTask_Speed`       | 10 | Boucle de vitesse / PID moteurs |
| `vTask_Control`     | 9  | Application des consignes de mouvement |
| `vTask_Key`         | 8  | Bouton KEY1 |
| `vTask_Auto_Report` | 7  | Publication télémétrie série (~24 Hz/type) |
| `vTask_App_Handle`  | 4  | Traitement applicatif divers |
| `vTask_IMU`         | 3  | Acquisition IMU + fusion attitude |
| `vTask_OLED`        | 1  | Affichage OLED |

### 1.2 Châssis supportés (`src/app_motion.h`)

| Code | Type | Modèle Yahboom |
|---|---|---|
| `0x01` | Mecanum (petit châssis) | X3 |
| `0x02` | Mecanum (grand châssis)  | X3 PLUS |
| `0x03` | Mecanum mini | — |
| `0x04` | 4 roues classiques | X1 |
| `0x05` | Ackermann (direction) | R2 |
| `0x06` | Sunrise | — |

### 1.3 Place dans une pile ROS / ROS2

Ce dépôt contient **uniquement le firmware MCU**. Dans un robot complet :

```
   +------------------------+        USB (CH340, 115200, binaire)        +------------------------+
   |  Hôte SBC (RPi5/Jetson)| <----------------------------------------> |  Carte STM32 (ce dépôt)|
   |  ROS 2 (nœud driver)   |   consignes (vitesse, servo, RGB…)         |  FreeRTOS + PID + IMU  |
   |  carto, nav, capteurs  |   télémétrie (odométrie, IMU, batterie)    |  4 moteurs + encodeurs |
   +------------------------+                                            +------------------------+
```

L'hôte exécute les nœuds **ROS 2** (le pilote série Yahboom / `Rosmaster_Lib`,
puis navigation, SLAM, etc.). Le STM32 ne « fait pas de ROS » lui-même : il
publie une télémétrie binaire que le nœud driver convertit en topics ROS
(`/odom`, `/imu`, `/cmd_vel`…). La carte alimente aussi le SBC (protocole
d'alimentation Raspberry Pi 5 pris en charge).

---

## 2. Description matérielle de la carte (YB-ERF01-V3.0)

### 2.1 Caractéristiques principales

| Élément | Valeur |
|---|---|
| MCU | **STM32F103RCT6** — ARM Cortex-M3, 72 MHz |
| Mémoire | **48 KB RAM**, **256 KB Flash** |
| IMU | **ICM20948** 9 axes (gyro + accéléro + magnéto) — MPU9250 aussi géré |
| Pont USB→série | **CH340** (sur le connecteur **micro-USB**), 115200 bps |
| Moteurs | **4×** moteurs **12 V** avec **encodeurs** (boucle PID) |
| Servos | **4× servos PWM** + **servos série bus** (bus half-duplex) |
| Éclairage | Bandeau **RGB** adressable |
| Affichage | **OLED** (I²C) |
| Son | **Buzzer** |
| Bus | **CAN** (H/L), **SBUS** (récepteur radio-commande) |
| Boutons | **RESET**, **KEY1**, **BOOT0** |
| Debug | Connecteur **SWD** (3V3 / SWCLK / GND / SWDIO) |
| Alim | Entrée **12 V** sur connecteur en **T** (type Deans/XT), protégée (anti-inversion, court-circuit, surintensité servo) |
| Sortie alim | **USB-C « 5VOUT1 »** = *sortie* 5 V pour SBC (⚠️ pas une entrée de flash) |
| Conso veille | ~50 mA |
| Température | -40 °C à +85 °C |

> ⚠️ **Deux ports USB distincts** :
> - **micro-USB « USB Connect »** = données / **flash** (via CH340) → à utiliser pour programmer et communiquer ;
> - **USB-C « 5VOUT1 »** = **sortie** d'alimentation 5 V vers Raspberry Pi/Jetson. **Ne sert pas à flasher.**

### 2.2 Cartographie logicielle des périphériques

- **USART1** (PA9 TX / PA10 RX) → CH340 → USB : protocole hôte + `printf` de debug.
- Buzzer, LED, encodeurs (TIM), moteurs (PWM + sens), servos PWM, OLED (I²C
  logiciel), RGB, CAN, SBUS, ADC batterie : voir `lib/BSP/`.
- IMU : `lib/MEMS/` (driver InvenSense ICM20948 + magnétomètre AK09916).

### 2.3 Alarme batterie (à connaître)

Le firmware surveille la tension via l'ADC (diviseur ×4.03). Seuils (type Rosmaster/12,6 V) :
- **bas** = 9,6 V, **haut** = 13,0 V.
Sous le seuil pendant ~2 s → **bip régulier + LED + arrêt moteurs** et
`g_system_enable = 0` (moteurs bloqués jusqu'au reset avec batterie correcte).

👉 Si la carte est alimentée **uniquement par l'USB** (sans 12 V sur le
connecteur en T), l'ADC lit ~0 V et la carte **bipe** en continu. C'est normal :
brancher une **alimentation 12 V** sur l'entrée en T fait cesser l'alarme.
On peut désactiver l'alarme pour du test à l'établi via `ENABLE_LOW_BATTERY_ALARM 0`
dans `src/config.h` (deux occurrences, lignes 14 et 52).

---

## 3. Contenu du dépôt

```
Bambou4WD_V3.02/
├── platformio.ini          # Config PlatformIO (env genericSTM32F103RC, build série)
├── STM32F103RC_FLASH.ld    # Link app relogée à 0x08004000 (240K Flash / 48K-32 RAM)
├── bootloader/             # Bootloader IAP résident (16K @0x08000000, projet PIO à part)
│   ├── platformio.ini      #   env:bootloader (registres nus, ni FWlib ni FreeRTOS)
│   ├── STM32F103RC_BL.ld   #   link bootloader (16K Flash)
│   └── src/main.c          #   décision boot + protocole IAP + écriture Flash
├── src/                    # Code applicatif
│   ├── main.c              # Point d'entrée
│   ├── app*.c/.h           # Logique robot : motion, PID, IMU/attitude, batterie,
│   │                       #   OLED, RGB, SBUS, servos, flash, cinématiques châssis
│   ├── protocol.c/.h       # Protocole série binaire (parsing + émission)
│   ├── config.h            # Interrupteurs de fonctionnalités (build-time)
│   └── syscalls.c          # Stubs newlib (printf → USART1) pour le build GCC
├── lib/
│   ├── CMSIS/              # CMSIS-3 + startup GCC + system_stm32f10x
│   ├── FWlib/              # StdPeriph ST (drivers registre F1)
│   ├── FreeRTOS/           # Kernel V10.4.3 + port GCC ARM_CM3
│   ├── BSP/                # Board Support : usart, moteurs, encodeurs, servo,
│   │                       #   oled, rgb, beep, can, adc, sbus, i2c…
│   ├── MEMS/               # Driver IMU ICM20948 (InvenSense) + MPU9250
│   ├── IAP/                # iap_protocol.h : contrat Flash + protocole partagé app/BL
│   └── TOOL/               # Outils (assistant PID…)
├── tools/
│   ├── ros_monitor.py      # Décodeur série lisible du protocole (voir §6)
│   ├── ros_mcp_server.py   # Serveur MCP (accès live + flash_firmware, voir §6.3)
│   └── iap_flash.py        # Client IAP hôte (flash appli par UART, voir §5.A)
└── .vscode/                # Réglages VSCode / PlatformIO
```

> Note portage GCC : ce projet ciblait Keil MDK. Ont été adaptés pour GCC : le
> startup (`lib/CMSIS/startup_stm32f103xe.s`), le port FreeRTOS
> (`lib/FreeRTOS/src/port.c`, `inc/portmacro.h`), les intrinsèques `strex` de
> `core_cm3.c`, le script de link et les syscalls newlib (`src/syscalls.c`).

---

## 4. Compiler le code

Environnement : **PlatformIO Core** + toolchain **arm-none-eabi-gcc** (installée
automatiquement par PlatformIO). Cible : `env:genericSTM32F103RC` (bare-metal,
sans framework HAL — le projet embarque sa propre CMSIS/StdPeriph).

Résultat typique : **RAM ~54 %** (26,7 KB/48 KB), **Flash ~64 %** (167 KB/256 KB).
Artefacts : `.pio/build/genericSTM32F103RC/firmware.elf` et `firmware.bin`.

### 4.1 Sous VSCode (extension PlatformIO IDE)

1. Installer l'extension **PlatformIO IDE**.
2. Ouvrir le dossier du projet (ou `Bambou4WD_V3.02.code-workspace`).
3. Barre d'état PlatformIO (en bas) :
   - **✓ (Build)** pour compiler ;
   - **→ (Upload)** pour flasher (voir §5) ;
   - **🔌 (Serial Monitor)** pour la console série.
4. Ou via la palette : `Ctrl+Shift+P` → *PlatformIO: Build*.

### 4.2 Hors VSCode (ligne de commande — dont Raspberry Pi)

PlatformIO Core (CLI `pio`) fonctionne sous Linux/macOS/Windows, **y compris sur
Raspberry Pi (ARM)**.

Installation de PlatformIO Core (exemple RPi / Debian) :

```bash
sudo apt update && sudo apt install -y python3 python3-venv python3-pip git
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py)"
# ajouter ~/.platformio/penv/bin au PATH, ou utiliser le chemin complet
export PATH="$HOME/.platformio/penv/bin:$PATH"
```

Compilation :

```bash
git clone <URL_DU_DEPOT> && cd Bambou4WD_V3.02
pio run                       # compile l'environnement genericSTM32F103RC
pio run -t clean              # nettoyage
```

> Sur Windows, le Python embarqué de PlatformIO est en
> `~/.platformio/penv/Scripts/` (`pio.exe`, `python.exe`).

---

## 5. Déployer le binaire sur la carte

Deux voies, toutes deux par le **CH340 / port micro-USB** (aucune sonde JTAG requise) :

| Voie | Quand | Boutons |
|------|-------|---------|
| **IAP (nominale)** — §5.A | mise à jour courante de l'application | **aucun** |
| **ROM série (installation / secours)** — §5.B | 1ʳᵉ fois (installer le bootloader) ou récupération | **BOOT0 + RESET** |

### 5.0 Architecture de la Flash (bootloader résident + application relogée)

La Flash 256 KB est découpée en deux images indépendantes :

```
0x08000000  ┌─────────────────────────────┐
            │  Bootloader IAP  (16 KB)     │  installé UNE fois via ROM/BOOT0
0x08004000  ├─────────────────────────────┤
            │  Application     (240 KB)    │  mise à jour par IAP (UART)
0x08040000  └─────────────────────────────┘
```

À chaque reset, le **bootloader résident** (`bootloader/`, registres nus, ni
FWlib ni FreeRTOS) s'exécute d'abord et décide :
- **flag partagé armé** (l'application a reçu `0xA3`) → mode IAP (réception UART) ;
- **application invalide** (Flash vierge/ratée) → mode IAP (sécurité anti-brique) ;
- **sinon** → saut immédiat vers l'application (démarrage nominal).

L'application est compilée **relogée à `0x08004000`** (`-DVECT_TAB_OFFSET=0x4000`
+ `STM32F103RC_FLASH.ld`), sa table des vecteurs étant repositionnée par
`SystemInit`. Contrat partagé : [`lib/IAP/iap_protocol.h`](lib/IAP/iap_protocol.h).

### 5.A Mise à jour par IAP (nominale, sans boutons)

Pré-requis : le bootloader résident est installé (voir §5.B, à faire **une** fois).

```bash
pio run                       # compile l'application (firmware.bin à 0x08004000)
python tools/iap_flash.py     # envoie 0xA3, ERASE/WRITE/VERIFY/GO sur COM4
```

`iap_flash.py` : bascule l'application dans le bootloader (`0xA3`), transfère
`firmware.bin`, vérifie par **CRC32** puis saute à l'application. Options :
`--port COM4`, `--bin <chemin>`, `--no-enter` (déjà dans le bootloader).

Depuis un agent, l'outil MCP **`flash_firmware`** fait tout (libère COM4, lance
`iap_flash.py`, rouvre le port) — voir §6.3.

> Le port doit être **libre** : fermez le moniteur série (et le serveur MCP, ou
> utilisez son outil `flash_firmware` qui libère lui-même le port).

### 5.B Installation / secours par le bootloader ROM (BOOT0 + RESET)

À réserver à la **première** installation du bootloader résident, ou à la
récupération si l'application ne démarre plus. Utilise le **bootloader série ROM**
du STM32 (`stm32flash`, protocole `serial`) — déjà configuré :

```ini
upload_protocol = serial
upload_port     = COM4        ; ← adapter au port CH340 de VOTRE machine
```

### 5.1 Préparer la carte (câblage + alimentation)

1. **Alimentation 12 V** sur l'entrée en **T** (connecteur rouge Deans/XT), en
   **respectant la polarité** (+ / –). Une alim 12 V / ≥2 A convient pour les
   tests ; prévoir plus de courant pour faire tourner les 4 moteurs en charge.
   → évite l'alarme batterie et réactive les moteurs.
2. **Câble micro-USB** entre le port **« USB Connect »** de la carte et le PC/SBC.
   (Le micro-USB peut aussi alimenter la logique, mais **pas** le rail moteur ni
   fournir une tension batterie valide — voir §2.3.)
3. Identifier le port série :
   - **Windows** : Gestionnaire de périphériques → *Ports (COM & LPT)* →
     « USB-SERIAL CH340 (COMx) ». Installer le pilote **CH340** (`CH341SER`) si absent.
   - **Linux/RPi** : `ls /dev/ttyUSB*` (souvent `/dev/ttyUSB0`). Ajouter
     l'utilisateur au groupe `dialout` (`sudo usermod -aG dialout $USER`, puis relogin).
   - Lister aussi via : `pio device list`.

### 5.2 Entrer en mode bootloader (manœuvre BOOT0)

1. **Maintenir** le bouton **BOOT0** ;
2. sans le relâcher, appuyer puis relâcher **RESET** ;
3. relâcher **BOOT0**.

→ Le STM32 attend le flash sur la liaison série.

### 5.3 Flasher (BOOT0 maintenu)

**Installation du bootloader résident** (à `0x08000000`, à faire une fois) :

```bash
pio run -d bootloader -t upload          # projet bootloader/ (16 KB)
```

**Flash direct de l'application** (à `0x08004000`, secours si l'IAP est inutilisable) :

```bash
# adapter éventuellement le port :
pio run -t upload
pio run -t upload --upload-port /dev/ttyUSB0     # Linux/RPi
pio run -t upload --upload-port COM4             # Windows
```

Attendre `Starting execution … done.` / `[SUCCESS]`.

> Ordre recommandé pour une carte neuve : flasher **le bootloader** puis
> **l'application** (deux manœuvres BOOT0+RESET), ou flasher le bootloader puis
> passer à l'IAP (§5.A) pour l'application.

### 5.4 Démarrer le firmware

Appuyer une fois sur **RESET** (BOOT0 relâché) → le bootloader s'exécute puis
saute à l'application.

> Sonde SWD (optionnel) : un ST-Link/J-Link sur le connecteur SWD permet aussi
> le flash/debug. Décommenter `debug_tool = stlink` (ou `jlink`) et utiliser
> `upload_protocol = stlink` dans `platformio.ini`.

> ⚠️ **Un seul programme par port** : fermer le moniteur série avant tout upload,
> sinon erreur « Access is denied » / port occupé.

---

## 6. Tester le fonctionnement

### 6.1 Décodeur série fourni — `tools/ros_monitor.py`

La carte émet en continu (auto-report) des **trames binaires** ; un moniteur
texte classique les affiche donc comme des caractères illisibles. Le script
`tools/ros_monitor.py` (pur Python + `pyserial`, déjà fourni avec PlatformIO)
les décode en clair, **vérifie le checksum**, et affiche un tableau de bord.

**Affichage agrégé par moyenne** : l'affichage n'est pas rafraîchi à chaque
trame. À intervalle fixe (`--interval`, **défaut 30 s**), toutes les trames
reçues pendant l'intervalle sont accumulées et c'est la **moyenne** de chaque
métrique qui est affichée (avec le nombre d'échantillons et la fréquence
mesurée). Baisser `--interval` rapproche du temps réel (ex. `--interval 0.5`).

```bash
# Windows :
~/.platformio/penv/Scripts/python.exe tools/ros_monitor.py            # dashboard, moyenne / 30 s
~/.platformio/penv/Scripts/python.exe tools/ros_monitor.py --interval 5   # moyenne toutes les 5 s
~/.platformio/penv/Scripts/python.exe tools/ros_monitor.py --interval 0.5 # quasi temps réel (2 Hz)
~/.platformio/penv/Scripts/python.exe tools/ros_monitor.py --mode log --hex
~/.platformio/penv/Scripts/python.exe tools/ros_monitor.py --list     # ports dispo
~/.platformio/penv/Scripts/python.exe tools/ros_monitor.py --port COM5

# Linux/RPi :
python3 tools/ros_monitor.py --port /dev/ttyUSB0
```

Exemple de sortie (dashboard, `--interval 30`) :

```
=== YB-ERF01 ROS monitor ===  moyenne / 30s  (fenetre 30.0s)
    trames OK:  2880  checksum KO:   0   | total OK: 2880  KO: 0

[0x0a] Vitesse & batterie  (moyenne de 720 trames, 24.0 Hz)
    Vx (mm/s)           :      0.000
    Vy (mm/s)           :      0.000
    Vz (rad/s)          :      0.000
    Batterie (V)        :     12.200
[0x0c] Attitude (RPY)  (moyenne de 720 trames, 24.0 Hz)
    Roll (deg)          :      0.290
    Pitch (deg)         :      0.440
    Yaw (deg)           :     11.530
[0x0e] IMU brut (ICM20948)  (moyenne de 720 trames, 24.0 Hz)
    Gyro  (rad/s)       :    +0.001    -0.002    +0.000
    Accel (m/s2)        :    +0.050    -0.030    -9.900
    Mag   (uT)          :   +12.300   -34.100   +48.700
[0x0d] Encodeurs  (moyenne de 720 trames, 24.0 Hz)
    Comptage (tics) : M1=    12034.0   M2=    12030.0   M3=    12028.0   M4=    12040.0
    Vitesse (tics/s): M1=    +330.5   M2=    +331.0   M3=    +329.8   M4=    +330.2
    Vitesse (tr/min): M1=   +15.0   M2=   +15.0   M3=   +15.0   M4=   +15.0
```

> **Vitesse de rotation des moteurs** : le comptage encodeur étant cumulatif, la
> vitesse de chaque moteur est dérivée = (comptage_fin − comptage_début) ÷ durée
> de la fenêtre, affichée en **tics/s** et en **tr/min**. Le signe donne le sens.
> La conversion en tr/min suppose le nombre de **tics par tour de roue** (option
> `--cpr`, défaut **1320** = moteur 330 RPM des châssis Mecanum X3 / 4-roues X1).
> Adapter selon le moteur : `--cpr 2464` (X3 PLUS 205 RPM), `1040` (Sunrise
> 450 RPM), `836` (Ackermann 550 RPM). Ces valeurs viennent de `src/app_motion.h`.
>
> Le « Comptage (tics) » reste la moyenne des positions sur la fenêtre (utile à
> l'arrêt) ; c'est sa variation qui donne la vitesse ci-dessus.

#### Déterminer les tics/tour de l'odomètre

Les encodeurs sont des **codeurs en quadrature 2 voies A/B** (2 capteurs Hall par
moteur), décodés ×4 en matériel par les timers du STM32. Les voies A/B et leur
sens ne sont **pas** reconfigurables depuis l'outil (câblage + firmware) : ce qui
compte côté hôte est le **nombre de tics par tour de roue** (`--cpr`). Deux moyens
pour le connaître sans démonter :

```bash
# 1. Interroger la carte sur son type de châssis -> tics/tour déduits (lecture seule)
python tools/ros_monitor.py --detect
#   Envoie FF FC 05 50 15 00 6A (FUNC_REQUEST_DATA=0x50, requête CAR_TYPE=0x15) ;
#   la carte répond son type, qui mappe la table 1320/2464/1040/836.

# 2. Mesure empirique : tourner une roue d'un nombre exact de tours à la main
python tools/ros_monitor.py --calibrate --turns 5
#   Affiche en direct la variation de comptage ; à l'arrêt (Ctrl+C) :
#   tics/tour = |delta| / nb_tours par moteur, et le signe = sens (voies A/B).
```

> La méthode 1 suppose que le firmware est bien configuré pour votre châssis ; la
> méthode 2 est une vérité terrain indépendante et révèle aussi le sens de comptage.

Interprétation rapide pour valider une carte : **Batterie ≈ tension réelle**,
**Accel Z ≈ -9,8 m/s²** carte posée à plat, **gyro ≈ 0** à l'arrêt,
**encodeurs** qui varient quand on tourne les roues à la main, **0 checksum KO**.

### 6.2 Protocole série (résumé)

Trame (little-endian) :

```
[0xFF][0xFB][LEN][FUNC][données…][CHECKSUM]
 tête   ID   |     |                  checksum = somme(octets[2..fin-1]) & 0xFF
             |     code fonction (src/protocol.h)
             LEN = taille_totale - 2
```

Trames auto-publiées :

| Func | Nom | Contenu |
|---|---|---|
| `0x0A` | REPORT_SPEED    | Vx,Vy,Vz (int16) + batterie (uint8, V×10) |
| `0x0E` | REPORT_ICM_RAW  | gyro[3], accel[3], mag[3] (int16, ×1000) |
| `0x0C` | REPORT_IMU_ATT  | roll,pitch,yaw (int16, rad×10000) |
| `0x0D` | REPORT_ENCODER  | M1..M4 (int32, comptage) |

Codes de commande (hôte → carte) : moteurs `0x10`, run `0x11`, mouvement `0x12`,
PID `0x13/0x14`, type châssis `0x15`, servos `0x03/0x04/0x20…`, RGB `0x05/0x06`,
buzzer `0x02`, version `0x51`… (liste complète : `src/protocol.h`).

### 6.3 Accès live via MCP (`tools/ros_mcp_server.py`)

Pour qu'un assistant compatible **MCP** (p. ex. Claude Code) lise les métriques en
direct, un serveur MCP tient le port série ouvert en permanence et expose des
outils. Zéro dépendance hors pyserial (protocole JSON-RPC/stdio implémenté à la
main). Il se reconnecte seul si le port est occupé au démarrage.

Enregistrement : le fichier `.mcp.json` à la racine du projet déclare le serveur
`bambou-board` (port/baud/cpr via variables d'environnement). Au prochain
démarrage de Claude Code, approuver le serveur (`/mcp` pour l'état).

Outils exposés :

| Outil | Rôle |
|---|---|
| `status` | connexion, Hz par type de trame, OK/KO, cpr, type châssis |
| `metrics` | vitesse & batterie, attitude RPY, gyro/accel, encodeurs, vitesse moteurs |
| `encoders` | comptage cumulatif + vitesse instantanée (tics/s, tr/min) |
| `car_type` | interroge la carte (0x50→0x15) → type + cpr, appliqué auto |
| `calibrate_baseline` / `calibrate_read` | mesure des tics/tour en tournant une roue à la main, **sans synchro temporelle** |
| `flash_firmware` | mise à jour de l'application par **IAP** (UART) : libère le port, lance `iap_flash.py`, rouvre (voir §5.A) |
| `enter_bootloader` | envoie `0xA3` et **libère** le port (pour un `pio run -t upload` manuel) |

Le port étant à propriétaire unique, fermer tout autre moniteur série. Test hors
client : `python tools/ros_mcp_server.py selftest` (lit 3 s, affiche l'état).

### 6.4 Via ROS 2 (sur l'hôte)

Sur le SBC (Raspberry Pi/Jetson) relié en USB à la carte :

1. Installer le **pilote série Yahboom** (`Rosmaster_Lib`) et le paquet ROS 2 du
   robot (fournis par Yahboom selon le modèle de châssis).
2. Le nœud driver ouvre `/dev/ttyUSB0` @115200, publie `/odom`, `/imu`,
   l'état batterie, et souscrit à `/cmd_vel` pour piloter les moteurs.
3. Test typique :
   ```bash
   ros2 topic echo /imu          # attitude/accéléro en direct
   ros2 topic echo /odom         # odométrie encodeurs
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"  # avance
   ```

> Le firmware étant agnostique de ROS, on peut aussi le piloter depuis n'importe
> quel script série envoyant les trames du §6.2 (Python `pyserial`, etc.).

---

## 7. Dépannage rapide

| Symptôme | Cause probable | Solution |
|---|---|---|
| Bip régulier après reset | Pas de 12 V (USB seul) → alarme batterie | Brancher 12 V sur l'entrée en T |
| Console série illisible | Protocole **binaire** (normal) | Utiliser `tools/ros_monitor.py` |
| Upload « Access is denied » | Port série déjà ouvert | Fermer le moniteur série |
| Upload « Failed to init device » | Carte pas en bootloader | Refaire BOOT0 + RESET, relancer |
| IAP « pas de réponse HELLO » | Bootloader résident absent, ou port occupé | Installer le bootloader (§5.B) ; libérer le port |
| Aucun COM / ttyUSB | Pilote CH340 absent / droits | Installer CH340 ; `dialout` sous Linux |
| Moteurs inertes | `g_system_enable=0` (alarme) | Batterie OK puis **RESET** |

---

## 8. Licence

Voir le fichier [`LICENSE`](LICENSE).
