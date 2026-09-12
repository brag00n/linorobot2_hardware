# GrovePi+ Bambou — carte capteurs ATmega328P (IMU + ultrasons + IR)

> **Statut : fonctionnel, protocole étendu + horodatage (v0.3).** La **GrovePi+ v3.0** n'est
> **pas** utilisée ici avec un Raspberry Pi : on **détourne son ATmega328P**
> (quartz 16 MHz = un Arduino Uno) en **microcontrôleur capteurs autonome**,
> flashé depuis le **PC** via un adaptateur USB↔série, qui parle le **même
> protocole de trames que la STM32 Bamboo**
> (cf. [`../stm32_bamboo/tools/ros_monitor.py`](../stm32_bamboo/tools/ros_monitor.py)).
> La carte est une sœur de `stm32_bamboo` : une **carte à flasher**, pas un
> appareil Linux comme `unitv2_bamboo`.

---

## 1. Rôle dans l'architecture Bambou

L'ATmega328P agrège trois familles de capteurs et les diffuse sur son UART :

- **IMU MPU6050** (I²C) → angle **roll/pitch fusionné à bord** (filtre complémentaire)
  + accéléro/gyro bruts. Le yaw reste fourni par l'IMU de la STM32 (pas de doublon).
- **4× HC-SR04** (ultrason) → distances, pour l'évitement d'obstacle rapproché.
- **1× Sharp GP2Y0A710K0F** (IR analogique) → distance longue portée (~0,6–5 m).

```
  [ MPU6050 ]───I2C(A4/A5)──┐
  [ 4x HC-SR04 ]─D2..D9──────┤
  [ Sharp IR ]───A0──────────▶[ ATmega328P @16MHz ]──UART──▶ USB<->série ──▶ PC
                                  trames [0xFF][ID][LEN][FUNC][..][CHK]
```

Intégration ciblée : un nœud-pont ROS 2 (côté SBC/PC) lit ces trames et publie
`sensor_msgs/Imu` (partiel : roll/pitch) et des `sensor_msgs/Range` — symétrique du
pont série STM32 existant. *(Le nœud-pont n'est pas fourni ici.)*

> **Pas de LED pilotable.** La GrovePi+ n'expose aucune LED utilisateur commandable
> par l'ATmega (seule une LED d'alimentation est câblée sur le rail). Il n'y a donc
> **pas de trame LED** dans ce protocole.

## 2. Matériel

| Élément | Référence | Valeur / interface |
|---|---|---|
| MCU | **ATmega328P**, quartz **16 MHz** | cible PlatformIO `uno`, 5 V |
| IMU | **InvenSense MPU-6050** | I²C @ **0x68** (A4=SDA, A5=SCL) |
| Ultrason | **4× HC-SR04** | Trig/Echo TTL 5 V (direct, pas de level-shift) |
| IR longue portée | **Sharp GP2Y0A710K0F** | sortie analogique **Vo → A0** |
| Adaptateur flash | **PL2303TA** (ou FTDI) | GND/CTS/VCC/TXD/RXD/DTR, sélecteur **5 V** |

## 3. Câblage (défauts firmware, reconfigurables à chaud)

- **MPU6050** sur un **port I²C Grove** (= bus A4/A5 de l'ATmega). VCC 5 V, GND, SDA, SCL.
- **HC-SR04** : **1 capteur par port digital Grove**. ⚠️ **Piège** : le connecteur Grove
  est câblé `[SIG1, SIG2, VCC, GND]`, alors que le HC-SR04 attend `[VCC, Trig, Echo, GND]`.
  **Ne pas** brancher un câble Grove droit — câbler **fil à fil** : `VCC→5V`, `Trig→SIGx`,
  `Echo→SIGy`, `GND→GND`. Défaut : `Trig = D2/D4/D6/D8`, `Echo = D3/D5/D7/D9`.
- **Sharp GP2Y0A710K0F** : connecteur 5 fils. **Vo (rouge) → A0** seul ; les **deux fils
  VCC (bleu+jaune) ensemble → 5 V** ; les **deux fils GND (blanc+noir) ensemble → GND**.
  Ajouter un **condensateur ~10 µF** entre VCC et GND au plus près du capteur (pointes de
  courant de la LED IR). Sortie **non linéaire** et ambiguë < ~0,6 m.
- **Adaptateur USB↔série** sur le **header 6 broches** (GND/CTS/VCC/TXD/RXD/DTR),
  **sélecteur sur 5 V**, croiser TXD↔RXD. DTR requis pour l'auto-reset (flash bootloader).
- ⚠️ Ne pas utiliser **D0/D1** (UART) ni **A4/A5** (I²C) pour les capteurs.

## 4. Mécanisme de communication (protocole série)

Trame **little-endian**, miroir de la STM32 :

```
[0xFF][ID][LEN][FUNC][donnees...][CHK]
  0xFF : entête
  ID   : 0xFC hôte→carte, 0xFB carte→hôte
  LEN  : taille_totale - 2   (= nb d'octets à partir de LEN inclus)
  FUNC : code fonction (table ci-dessous)
  CHK  : somme(octets[2..fin-1]) & 0xFF   (de LEN à l'avant-dernier)
```

### 4.1 Table des codes fonction (FUNC)

| FUNC | Sens | Rôle | Payload |
|---|---|---|---|
| `0x50` REQUEST_DATA | h→c | lecture à la demande | `[subfunc][param]` (param = dev_id, `0xFF`=tous) |
| `0x51` VERSION | c→h | version µcode (**test de contact**) | `[major][minor][patch]` |
| `0x52` TIME_SYNC | h↔c | echo d'horloge (synchro) | req `[seq]` ; ack `[seq][t_board u32 ms]` |
| `0x53` STATUS | c→h | état carte + santé devices | `[board_state][n]` + n×`[dev_id][health]` |
| `0x54` CONFIG | c→h | dump config d'un device | `[dev_id][enabled][pinA][pinB][period u16][health]` |
| `0x55` SET_CONFIG | h→c | régler un device (**RAM seule**) | `[dev_id][enabled][pinA][pinB][period u16]` |
| `0x56` CONFIG_ACTION | h↔c | save/defaults/reload EEPROM + ack | req `[action][guard]` ; ack `[action][result]` |
| `0x60` REPORT_IMU | c→h | angle + bruts | `[ts u32 ms]` + `roll,pitch` (i16, deg×100) + `ax,ay,az,gx,gy,gz` (i16) |
| `0x61` REPORT_ULTRA | c→h | 4 distances | `[ts u32 ms]` + `dist[4]` (u16 mm ; `0xFFFF`=pas d'écho) |
| `0x62` REPORT_IR | c→h | distance IR | `[ts u32 ms]` + `dist` (u16 mm ; `0xFFFF`=hors plage) + `adc_brut` (u16) |

- `0x5F` = **SAVE_VERIFY** : octet de garde exigé par `CONFIG_ACTION` (convention STM32,
  **ce n'est pas un FUNC**). Une action EEPROM sans cette garde est refusée.
- **Auto-report** : IMU, ultrasons et IR sont diffusés spontanément à la cadence propre de
  chaque device (défaut 50 ms). Chaque trame report porte un **timestamp `ts` (u32 ms, horloge
  monotone interne)** en **tête de payload** (cf. §12). `REQUEST_DATA` permet en plus une lecture ponctuelle.

### 4.2 Exemples de trames (hex)

```
Demander la version :   FF FC 05 50 51 FF A5        (REQUEST_DATA, subfunc=0x51, param=0xFF)
Réponse version 0.3.0 : FF FB 06 51 00 03 00 5A
Demander le statut :    FF FC 05 50 53 FF A7
Synchro horloge seq=1 : FF FC 04 52 01 57            (TIME_SYNC ; ack = FF FB 07 52 01 <t_board u32> <chk>)
Régler period IR=100ms: FF FC 09 55 20 FF FF FF 64 00 DF   (dev=0x20, enabled/pins=garder, period=100)
Sauver en EEPROM :      FF FC 05 56 01 5F BB        (action=save, guard=0x5F)
```

(`LEN = taille_totale − 2` ; `CHK = Σ(octets de LEN à l'avant-dernier) & 0xFF`.)

## 5. Devices

### 5.1 MPU6050 (IMU 6 axes, I²C)

- **Référence** : InvenSense MPU-6050, accéléromètre 3 axes + gyroscope 3 axes, I²C @ `0x68`.
- **Fonctionnement** : lecture des 14 octets accel/temp/gyro (registre `0x3B`), puis **filtre
  complémentaire** à bord (98 % intégration gyro + 2 % angle accéléro) → `roll`/`pitch`
  absolus, peu bruités, sans dérive lente. Le module met à jour l'angle à **chaque tour de
  boucle** pour rester réactif. Échelles : accel ±2 g (16384 LSB/g), gyro ±250 °/s (131 LSB/°/s).
- **Câblage** : port I²C Grove → A4 (SDA), A5 (SCL), VCC 5 V, GND.
- **Trames** : sortie `0x60 REPORT_IMU` (roll, pitch en deg×100 + 6 valeurs brutes) ;
  configurable via `dev_id 0x00` (`pinA` = adresse I²C).
- **Santé** : `i2c-nack` (0x11) si pas d'ACK à l'adressage, `i2c-read` (0x12) si lecture < 14 o.
- **Réf.** : [datasheet MPU-6050 (InvenSense)](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf),
  [registres](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).
- **Code** : [`lib/Mpu6050/`](lib/Mpu6050/).

### 5.2 HC-SR04 (télémètre ultrason)

- **Référence** : HC-SR04, portée ~2 cm–4 m, écho TTL 5 V. Ici **plafonné à 250 cm**.
- **Fonctionnement** : impulsion Trig 10 µs → le capteur émet un train ultrason et lève Echo
  pendant la durée aller-retour. `pulseIn()` (bloquant) mesure cette durée ; distance = durée
  × 10 / 58 (mm). `pulseIn` étant bloquant, on lit **un seul capteur par tour de boucle**
  (round-robin, canaux désactivés sautés) → l'IMU reste réactif, chaque ultrason ~loop/N.
- **Câblage** : voir §3 (câblage **fil à fil**, pas de câble Grove droit).
- **Trames** : sortie `0x61 REPORT_ULTRA` (4× u16 mm, `0xFFFF` = pas d'écho) ;
  groupe configurable via `dev_id 0x01` (enable + cadence du batch), canaux individuels via
  `dev_id 0x10..0x13` (`pinA` = Trig, `pinB` = Echo, enable).
- **Santé** : `no-echo` (0x21) après plusieurs timeouts consécutifs (capteur absent / hors portée).
- **Réf.** : [datasheet HC-SR04](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf),
  [GrovePi+ graphical datasheet (Dexter Industries)](https://github.com/DexterInd/GrovePi).
- **Code** : [`lib/Ultrasonic/`](lib/Ultrasonic/).

### 5.3 Sharp GP2Y0A710K0F (IR analogique longue portée)

- **Référence** : Sharp GP2Y0A710K0F, télémètre IR **100–550 cm**, sortie tension analogique.
- **Fonctionnement** : sortie Vo **non linéaire**. Conversion issue de la lib
  [guillaume-rico/SharpIR](https://github.com/guillaume-rico/SharpIR) (modèle 100550) :
  `current_mV = map(adc, 0..1023, 0..5000)` ; valide seulement si `current_mV ∈ [1400, 3300]`
  (~63–500 cm) ; `distance_cm = 137500 / (current_mV − 1125)`. Le module lit une **médiane de
  9 échantillons** (robuste aux pics) et conserve l'**ADC brut** dans la trame pour la
  calibration par exemplaire.
- **Câblage** : Vo → A0 ; VCC/GND doublés ; condensateur ~10 µF (voir §3).
- **Trames** : sortie `0x62 REPORT_IR` (`dist` u16 mm + `adc_brut` u16) ;
  configurable via `dev_id 0x20` (`pinA` = broche analogique).
- **Santé** : `adc-low` (0x31) si ADC bloqué à 0 (débranché ?), `adc-high` (0x32) si saturé à 1023.
- **Réf.** : [tutoriel makerguides](https://www.makerguides.com/sharp-gp2y0a710k0f-ir-distance-sensor-arduino-tutorial/),
  [datasheet Sharp](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a710k_e.pdf).
- **Code** : [`lib/SharpIR/`](lib/SharpIR/). *(Validation matérielle IR reportée : device secondaire.)*

## 6. Configuration des devices

Chaque device a un **`dev_id`** ; sa config tient dans un `DevCfg`
`{ enabled, pinA, pinB, periodMs }` :

| dev_id | Device | `pinA` | `pinB` | `periodMs` |
|---|---|---|---|---|
| `0x00` | IMU | adresse I²C (0x68) | — | cadence report `0x60` |
| `0x01` | ULTRA (groupe) | — | — | cadence report `0x61` |
| `0x10`–`0x13` | Ultra canal 0–3 | Trig | Echo | — |
| `0x20` | Sharp IR | broche analog (A0) | — | cadence report `0x62` |

- **Lire** : `REQUEST_DATA 0x50` avec `subfunc = 0x54` et `param = dev_id` → réponse `0x54 CONFIG`.
- **Régler (RAM)** : `SET_CONFIG 0x55` `[dev_id][enabled][pinA][pinB][period u16]`.
  Un octet à **`0xFF` = « garder »** (enabled/pinA/pinB inchangés) ; `period = 0` = garder.
  L'écriture **ne touche jamais l'EEPROM** ; la carte accuse en renvoyant le `0x54 CONFIG` du device.
- **Persister** : `CONFIG_ACTION 0x56` `[action][guard=0x5F]` :
  - `0x01` **save** RAM→EEPROM · `0x02` **defaults** (reset usine) · `0x03` **reload** depuis EEPROM.
  - Ack `0x56` `[action][result]` avec `result` : `0`=fait, `1`=inchangé, `2`=**throttlé**
    (trop rapproché), `3`=garde invalide.
- **Protection anti-usure EEPROM (rafale de save)** — dans `lib/DeviceConfig/` :
  (a) `EEPROM.update()` octet par octet (n'écrit que ce qui change) ;
  (b) skip total si le CRC de la config RAM == CRC stocké (`result=1`) ;
  (c) **intervalle minimal ≥ 2 s** entre deux écritures physiques ; une save trop rapprochée
  est refusée (`result=2`) **sans toucher l'EEPROM**. Au boot, la config est chargée depuis
  l'EEPROM si `magic`+`crc` sont valides, sinon les défauts compilés s'appliquent.

## 7. Statut carte

`REQUEST_DATA 0x50` / `subfunc 0x53` → `0x53 STATUS` `[board_state][n]` + n×`[dev_id][health]`.

- **board_state** : `0`=OK · `1`=DEGRADED (≥1 device activé en erreur) · `2`=FAULT (IMU en erreur).
- **health / code erreur** (octet par device, aussi présent dans `0x54 CONFIG`) :
  `0x00`=OK · `0x01`=désactivé · `0x10`=erreur générique · `0x11`=I²C no-ACK ·
  `0x12`=I²C lecture courte · `0x21`=ultra sans écho · `0x31`=ADC IR à 0 · `0x32`=ADC IR saturé.

## 8. Structure du firmware (`lib/`)

Modularisé par device, miroir de `../stm32_bamboo/lib/BSP/` (un module = un capteur) :

| Module | Rôle |
|---|---|
| [`lib/Protocol/`](lib/Protocol/) | trame (HEAD/ID/FUNC), `sendFrame`, `put16/32`/`get16/32`, `FrameParser` (RX), codes dev_id/erreur |
| [`lib/Clock/`](lib/Clock/) | horloge monotone `nowMs()` (Timer2 ISR @ 1 kHz, lecture atomique) — référentiel de temps (cf. §12) |
| [`lib/Mpu6050/`](lib/Mpu6050/) | IMU MPU6050 + filtre complémentaire, santé I²C |
| [`lib/Ultrasonic/`](lib/Ultrasonic/) | HC-SR04 (un canal), `pingMm`, santé no-echo |
| [`lib/SharpIR/`](lib/SharpIR/) | Sharp GP2Y0A710K0F, médiane + conversion, santé ADC |
| [`lib/DeviceConfig/`](lib/DeviceConfig/) | table `DevCfg`, défauts, EEPROM load/save (CRC, magic, throttle) |
| [`src/main.cpp`](src/main.cpp) | orchestration : ordonnanceur par-device, dispatch, STATUS/CONFIG |

## 9. Flash — procédure

### Étape 0 — valider le PL2303TA (Windows 11)
Beaucoup de PL2303(TA) sont bloqués par le driver Prolific récent (« code 10, contrefait »).
Brancher l'adaptateur, vérifier dans le **Gestionnaire de périphériques** qu'un **COMx monte
sans erreur**. Repli : driver Prolific **legacy 3.3.2.102**. Sans port stable, rien ne marche.

### Étape 1 — détecter le bootloader (lecture seule, aucune écriture)
```
avrdude -c arduino -p m328p -P COMx -b 115200
```
- **Signature `0x1E 95 0F` lue** → bootloader Arduino présent → flash DTR (§2a).
- **`not in sync` / timeout** → pas de bootloader série → flash ISP (§2b).

### Étape 2a — flash nominal (bootloader + DTR)
```
pio run -e grovepi -t upload        # ajuster upload_port dans platformio.ini
```

### Étape 2b — flash par ISP (voie retenue sur cette carte)
Historique : cet ATmega n'a **jamais eu de bootloader série exploitable** → on flashe par
**ISP** (USBtinyISP ou « Arduino as ISP ») câblé sur l'**ICSP 6 broches** :
```
pio run -e isp -t fuses             # une fois : (re)grave lfuse/hfuse/efuse (voir platformio.ini)
pio run -e isp -t upload            # puis flash firmware.hex par ISP
```
> `hfuse=0xD9` (BOOTRST déprogrammé) : au reset le CPU va droit à `0x0000` (notre code), sans
> dépendre d'un bootloader. Pour passer à un flash FTDI/DTR plus tard : installer Optiboot
> (`hfuse=0xDE`) et câbler DTR→100 nF→RESET (auto-reset).

## 10. Vérification (banc de test)

```
firmware/grovepi_bamboo/tools/check.py COMx [test|all]
```
Suite de tests nommés (PASS/FAIL, code de sortie agrégé) :
`connexion`, `version` (contact), `status`, `get_config`, `set_config` (round-trip),
`enable_disable` (IR), `ultra_pins`, `timesync` (echo horloge : RTT/offset), `timestamp`
(ts croissant, Δts ≈ période), `auto_report` — et, **exclus de `all`** car ils écrivent
l'EEPROM : `eeprom_save`, `eeprom_throttle` (prouve l'anti-rafale). `all+eeprom` les inclut.
Les tests config/statut/version passent après reflash ≥ 0.2.0 ; `timesync`/`timestamp` exigent ≥ 0.3.0.
À lancer **sur le PC**, adaptateur branché, aucun autre moniteur série ouvert.

## 11. Points ouverts / limites

- **Réactivité** : IMU à chaque tour ; 4 HC-SR04 en round-robin (un ping/tour, portée ≤ 250 cm).
  `pulseIn` reste bloquant : lib `NewPing` (timer non bloquant) envisageable plus tard.
- **Bus I²C partagé** : ports I²C Grove sur A4/A5 — surveiller les collisions d'adresses si
  d'autres capteurs I²C sont ajoutés.
- **Sharp IR** : validation matérielle reportée (device secondaire) ; formule à recaler par
  exemplaire via l'`adc_brut` de `0x62`.
- **Nœud-pont ROS 2** : cible d'intégration, non implémenté dans ce dépôt.

## 12. Horodatage & synchronisation (v0.3)

Objectif : fournir à ROS 2 des mesures **horodatées** pour **fusionner les métriques de
plusieurs cartes** (STM32 + GrovePi + …) dans un même référentiel temporel.

### 12.1 Horloge interne — monotone, autonome, sûre

- Module [`lib/Clock/`](lib/Clock/) : compteur **`uint32` de millisecondes** piloté par une
  **interruption matérielle dédiée** — **Timer2 en mode CTC @ 1 kHz** (prescaler 64,
  `OCR2A = 249` → 250 kHz/250 = 1 IRQ/ms). `ISR(TIMER2_COMPA_vect)` incrémente le compteur.
- **Indépendante du core Arduino** : `millis()`/`delay()` reposent sur Timer0 ; on garde un
  référentiel qu'on maîtrise. Timer2 est libre ici (ni `tone()`, ni `analogWrite` sur D3/D11).
- **Autonome** : l'ISR tourne quoi qu'il arrive, y compris pendant `pulseIn()` (qui laisse les
  interruptions actives) — le compteur ne dépend d'aucun appel côté `loop()`.
- **Lecture atomique** : `nowMs()` fait `s=SREG; cli(); m=_ms; SREG=s;` → pas de déchirure de
  lecture du compteur 32 bits pendant que l'ISR l'incrémente.
- **Monotone** : l'horloge **ne saute jamais et ne recule jamais** (sauf reboot → repart de 0).
  C'est le point clé pour ROS : une horloge qui recule casse la fusion / les buffers temporels.

### 12.2 Timestamp des trames report

Chaque trame `0x60/0x61/0x62` porte un **`ts` (u32 ms) en tête de payload**, capturé à
**l'instant d'émission** (`Clock::nowMs()`). Le jitter sous-boucle (< une période de boucle) est
assumé ; un affinage (estampiller à l'échantillonnage) reste possible. Divergence assumée avec
le format STM32 : la GrovePi a son propre `check.py`.

### 12.3 Synchro aller-retour (TIME_SYNC 0x52)

```
  hôte/ROS                         carte
     |  t1: TIME_SYNC [seq] ------->|
     |                             (echo immédiat, l'horloge n'est JAMAIS modifiée)
     |  t2: <----- [seq][t_board]   |
     RTT   = t2 - t1
     offset ≈ (t1 + t2)/2 - t_board        (ros_time ≈ device_time + offset)
```

- La carte **ne modifie jamais son horloge** : elle **écho** seulement `seq` + `Clock::nowMs()`.
  `seq` corrèle requête/réponse (plusieurs échanges en vol). Traité **directement** dans
  `handleFrame` (hors `REQUEST_DATA`) pour une latence minimale.
- Côté **nœud-pont ROS** (non implémenté ici) : estimer un **offset par carte** (filtré sur
  plusieurs échanges, en écartant les RTT élevés), puis convertir chaque `ts` de report en
  `ros_time = ts + offset`. Chaque carte a son offset → fusion multi-sources cohérente.
- **Reboot** : au reset l'horloge repart de 0 **et** la carte ré-émet `0x51 VERSION`. L'hôte
  détecte la discontinuité (`ts` qui recule / `VERSION`) et **resynchronise** (`boot_id` explicite
  jugé inutile).
