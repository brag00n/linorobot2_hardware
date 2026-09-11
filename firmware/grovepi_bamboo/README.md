# GrovePi+ Bambou — carte capteurs ATmega328P (IMU + ultrasons)

> **Statut : amorce testable.** La **GrovePi+ v3.0** n'est **pas** utilisée ici avec
> un Raspberry Pi : on **détourne son ATmega328P** (quartz 16 MHz = un Arduino Uno)
> en **microcontrôleur capteurs autonome**, flashé depuis le **PC** via un
> adaptateur USB↔série, qui parle le **même protocole de trames que la STM32
> Bamboo** (cf. [`../stm32_bamboo/tools/ros_monitor.py`](../stm32_bamboo/tools/ros_monitor.py)).
> La carte devient une sœur de `stm32_bamboo` : une **carte à flasher**, pas un
> appareil Linux comme `unitv2_bamboo`.

---

## 1. Rôle dans l'architecture Bambou

L'ATmega328P agrège deux familles de capteurs et les diffuse sur son UART :

- **IMU MPU6050** (I²C) → angle **roll/pitch fusionné à bord** (filtre complémentaire)
  + accéléro/gyro bruts. Le yaw reste fourni par l'IMU de la STM32 (pas de doublon).
- **4× HC-SR04** (ultrason) → distances, pour l'évitement d'obstacle.

```
  [ MPU6050 ]──I2C(A4/A5)──┐
                           ▼
  [ 4x HC-SR04 ]──D2..D9──▶[ ATmega328P @16MHz ]──UART──▶ USB<->serie (PL2303TA) ──▶ PC
                                                          trames [0xFF][ID][LEN][FUNC][..][CHK]
```

Intégration ciblée : un nœud-pont ROS 2 (côté SBC/PC) lit ces trames et publie
`sensor_msgs/Imu` (partiel : roll/pitch) et 4× `sensor_msgs/Range` — symétrique du
pont série STM32 existant.

## 2. Matériel

| Élément | Valeur |
|---|---|
| MCU | ATmega328P, **quartz 16 MHz** (⇒ cible PlatformIO `uno`, 5 V) |
| IMU | **MPU6050** I²C @ 0x68 (A4=SDA, A5=SCL) |
| Télémètres | **4× HC-SR04** (Trig/Echo, 5 V — direct, pas de level-shift) |
| Adaptateur flash | **PL2303TA** (GND/CTS/VCC/TXD/RXD/DTR, sélecteur 3v3/**5 V**) |

## 3. Câblage

- **MPU6050** sur un **port I²C Grove** (= bus A4/A5 de l'ATmega). VCC 5 V, GND, SDA, SCL.
- **HC-SR04** : **1 capteur par port digital Grove** (le connecteur Grove a 2 lignes
  signal → Trig sur la primaire, Echo sur la secondaire). Défaut firmware :
  `Trig = D2/D4/D6/D8`, `Echo = D3/D5/D7/D9`. Adapter `TRIG_PIN[]`/`ECHO_PIN[]`
  dans [`src/main.cpp`](src/main.cpp) au mapping réel des ports Grove.
- **Adaptateur USB↔série** sur le **header 6 broches** de la GrovePi+
  (GND/CTS/VCC/TXD/RXD/DTR). **Sélecteur sur 5 V.** Croiser TXD↔RXD.
  DTR est requis pour l'auto-reset (flash par bootloader).
- ⚠️ Ne pas utiliser **D0/D1** (UART) ni **A4/A5** (I²C) pour les ultrasons.

## 4. Flash — procédure

### Étape 0 — valider le PL2303TA (Windows 11)
Beaucoup de PL2303(TA) sont bloqués par le driver Prolific récent (« code 10,
contrefait »). Brancher l'adaptateur, vérifier dans le **Gestionnaire de
périphériques** qu'un **COMx monte sans erreur**. Repli : driver Prolific
**legacy 3.3.2.102**. Sans port stable, rien ne marche.

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

### Étape 2b — flash de secours (ISP)
Nécessite un programmateur (Arduino en « Arduino as ISP », ou USBtinyISP) câblé
sur l'**ICSP 6 broches** :
```
pio run -e isp -t bootloader        # une fois : (re)grave le bootloader
pio run -e isp -t upload            # puis flash par ISP
```

## 5. Vérification
```
python firmware/grovepi_bamboo/tools/check.py COMx
```
Valide **connexion / version / auto-report** (IMU 0x60 + ultrason 0x61), et affiche
le dernier angle roll/pitch et les 4 distances. Code de sortie 0 = PASS.
À lancer **sur le PC**, adaptateur branché, aucun autre moniteur série ouvert.

## 6. Protocole série

Trame little-endian, miroir de la STM32 :
```
[0xFF][ID][LEN][FUNC][donnees...][CHK]
  ID  : 0xFC hote->carte, 0xFB carte->hote
  LEN : taille_totale - 2
  CHK : somme(octets[2..fin-1]) & 0xFF
```

| FUNC | Sens | Payload |
|---|---|---|
| `0x50` REQUEST_DATA | hôte→carte | `[subfunc]` → renvoie la trame `subfunc` |
| `0x51` VERSION | carte→hôte | `major, minor` (uint8) |
| `0x60` REPORT_IMU | carte→hôte | `roll, pitch` (int16, deg×100) + `ax,ay,az,gx,gy,gz` (int16 bruts) |
| `0x61` REPORT_ULTRA | carte→hôte | `dist[4]` (uint16, mm ; `0xFFFF` = pas d'écho) |

IMU et ultrasons sont **auto-diffusés ~20 Hz** ; `REQUEST_DATA` permet en plus la
lecture à la demande.

## 7. Points ouverts / limites

- **Bootloader** : présence à confirmer à l'étape 1 (décide DTR vs ISP).
- **Mapping ports Grove → broches ATmega** : à confirmer sur la carte réelle
  (schéma GrovePi+) et reporter dans `TRIG_PIN[]`/`ECHO_PIN[]`.
- **Réactivité** : IMU mis à jour à chaque tour de boucle ; les 4 HC-SR04 en
  **round-robin** (un ping/tour, portée plafonnée à 250 cm) → IMU réactif, chaque
  ultrason ~16 Hz au pire cas. `pulseIn` reste bloquant : lib `NewPing` (timer
  non bloquant) envisageable plus tard si besoin.
- **Bus I²C partagé** : sur la GrovePi+ les ports I²C Grove sont sur le bus A4/A5
  de l'ATmega — surveiller collisions d'adresses si d'autres capteurs I²C sont ajoutés.
