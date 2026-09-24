# Carte Yahboom « ROS Robot Control Board V3.0 » (STM32F103RCT6)

Fiche matérielle de référence de la carte de contrôle du robot `bamboo4WD_V4_YBStm32`
(nom d'usage **bambooSTM32YB**). Microcode maison : [`../firmware/stm32_bamboo/`](../firmware/stm32_bamboo/).

- **Nom constructeur** : Yahboom *ROS robot control board V3.0 with STM32F103RCT6*
  (annoncée pour RaspberryPi 5 / Jetson / RDK X3).
- **Rôle projet** : **unique** interface de puissance et de capteurs — elle pilote les 4 moteurs,
  lit les 4 encodeurs, porte l'IMU, commande les servos, **et alimente le RPi4 depuis son propre
  VIN**. Il n'y a donc **aucun abaisseur dans le robot** (voir §2).
- **Sources** : page produit Yahboom (`category.yahboom.net/collections/stm32/products/ros-driver-board`),
  dossier constructeur Drive (sous-dossiers AM2857, MPU9250, SCH, STM32F103RCT6), et le microcode
  lui-même — qui est la source la plus fiable des trois pour tout ce qui est chiffré ici.

> ⚠️ **Bande d'entrée interdite : 8,6 – 9,5 V.** Le microcode déclenche son alarme basse tension à
> 9,6 V mais **filtre** la plage 6,5–8,5 V (fenêtre héritée des packs 2S). Une entrée réglée « vers
> 9 V » tombe donc dans un trou : ni filtrée, ni normale → **arrêt latchant au bout de 2 s**,
> récupérable **seulement par reset**. Deux fenêtres sûres : **6,5–8,5 V** ou **9,6–13,0 V**.

---

## 1. Caractéristiques générales

| Élément | Valeur |
|---|---|
| MCU | **STM32F103RCT6** (72 MHz, Cortex-M3) |
| Entrée d'alimentation (VIN) | **12 V DC** — « This board just support 12VDC input », maximum annoncé **12 V** |
| Sortie calculateur | alimente le **RPi4** depuis VIN ; « supports the Raspberry Pi 5 power supply protocol » |
| Ampérage du rail 5 V | `<< NON PUBLIÉ — à relever >>` |
| Étage de puissance moteurs | **AM2857** (identifié par le dossier constructeur), 4 canaux |
| Courant par canal moteur | `<< NON PUBLIÉ — à relever >>` |
| Encodeurs | **4 voies** (contrairement à la General Driver WaveShare qui n'en a que 2) |
| Alimentation encodeur | broche **3,3 V** dédiée sur le header moteur ; la tension moteur, elle, est VIN **brut** |
| IMU | **MPU9250** — **avec magnétomètre**, donc **cap absolu possible** (impossible sur MPU6050) |
| Servos | 4 voies **PWM** + servos de **bus série** |
| Tension du rail servo | `<< NON PUBLIÉ — À MESURER AVANT TOUT BRANCHEMENT >>` (voir §5) |
| Buzzer | embarqué, commandable par l'hôte (`FUNC_BEEP 0x02`, `bsp_beep.c`) |
| Liaison hôte | UART / **CH340** `1A86:7523` ; protocoles **MAVLink v2 (bamboo)** ou trames Yahboom |

---

## 2. Alimentation : architecture **zéro abaisseur**

```
Pack Li-ion 3S2P 7 Ah ──VIN 12,6 → 9,6 V──▶ [ Yahboom ROS Control Board V3.0 ]
  (BMS interne, inaccessible)                 ├──VIN brut──▶ 4 moteurs CHR-GM25-310 (AM2857)
                                              ├──régulé────▶ RPi4  (+ lidar D500 / caméra en USB)
                                              ├──3,3 V─────▶ encodeurs
                                              └──ADC───────▶ surveillance batterie (§3)

        2 × SG90 (caméra motorisée) ◀──BEC 5 V dédié pris sur le pack── (signal seul vers la carte, §5)
```

**3S2P** = 3 cellules en **S**érie (11,1 V nominal / 12,6 V chargé / 9,0 V plancher à 3,0 V par
cellule) × 2 chaînes en **P**arallèle (2 × 3,5 Ah = 7 Ah). La tension vient du S, la capacité du P.

Pourquoi aucun abaisseur : la carte est **conçue pour ce pack**. Ses propres seuils d'alarme
(9,6 V bas / 13,0 V haut, §3) sont ceux d'un 3S, et elle régule elle-même le rail du RPi. Insérer un
abaisseur 8 V serait faux trois fois : hors plage d'entrée annoncée, ~1,5× de courant d'entrée pour
le rail RPi, et la carte lirait une tension constante — **aveugle au pack**, donc plus aucune alarme
utile.

> ⚠️ **Marge à vérifier** : le pack chargé donne **12,6 V**, la carte annonce **max 12 V**. Le
> microcode ne s'en émeut qu'à 13,0 V, mais l'écart n'est pas documenté côté matériel.
> `<< À CONFIRMER sur le schéma (dossier SCH) avant première mise sous tension pack plein >>`

Ce qui n'est **pas** dans le robot, et c'est un choix : aucun module DC-DC, aucune isolation
galvanique (elle exigerait un transformateur — deux régulateurs sur un même pack ne sont pas
isolés, seulement régulés séparément, masse commune obligatoire pour les signaux servo et l'USB).
L'INA219 reste **optionnel**, utile pour le **courant** seulement : la tension, la carte la mesure.

---

## 3. Surveillance batterie du microcode (native, elle fonctionne)

`src/app_bat.c`, activée par `ENABLE_LOW_BATTERY_ALARM 1` (`src/config.h:14`).

| Seuil | Valeur | Origine |
|---|---|---|
| Basse tension | **9,6 V** | `Bat_Get_Low_Voltage()` → 96 (3,2 V/cellule) |
| Surtension | **13,0 V** | `Bat_Get_Over_Voltage()` → 130 |
| Fenêtre filtrée | **6,5 – 8,5 V** | commentaire « 过滤6.5-8.5之间的低电压报警功能 » (`:55-62`) |
| Temporisation | **2 s** | `BAT_CHECK_COUNT 20` × 100 ms |

Hors normale, `g_system_enable = 0` : **arrêt latchant**, reset obligatoire. Ce n'est **pas une
panne** : à 9,6 V le robot s'immobilise pendant que le RPi continue de tourner — donc il reste
joignable en SSH et par MCP pour être arrêté proprement. C'est le comportement voulu.

Le pack déclenchera donc l'alarme carte (9,6 V) **avant** son plancher cellule (9,0 V), ce qui
laisse la marge du bon côté. Correspondance utile à garder en tête, du plein au vide :
**12,6 V** chargé · **11,1 V** nominal · **10,5 V** à surveiller · **9,6 V** arrêt carte ·
**9,0 V** plancher dur (BMS interne, inaccessible).

Le buzzer étant sur cette carte et commandable (`FUNC_BEEP 0x02`), l'alerte sonore n'a besoin
d'aucun matériel supplémentaire — pas d'ESP32 dans la boucle.

---

## 4. Protection des moteurs 7,4 V : par les constantes PWM

**Le microcode n'a aucune notion de tension moteur** — pas de constante, aucune mise à l'échelle :
**la tension du pack arrive brute sur le moteur**. Des moteurs **CHR-GM25-310 7,4 V / 275 RPM** sur
un pack 12,6 V sont donc à 170 % de leur nominal si rien ne borne le rapport cyclique.

Ce qui rend la protection logicielle légitime : `lib/BSP/bsp_motor.c:28-46` règle la PWM à
**20 kHz** (prescaler 0 sur 72 MHz, ARR 3600). La constante électrique d'un GM25 (~0,3–1 ms) valant
6 à 20 fois la période de 50 µs, l'inductance **moyenne** les impulsions : **borner le rapport
cyclique, c'est borner la tension.**

⚠️ **Piège** : `MOTOR_IGNORE_PULSE` n'est pas un plancher, c'est un **décalage additif**
(`bsp_motor.c:13-19` : `pulse + MOTOR_IGNORE_PULSE`), et le PID sature à
`±(MOTOR_MAX_PULSE − MOTOR_IGNORE_PULSE)` (`src/app_pid.c:108-111`). La plus petite consigne non
nulle applique donc **déjà** 1600/3600 = 44 % = **5,6 V**. Borner le seul plafond monterait la
vitesse minimale à ~208 tr/min : inexploitable. **Les deux constantes se mettent à l'échelle
ensemble** (`lib/BSP/bsp_motor.h:80-92`) :

| Constante | Aujourd'hui | Cible 7,4 V | Sur le moteur |
|---|---|---|---|
| `MOTOR_IGNORE_PULSE` | 1600 (44 %) | **~940** (26 %) `<< À MESURER, test T2 >>` | 5,6 V → 3,3 V |
| `MOTOR_MAX_PULSE` | 3600 (100 %) | **2114** (59 %) | 12,6 V → **7,4 V** |

Variante préférable, **compensée en tension** — `Bat_Voltage_Z10()` renvoie déjà VIN × 10, donc le
plafond de 7,4 V se tient sur toute la décharge au lieu de dériver :

```c
max_pulse = (MOTOR_MAX_PULSE * 74) / Bat_Voltage_Z10();   /* 74 = 7,4 V x10 */
/* 12,6 V -> 2114   ·   11,1 V -> 2400   ·   9,6 V -> 2775 */
```

Les deux constantes sont **centralisées** : `mav_protocol.c:388`, `protocol.c:521` et `protocol.c:1006`
en dérivent tous, aucun littéral dispersé. À compléter par une **rampe d'accélération** (le BMS du
pack est inaccessible, donc la limitation d'appel de courant doit venir de notre côté) et un
condensateur de découplage sur VIN.

---

## 5. Servos SG90 (caméra motorisée, 2 axes)

Chaîne **entièrement logicielle**, donc entièrement lisible : `bsp_timer.c:42-43` donne une
interruption toutes les **10 µs** (prescaler 71 → 1 MHz, période 9) et `bsp_pwmServo.c:10-46` compte
jusqu'à 2000 avant de reboucler → **50 Hz exactement**, la fréquence nominale du SG90. PWM par GPIO
(`PCout(3..0)` pour J1–J4), pas de timer matériel. Conversion (`bsp_pwmServo.c:155`) :

```c
g_angle_num[index] = (angle * 11 + 500) / 10;   /* largeur = 11 x angle + 500 us */
```

| Angle demandé | Largeur émise | SG90 (plage 500–2400 µs) |
|---|---|---|
| 0° | 500 µs | limite basse exacte |
| 90° | **1490 µs** | neutre à 10 µs près — écart invisible |
| 172° | 2392 µs | dernier angle sain |
| **180°** | **2480 µs** | **80 µs au-delà de la butée** |

⚠️ Demander 180° pousse le SG90 **dans sa butée mécanique** : il ne bouge plus mais continue de
forcer (~700 mA en continu), il bourdonne, chauffe, et les pignons plastique y passent. Sur une
caméra qui reste en position des minutes entières, c'est la panne classique.
→ **Borner la course à 8°–172° côté hôte** : `PwmServo_Set_Angle` reçoit la valeur de l'hôte, le
clamp vit donc dans le pilotage pan/tilt, **sans reflash**.

⚠️ **Tension du rail servo inconnue et non publiée.** La carte annonce aussi des servos de **bus
série**, famille qui tourne en 6–12 V chez Yahboom. Si le header partage ce rail, brancher un SG90
(4,8–6 V) le détruit **au premier contact**. Deux routes, dans cet ordre :

1. **Mesurer** le V+ du header au multimètre, carte alimentée, moteurs à l'arrêt (test **T1**).
   Obligatoire dans tous les cas, avant le premier branchement.
2. **Ne pas dépendre de la réponse** : les deux SG90 sur leur **propre BEC 5 V** pris sur le pack,
   et vers la carte **le fil de signal seul**, masse commune. C'est la route retenue même si T1
   donne 5 V : deux SG90 en butée, c'est ~1,4 A impulsionnel injecté dans le régulateur qui
   alimente le STM32 — exactement le couplage qu'on retire du rail moteur au §4.

---

## 6. Encodeurs et `counts_per_rev`

**cpr = 1320**, cohérent par deux chemins indépendants : la fiche des moteurs CHR-GM25-310, et la
dérivation du microcode lui-même (`src/app_motion.h:8-25`) —
`30 (réduction) × 11 (lignes) × 4 (quadrature) = 1320` (`ENCODER_CIRCLE_330`, `WHEEL_GEOM_DEF_CPR`).

Les moteurs n'étant **pas** des Yahboom, leur régime nominal (275 RPM) est absent de la table du
microcode (205 / 330 / 450 / 550) : cette absence n'invalide rien, seul le train d'engrenages entre
dans le cpr. **À valider par comptage réel sur les 4 voies, roues surélevées** (test **T3**).

---

## 7. Tests à passer **avant toute actuation**

Numérotation **propre à ce robot** : ces `Tn` repartent de **T1** et ne sont pas les `Tn` de
`bamboo4WD_V4_WSEsp32` (série arrivée à T22). Un numéro d'essai ne vaut donc **que** qualifié par son
robot : toujours dire « T2 de bambooSTM32YB », jamais « T2 » seul.

| N° | Objet | Condition |
|---|---|---|
| **T1** | V+ du header servo au multimètre | carte alimentée, moteurs arrêtés, **avant** tout branchement servo |
| **T2** | bande morte réelle : rapport cyclique minimal de démarrage, par moteur | **roues surélevées** — donne le `MOTOR_IGNORE_PULSE` du §4 |
| **T3** | comptage encodeur sur 10 tours manuels, 4 voies | roues surélevées, aucun couple moteur → confirme cpr 1320 |
| **T4** | seuils batterie : lecture `/battery` contre multimètre sur VIN | vérifie l'étalonnage de l'ADC avant de se fier à l'alarme |
| **T5** | matrice de direction (signes `linear.x` / `angular.z`) | roues surélevées, après T2 |

Aucun essai au sol avant T2, T3 et T5 validés.

## Sécurité

- **Ne jamais alimenter en 8,6–9,5 V** : arrêt latchant en 2 s, reset obligatoire (§3).
- **Ne pas brancher un SG90 avant le test T1** : le rail peut être en 6–12 V (§5).
- **Roues surélevées** pour T2, T3 et T5 ; la tension pack arrivant brute sur des moteurs 7,4 V,
  ne pas actionner avant la mise à l'échelle des constantes du §4.
- BMS du pack **enfermé et inaccessible** : la limitation d'appel de courant (rampe) et le plafond
  de rapport cyclique sont les seules protections que nous maîtrisons.
- Masse commune obligatoire entre le BEC servo, la carte et le pack.

## Voir aussi

- [`../firmware/stm32_bamboo/`](../firmware/stm32_bamboo/) — microcode de cette carte.
- [`../tools/robot_control/robots/bamboo4WD_V4_YBStm32.json`](../tools/robot_control/robots/bamboo4WD_V4_YBStm32.json) — profil outillage (port, VID:PID, cpr, protocole).
- [`hardware_waveshare_ups_module_3s.md`](hardware_waveshare_ups_module_3s.md) — alimentation de l'**autre** robot (`bamboo4WD_V4_WSEsp32`), architecture différente : ne pas confondre.
