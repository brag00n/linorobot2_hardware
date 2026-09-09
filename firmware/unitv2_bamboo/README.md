# UnitV2 Bambou — module caméra IA M5Stack UnitV2

> **Statut : WIP (amorce).** Ce dossier accueillera l'intégration de la caméra
> **M5Stack UnitV2** au robot Bambou4WD v4. Rien n'est encore implémenté :
> seule l'arborescence et le cadrage sont posés ici.

---

## 1. Ce qu'est la UnitV2 (à connaître avant d'intégrer)

Contrairement aux autres cibles de `firmware/` (STM32, ESP32, Teensy — des MCU que
l'on **flashe**), la **UnitV2 n'est pas un microcontrôleur** : c'est un **module
caméra IA autonome sous Linux**.

| Élément | Valeur (à confirmer sur l'unité réelle) |
|---|---|
| SoC | SigmaStar **SSD202D** (ARM Cortex-A7 double cœur, ~1,2 GHz) |
| Mémoire | 128 Mo DDR3 |
| OS | **Linux embarqué** (rootfs fourni M5Stack) |
| Caméra | capteur ~GC2145 / OV, sortie jusqu'à 1080p |
| Accélération | ISP + NPU/neurones du SSD202D (modèles fournis) |
| Liaison | **USB-C** exposant un **Ethernet-over-USB** (IP type `10.254.239.1`) + port série |
| Logiciel usine | UI web + « function modules » (détection visages, formes, couleurs,
|          | tags AprilTag, suivi de ligne, classification, lecture code-barres…) |
| Accès dev | **SSH / port série** → Python + OpenCV, sortie **JSON** sur la liaison |

Conséquence : la UnitV2 se comporte comme un **capteur intelligent** qui **pousse
des résultats déjà traités** (détections, poses, coordonnées) — pas un flux
d'images brut à traiter côté hôte. Elle ne se « flashe » pas comme un MCU ; on y
**déploie du code** (script Python / module) et on **consomme sa sortie**.

## 2. Rôle pressenti dans l'architecture Bambou (à décider)

Rappel de l'architecture ROS 2 ciblée du robot (cf.
[`../stm32_bamboo/README.md` §1.3](../stm32_bamboo/README.md)) : cinématique +
odométrie sur le STM32, graphe ROS 2 (Nav2/SLAM/EKF) sur le SBC.

La UnitV2 s'y insère **côté perception**, en amont du SBC. Deux voies possibles,
**à trancher** :

- **A. UnitV2 → SBC (recommandé par défaut).** La UnitV2 est branchée en USB au
  **SBC** (RPi5/Jetson). Un nœud-pont ROS 2 lit sa sortie JSON (série ou TCP
  Ethernet-over-USB) et publie des topics de perception
  (`/detections`, `/apriltag/…`, `/line_pose`…) que Nav2 / la logique de mission
  consomment. Le STM32 n'est pas concerné. **Symétrique du pont série existant.**
- **B. UnitV2 → STM32.** La UnitV2 parle directement au STM32 (UART libre). Plus
  simple électriquement pour un asservissement réflexe (suivi de ligne embarqué),
  mais mélange perception et contrôle bas niveau et complique le protocole série
  Yahboom. À réserver à un besoin temps-réel précis.

> Décision à prendre avec l'utilisateur avant d'écrire du code : **voie A ou B**,
> **transport** (série vs TCP Ethernet-over-USB), et **fonction visée**
> (AprilTag pour recalage de pose ? suivi de ligne ? détection d'obstacle ?).

## 3. Contenu prévu de ce dossier (à venir)

```
unitv2_bamboo/
├── README.md            # ce fichier
├── device/              # code déployé SUR la UnitV2 (script Python / module)
├── bridge/              # nœud-pont côté SBC : sortie UnitV2 -> topics ROS 2
├── docs/                # notes matériel, brochage, protocole de sortie
└── tools/               # utilitaires (dump série, visualisation détections)
```

## 4. Prochaines étapes

1. Brancher la UnitV2, relever l'accès réel (IP Ethernet-over-USB / port série, débit).
2. Choisir la voie d'intégration (§2) et le format de sortie exploité.
3. Spécifier le contrat de données (schéma JSON des détections → messages ROS 2).
4. Implémenter le pont, puis valider bout en bout.

## 5. Références

- Documentation M5Stack UnitV2 : <https://docs.m5stack.com/en/unit/unitv2>
