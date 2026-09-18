# Migration ROS2 — mapping des nodes roslite v3

> **But du document.** Le framework maison `roslite` (bus mono-thread, files
> KEEP_LAST(1), messages = dataclasses Python — cf. [`../msgs.py`](../msgs.py)) a été
> écrit pour préparer une migration *mécanique* vers ROS2. Ce document établit, node par
> node, la correspondance E/S → primitive ROS2 (topic / parameter / service / action /
> driver matériel) et le type de message associé. Les squelettes d'interfaces `.msg` /
> `.srv` / `.action` sont fournis dans [`ros2_interfaces/`](ros2_interfaces/).

## 1. Toutes les E/S ne sont pas des topics

En ROS2 un **topic** est un flux de données périodique *sans réponse*. Le bus roslite,
lui, transporte aussi de la config et des commandes ponctuelles (motif `seq` incrémenté
sur file profondeur 1 = contournement de l'absence de req/rép). La migration doit donc
répartir les « E/S » actuelles sur **quatre** primitives distinctes.

| Nature réelle | Exemple actuel | Primitive ROS2 |
|---|---|---|
| Flux de données périodique | `/camera/image`, `/board/telemetry`, `/servo/state` | **Topic** + msg |
| Config runtime | `/tracking/config`, `/recognition/config` (mode, detector…) | **Parameter** (+ callback `add_on_set_parameters_callback`) |
| Commande ponctuelle (motif `seq`) | `ServoCmd` center/nudge, `RecognitionConfig.command` | **Service** (`.srv`, req/rép) |
| Tâche longue avec avancement | `train` + `/recognition/train_state` | **Action** (`.action`, goal/feedback/result) |
| Ressource matérielle partagée | `link` série STM32 (BoardNode **et** ServoNode) | **1 node driver** unique (§5) |
| Edges externes | clavier, manette (`/joy`), socket MCP, disque `faces/` | node teleop / `joy_node` / outils natifs / FS |

## 2. Topics → types de messages ROS2

| Topic roslite | Type actuel | Type ROS2 | Note |
|---|---|---|---|
| `/camera/image` | `ImageMsg(seq, stamp, frame)` | `sensor_msgs/Image` (ou `CompressedImage`) | `seq`+`stamp` → `std_msgs/Header`; QoS *sensor_data* |
| `/board/telemetry` | `BoardTelemetry` | **éclater** : `sensor_msgs/BatteryState` + `sensor_msgs/Imu` + `geometry_msgs/TwistStamped` + `WheelRpm` (custom) | **`ts_*` → `header.stamp`** (datation source); `*_age` supprimé (comparaison de stamps) |
| `/grovepi/telemetry` | `GrovePiTelemetry` | `sensor_msgs/Range` ×4 + `sensor_msgs/Imu` + `sensor_msgs/Range` (IR) | idem `ts`→`header.stamp` |
| `/servo/state` | `ServoState(angleH, angleV…)` | `sensor_msgs/JointState` | pan/tilt = 2 joints nommés |
| `/tracking/result` | `TrackingResult` | `vision_msgs/Detection2DArray` + `TrackState` (custom) | détections standard + état suivi |
| `/tracking/metrics` | `TrackingMetrics` | `diagnostic_msgs/DiagnosticArray` | métriques ≈ diagnostics |
| `/recognition/result` | `RecognitionResult` | `RecognitionResult` (custom) | pas d'équivalent std direct |
| `/servo/cmd` (cible suivi) | `ServoCmd(kind="track", nx, ny)` | `geometry_msgs/PointStamped` | **seule la cible reste un topic** |
| `/servo/cmd` (center/nudge/home) | `ServoCmd(kind="center"/"nudge_*")` | **Service** (§3) | commandes ponctuelles |
| `/joy` | `JoyMsg(axes, buttons, hats, connected, name, seq)` | `sensor_msgs/Joy` | mapping 1:1 (`axes`/`buttons`); `seq`+stamp → `header`; QoS *sensor_data* |

## 3. Config : comment ROS2 la gère

Les 3 canaux actuels (args CLI, clavier, socket MCP) se répartissent ainsi :

| Config actuelle | Canal actuel | Mécanisme ROS2 | Réglage |
|---|---|---|---|
| Tunables statiques (`det_conf`, `pan_gain`, `recog_cos_thr`, `cpr`, `faces_dir`…) | args argparse | **Parameters** (`declare_parameter`) | YAML de launch, `ros2 param set`, CLI |
| Toggles runtime (`active` F, `detector` M, `track_mode` T, `predict_mode` P, `recog_mode` R/A) | touches + MCP | **Parameters + `add_on_set_parameters_callback`** | « dynamic reconfigure » moderne; GUI **rqt_reconfigure** |
| Commandes ponctuelles (`center` C, nudge, `recognize_file`, `acquire_file`, switch caméra V) | touches + MCP | **Services** (`.srv`) | `ros2 service call …`; supprime le motif `seq` |
| Apprentissage (`train` G) | touche + MCP | **Action** (`.action`) | feedback = lignes de `TrainState`; result = `summary`; annulable |
| Pilotage moteur clavier (flèches, vitesse) | `_process_key` (Core) | **node teleop** → `geometry_msgs/Twist` sur `/cmd_vel` | ≈ `teleop_twist_keyboard` |
| Pilotage moteur manette (sticks, boutons) | `GamepadNode`→`/joy`, `_drive_from_gamepad`/`_gamepad_buttons` (Core) | **`joy_node`** → `/joy`, **`teleop_twist_joy`** → `/cmd_vel` | mapping arcade; boutons = params/services std |
| Interface MCP (socket 8787) | `gateway.CommandServer` | **redondant** → `ros2 topic/param/service`, rqt (ou fin node-pont) | le gateway disparaît |

## 4. Récap par node en interfaces ROS2

| Node | Subscriptions | Publications | Parameters | Services / Actions | Point dur |
|---|---|---|---|---|---|
| **CameraNode** | — | `image` | size, index, fps, flip, rotate… | `~/switch_camera` (`std_srvs/Trigger`) | driver matériel (ou `v4l2_camera`) |
| **TrackingNode** | `image` | `detections`, `metrics`, `target` (`PointStamped`) | detector, track_mode, predict_mode, **active** (+callback) | — | config = params, plus de `/tracking/config` |
| **ServoNode** | `target` (`PointStamped`) | `joint_states` | gains, limites, deadzone… | `center` (`Trigger`), `nudge` (`Nudge`) | **écrit sur le série STM32** → §5 |
| **BoardNode** | `/cmd_vel` (`Twist`), servo | `battery`, `imu`, `twist` (**stamp = ts carte**) | cpr, port, baud | — | **= driver STM32** (lecture+écriture) |
| **GrovePiNode** | — | `range`×4, `imu`, `ir` | port, baud | — | driver série propre (port distinct → OK) |
| **GamepadNode** | — | `joy` (`sensor_msgs/Joy`) | index, deadzone, expo | — | ≈ `joy_node` std; le Core = `teleop_twist_joy` (`/joy`→`/cmd_vel`) |
| **FaceRecogNode** | `image`, `detections`, `train_state` | `recognition` (custom) | faces_dir, seuils, mode (+callback) | `recognize_file`, `acquire_file` (`.srv`) | écritures disque = effet de bord |
| **FaceTrainNode** | *(goal d'action)* | `train_state` → **feedback d'action** | seuils, min_imgs | **`train` (`TrainFaces.action`)** | worker thread → serveur d'action |

## 5. Point dur : le série STM32 partagé

Aujourd'hui `BoardNode` (lecture) **et** `ServoNode` (écriture servo) **et** le Core
(écriture moteur) partagent le **même objet `link` in-process** (`RobotComSerial`, injecté
par `RobotMain.setup`). En ROS2 ce sont des **process séparés** : impossible d'ouvrir deux
fois le même `/dev/ttyUSB`. Solution canonique : **un seul node driver** possède le port.

```
                 ┌─────────── stm32_driver (owns /dev/ttyUSB0) ───────────┐
   /cmd_vel   ──▶│ sub geometry_msgs/Twist                                 │
   /servo/cmd ──▶│ sub PointStamped (cible pan/tilt)                       │
   (srv nudge/  )│                    [série Yahboom v4 + ros_monitor.py]  │──▶ /battery   (BatteryState)
   (   center  )─▶ srv                        │                            │──▶ /imu       (Imu, stamp=ts)
                 │                             ▼                            │──▶ /odom/twist(TwistStamped)
                 └──────────────────────────────────────────────────────── ┘──▶ /joint_states(JointState)
```

`BoardNode` + `ServoNode` fusionnent dans ce driver (ou l'écriture servo lui est adressée
par service). C'est là que le protocole [`../../stm32_bamboo/tools/ros_monitor.py`](../../stm32_bamboo/tools/ros_monitor.py)
(FrameParser + décodeurs, **dont le timestamp**) se reloge. Alternative standard mais plus
lourde : interface matérielle **ros2_control**.

## 6. Retombée du timestamp horloge interne

Le timestamp u32 LE ms ajouté aux 4 trames de métriques STM32 (commits `53eba71`/`4f36b0d`,
convention identique à la carte GrovePi) est exactement ce dont ROS2 a besoin pour remplir
`header.stamp` **à la source** (datation capteur) au lieu de l'instant de réception hôte.
C'est la condition d'une fusion propre STM32 + GrovePi sur base temps commune
(`message_filters::TimeSynchronizer`, arbre TF). Les `*_age` calculés côté hôte ne sont
qu'un substitut de fraîcheur; en ROS2 ils deviennent la comparaison des `header.stamp`.

## 7. QoS

- `image`, `range`, `imu` (données capteur haute cadence) → profil **`sensor_data`**
  (best-effort, depth 1) : équivaut au KEEP_LAST(1) best-effort actuel.
- `battery`, télémétrie d'état, `joint_states` → **reliable**, depth 1.
- Paramètres et services → transport fiable natif (pas de QoS à régler).

## 8. Package d'interfaces proposé

```
bamboo_interfaces/            # package ament_cmake dédié aux .msg/.srv/.action
├── msg/
│   ├── TrackState.msg        # partie suivi non couverte par vision_msgs
│   ├── RecognitionResult.msg
│   └── WheelRpm.msg          # rpm M1..M4 (sinon Float32MultiArray)
├── srv/
│   ├── Nudge.srv             # pan/tilt relatif (center/switch_camera = std_srvs/Trigger)
│   ├── RecognizeFile.srv
│   └── AcquireFile.srv
└── action/
    └── TrainFaces.action
```

Squelettes fournis dans [`ros2_interfaces/`](ros2_interfaces/) — à recopier dans un vrai
package `bamboo_interfaces` (avec `rosidl_default_generators` dans `package.xml` /
`CMakeLists.txt`).
