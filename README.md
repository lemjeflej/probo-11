# PROBO-11 — FR3 sur rail linéaire (ROS 2 Humble + Gazebo)

Simulation d'un bras Franka FR3 monté sur un rail linéaire de 1.9 m, avec sonde échographique en end-effector. Objectif : valider une stratégie de placement optimal du robot par rapport à une région anatomique cible.

## Packages

| Package | Rôle |
|---|---|
| `franka_sonde` | URDF, launch, config controllers |
| `controleurs` | Nœud IK + action client (`cartesian_commander`), GUI pose |
| `franka_description` | URDF officiel Franka — **ne pas modifier** |
| `franka_ros2` | Intégration ROS 2, bridge Gazebo — **ne pas modifier** |
| `libfranka` | Lib C++ bas niveau — **ne pas modifier** |

## Installation

```bash
git clone https://github.com/lemjeflej/probo-11.git probo-11
cd probo-11

# Dépendances Franka
vcs import src < franka.repos
vcs import src < src/franka_ros2/dependency.repos

# Sous-modules libfranka (obligatoire)
cd src/libfranka && git submodule update --init --recursive && cd ../..

# Dépendances système
rosdep install --from-paths src --ignore-src -r -y

# Build (~7 min, warnings franka_* normaux)
colcon build --symlink-install
source install/setup.bash
```

## Utilisation

### Lancer la simulation

```bash
ros2 launch franka_sonde gazebo_complet.launch.py
```

Séquence automatique : Gazebo → robot_state_publisher → joint_state_broadcaster → fr3_arm_controller → cartesian_commander (pose ready).

### Lancer le GUI de commande (terminal séparé)

```bash
ros2 run controleurs pose_gui.py
```

### Rebuild après modification

```bash
colcon build --packages-select franka_sonde controleurs
source install/setup.bash
```

## Repères et coordonnées

**IMPORTANT** : les poses envoyées au robot sont exprimées dans le frame `world`.

La chaîne cinématique depuis `world` jusqu'à `fr3_link0` introduit deux offsets :

```
world
 └─ table           (z = 0)
     └─ rail        (z = 0,   y = 0)
         └─ montage (y = -0.85 m   ← origin du rail_joint)
             └─ fr3_link0 (z = +1.04 m  ← montage_to_arm)
```

**fr3_link0 est à `(x=0, y=-0.85, z=1.04)` dans world.**

Pour commander le TCP, penser en coordonnées relatives à `fr3_link0` puis ajouter les offsets :

| Axe | Offset à ajouter |
|---|---|
| y | −0.85 m |
| z | +1.04 m |

### Exemple de pose valide

```
x =  0.4 m
y = -0.85 m   (= 0.0 relatif au robot)
z =  1.5 m    (= 0.46 m au-dessus de fr3_link0)
roll = 180°, pitch = 0°, yaw = 0°   (sonde vers le bas)
```

L'espace de travail accessible (en frame world) est approximativement :

```
x : 0.1  →  0.8 m
y : -1.5 →  -0.1 m
z : 1.1  →  1.9 m
```

## Architecture du système

```
pose_gui.py  →  /target_pose  →  cartesian_commander
                                      │
                                   IK (KDL)
                                      │
                              /fr3_arm_controller/
                              follow_joint_trajectory
                                      │
                                   Gazebo
```

Le `cartesian_commander` :
- Construit la chaîne KDL `world → sonde_tcp` au démarrage
- Envoie le robot en pose `ready` automatiquement
- Résout l'IK depuis la config courante (fallback sur home si échec)
- Publie `/cartesian_commander/busy` pour bloquer le GUI pendant le mouvement

## Dépannage

**IK impossible (code -5)** : la pose est hors espace de travail. Vérifier les offsets y/z décrits ci-dessus.

**`fr3_arm_controller` non chargé** : le yaml shadow `franka_sonde/config/franka_gazebo_controllers.yaml` doit être installé. Rebuilder `franka_sonde` et re-sourcer.

**libfranka ne compile pas** : sous-modules non initialisés. Relancer `cd src/libfranka && git submodule update --init --recursive`.

**Warnings `allow_nonzero_velocity_at_trajectory_end`** : avertissement dépréciation de `joint_trajectory_controller`, sans impact fonctionnel.
