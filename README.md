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
# Position rail par défaut (0.0 m)
ros2 launch franka_sonde gazebo_complet.launch.py

# Avec une position rail choisie (0.0 → 1.7 m)
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=0.5
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

## Position du rail

La position du rail est fixée **au lancement** via l'argument `rail_position` et baked dans l'URDF. Elle ne peut pas être modifiée dynamiquement sans relancer.

**Pourquoi** : `franka_ign_ros2_control` (plugin Gazebo Franka) construit une chaîne KDL interne pour la compensation gravitationnelle. Cette chaîne traverse toute l'arborescence URDF — un joint prismatique dedans fait planter le solveur KDL et désactive tous les controllers. La solution est de fixer le joint et de changer sa position d'origine au moment du parsing xacro.

Pour tester différentes configurations de placement :
```bash
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=0.0
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=0.5
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=1.2
```

## Controllers actifs

| Controller | Type | Joint(s) |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | tous |
| `fr3_arm_controller` | JointTrajectoryController | fr3_joint1..7 |

## Repères et coordonnées

Les poses envoyées via le GUI sont exprimées dans le frame **`fr3_link0`** (base du bras).

La chaîne KDL du `cartesian_commander` part de `fr3_link0` → `sonde_tcp` (7 joints, bras seul). Le rail est fixe géométriquement pendant la session.

### Exemple de pose valide

```
x =  0.4 m
y =  0.0 m
z =  0.46 m   (≈ 46 cm au-dessus de fr3_link0)
roll = 180°, pitch = 0°, yaw = 0°   (sonde vers le bas)
```

Espace de travail accessible (en frame `fr3_link0`) :

```
x :  0.1 → 0.8 m
y : -0.7 → 0.7 m
z :  0.1 → 0.9 m
```

### Architecture URDF (repères clés)

```
world
 └─ table
     └─ rail
         └─ rail_joint (fixed, origin y = -0.85 + rail_position)
             └─ montage
                 └─ fr3_link0  (origin z = +1.04 m)
                     └─ [7 joints FR3]
                         └─ fr3_link8
                             └─ sonde_base  (flip 180° autour X)
                                 └─ sonde_tcp  (+0.2 m en Z)
```

## Architecture du système

```
pose_gui.py  →  /target_pose  →  cartesian_commander
                                      │
                                   IK KDL (fr3_link0 → sonde_tcp)
                                      │
                              /fr3_arm_controller/follow_joint_trajectory
                                      │
                                   Gazebo
```

Le `cartesian_commander` :
- Construit la chaîne KDL `fr3_link0 → sonde_tcp` au démarrage
- Envoie le robot en pose `ready` automatiquement
- Résout l'IK depuis la config courante (fallback sur home si échec)
- Publie `/cartesian_commander/busy` pour bloquer le GUI pendant le mouvement

## Dépannage

**Controllers en `inactive`** : le plugin Franka ne supporte pas les joints prismatiques dans sa chaîne KDL. Vérifier que `rail_joint` est bien `fixed` dans `srr.xacro` / `rail_sur_table.xacro`.

**IK impossible (code -5)** : pose hors espace de travail en frame `fr3_link0`. Vérifier les valeurs ci-dessus.

**`fr3_arm_controller` non chargé** : rebuilder `franka_sonde` et re-sourcer.

**libfranka ne compile pas** : `cd src/libfranka && git submodule update --init --recursive`.

**Warnings `allow_nonzero_velocity_at_trajectory_end`** : dépréciation `joint_trajectory_controller`, sans impact fonctionnel.
