# PROBO-11 — FR3 sur rail linéaire (ROS 2 Humble + Gazebo)

Simulation d'un bras Franka FR3 monté sur un rail linéaire de 1.9 m, avec sonde échographique en end-effector. Objectif : valider une stratégie de placement optimal du robot par rapport à une région anatomique cible.

## Packages

| Package | Rôle |
|---|---|
| `franka_sonde` | URDF, launch, config controllers, scène patient |
| `controleurs` | Nœud IK + action client (`cartesian_commander`), GUI pose |
| `carte` | Carte de capacité (accessibilité + manipulabilité) + sélecteur rail optimal |
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

# Dépendances Python (carte de capacité)
pip install roboticstoolbox-python spatialmath-python matplotlib numpy

# Build (~7 min, warnings franka_* normaux)
colcon build --symlink-install
source install/setup.bash
```

## Workflow complet

### 1. Déterminer la position rail optimale

```bash
cd src/carte
# Modifier TARGET_WORLD dans optimal_rail_finder.py selon la cible souhaitée
python3 optimal_rail_finder.py
# → affiche le rail_position optimal + la commande de lancement
```

### 2. Lancer la simulation avec ce rail

```bash
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=<valeur>
```

### 3. Commander le bras

```bash
# Terminal séparé
ros2 run controleurs pose_gui.py
```

Entrer la pose cible en frame **`fr3_link0`** (base du bras).

## Position du rail

Fixée au lancement via `rail_position` (0.0 → 1.7 m), baked dans l'URDF.

**Pourquoi** : `franka_ign_ros2_control` construit une chaîne KDL interne pour la compensation gravitationnelle. Un joint prismatique dans la chaîne fait planter cette compensation et désactive tous les controllers. Solution : joint fixe avec position d'origine variable au parsing xacro.

```bash
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=0.0
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=0.5
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=1.2
```

## Carte de capacité

Précalculée par `src/carte/Capacity_Map_Creator.py`, stockée dans `src/carte/results/`.

| Fichier | Contenu |
|---|---|
| `voxels_Z_X.XX.pkl` | Voxels d'une strate Z avec accessibilité (%) et manipulabilité |
| `Voxel_visualisation.py` | Visualisation 3D des cartes |
| `optimal_rail_finder.py` | Sélection automatique du rail optimal pour une cible donnée |

```bash
cd src/carte
python3 Voxel_visualisation.py       # visualiser la carte
python3 optimal_rail_finder.py       # trouver le rail optimal
```

## Repères et coordonnées

Les poses GUI sont en frame **`fr3_link0`** (base du bras).

```
world
 └─ table
     └─ rail
         └─ rail_joint (fixed, origin y = -0.85 + rail_position)
             └─ montage
                 └─ fr3_link0  (z = +1.04 m)
                     └─ [7 joints FR3]
                         └─ fr3_link8
                             └─ sonde_base  (flip 180° autour X)
                                 └─ sonde_tcp  (+0.2 m en Z)
```

Espace de travail accessible (frame `fr3_link0`) :

```
x :  0.1 → 0.8 m
y : -0.7 → 0.7 m
z :  0.1 → 0.9 m
```

## Scène patient

Spawné automatiquement au lancement. Table d'examen + mannequin simplifié positionné devant le robot. Modifier `src/franka_sonde/urdf/patient_scene.urdf` pour ajuster la géométrie.

## Dépannage

**Controllers en `inactive`** : joint prismatique dans l'URDF — vérifier que `rail_joint` est `fixed`.

**IK impossible (code -5)** : pose hors espace de travail en frame `fr3_link0`.

**`fr3_arm_controller` non chargé** : rebuilder `franka_sonde` et re-sourcer.

**libfranka ne compile pas** : `cd src/libfranka && git submodule update --init --recursive`.

**Warnings `allow_nonzero_velocity_at_trajectory_end`** : dépréciation sans impact fonctionnel.
