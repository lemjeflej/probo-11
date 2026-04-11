# PROBO-11 — FR3 sur rail linéaire (ROS 2 Humble + Gazebo)

Simulation d'un bras Franka FR3 sur rail linéaire de 1.9 m avec sonde échographique. Objectif : valider une stratégie de placement optimal du robot par rapport à une cible anatomique.

## Packages

| Package | Rôle |
|---|---|
| `franka_sonde` | URDF, launch, controllers, scène patient |
| `controleurs` | `cartesian_commander` (IK KDL), `pose_gui` |
| `carte` | Carte de capacité + sélecteur rail optimal |
| `franka_description` / `franka_ros2` / `libfranka` | Upstream Franka — **ne pas modifier** |

## Installation

```bash
vcs import src < franka.repos
vcs import src < src/franka_ros2/dependency.repos
cd src/libfranka && git submodule update --init --recursive && cd ../..
rosdep install --from-paths src --ignore-src -r -y
pip install roboticstoolbox-python spatialmath-python matplotlib numpy
colcon build --symlink-install && source install/setup.bash
```

## Workflow

```bash
# 1. Trouver la position rail optimale pour une cible world (x, y, z)
cd src/carte && python3 optimal_rail_finder.py

# 2. Lancer la simulation avec ce rail
ros2 launch franka_sonde gazebo_complet.launch.py rail_position:=<valeur>

# 3. Commander le bras (terminal séparé)
ros2 run controleurs pose_gui
```

Le GUI se remplit automatiquement avec la pose TCP courante. Les poses sont en frame **`fr3_link0`**.

## Contrainte rail — pourquoi le joint est fixe

Le plugin Franka Gazebo (`franka_ign_ros2_control/IgnitionSystem`) construit sa chaîne KDL depuis `world` jusqu'au tip de **l'URDF entier**, puis calcule la compensation gravitationnelle sur exactement 7 joints (hardcodé). Un joint prismatique dans ce chemin → 8 joints → crash → tous les controllers inactifs.

```cpp
// ign_ros2_control_plugin.cpp
root_link = model.getRoot()->name;  // "world"
tip_link  = findTipLink(model);     // "sonde_tcp"
kdl_model_ = ModelKDL(model, root_link, tip_link);
// prismatique dans ce chemin → JntToGravity plante
```

Le pattern standard (deux blocs `<ros2_control>`) ne contourne pas ce problème car le plugin lit le modèle complet indépendamment des blocs déclarés.

**Solution actuelle** : `rail_joint` reste `fixed`, position baked dans l'URDF au lancement via xacro arg.

## Couplage rail dynamique (branche `feature/rail-dynamique`)

### Approche retenue

Découpler l'ancrage du bras du rail dans l'URDF : le bras est fixé directement à `world` avec un offset Y variable. Le rail existe comme entité visuelle séparée avec son propre bloc `<ros2_control>` + `gz_ros2_control/GazeboSystem`. Ça fonctionne identiquement en simulation et sur le vrai hardware.

### Étapes

1. **URDF** — Séparer ancrage bras (joint `world → fr3_link0`, offset Y variable) du rail visuel
2. **`rail_mover` node** — Souscrit `/target_rail_pos`, met à jour l'offset, republie `robot_description`
3. **Launch** — Intégrer `rail_mover` dans `gazebo_complet.launch.py`
4. **`cartesian_commander`** — Calculer rail optimal → attendre repositionnement → IK + trajectoire bras

## Carte de capacité

Strates Z ∈ [0.0, 0.3 m] en frame `fr3_link0`, précalculées par `Capacity_Map_Creator.py`.

```bash
cd src/carte
python3 Voxel_visualisation.py    # visualiser
python3 optimal_rail_finder.py    # trouver le rail optimal
```

## Repères

```
world → table → rail → rail_joint (fixed, y = -0.85 + rail_position)
    → montage → fr3_link0 (z=+1.04) → [7 joints] → fr3_link8
    → sonde_base (flip 180° X) → sonde_tcp (+0.2 m Z)
```

Espace de travail (frame `fr3_link0`) : x [0.1–0.8], y [−0.7–0.7], z [0.0–0.3] m

## Dépannage

- **Controllers inactifs** : `rail_joint` pas `fixed` dans l'URDF
- **IK code −5** : pose hors espace de travail en frame `fr3_link0`
- **`fr3_arm_controller` non chargé** : rebuilder `franka_sonde` et re-sourcer
- **libfranka ne compile pas** : `cd src/libfranka && git submodule update --init --recursive`
