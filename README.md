# Workspace ROS2 - Projet Franka FR3

Ce workspace contient les packages pour le projet de robotique avec le bras Franka FR3.

## Structure du Projet

```
probo-11/
├── franka.repos              # Dépendances principales (franka_ros2)
├── src/
│   ├── controleurs/          # Nos contrôleurs personnalisés
│   ├── franka_sonde/         # Configuration et lancement du projet
│   ├── franka_ros2/          # Packages officiels Franka (géré par vcs)
│   ├── franka_description/   # Descriptions URDF Franka (géré par vcs)
│   └── libfranka/            # Bibliothèque C++ Franka (géré par vcs)
```

## Prérequis

- Ubuntu 22.04
- ROS2 Humble
- Git installé

## Installation du Workspace

### 1. Cloner le dépôt

```bash
cd ~/robotics
git clone <https://github.com/lemjeflej/probo-11/tree/master> probo-11
cd probo-11
```

### 2. Installer vcstool

Si ce n'est pas déjà fait :

```bash
sudo apt update
sudo apt install python3-vcstool
```

### 3. Récupérer les dépendances Franka

Importer les packages principaux :

```bash
vcs import src < franka.repos
```

Importer les dépendances additionnelles (libfranka et franka_description) :

```bash
vcs import src < src/franka_ros2/dependency.repos
```

### 4. Initialiser les sous-modules de libfranka

**IMPORTANT** : libfranka nécessite ses sous-modules pour compiler correctement :

```bash
cd src/libfranka
git submodule update --init --recursive
cd ../..
```

### 5. Installer les dépendances système

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Compiler le workspace

```bash
colcon build --symlink-install
```

La compilation prend environ 7 minutes. Certains packages afficheront des warnings (stderr), c'est normal.

### 7. Sourcer le workspace

```bash
source install/setup.bash
```

Pour sourcer automatiquement à chaque nouveau terminal :

```bash
echo "source ~/robotics/probo-11/install/setup.bash" >> ~/.bashrc
```

## Vérification de l'Installation

Vérifier que les packages Franka sont disponibles :

```bash
ros2 pkg list | grep franka
```

Vous devriez voir une liste de packages `franka_*`.

## Utilisation

### Lancer la simulation

```bash
ros2 launch franka_sonde project.launch.py
```

### Lancer RViz avec la configuration du rail

```bash
ros2 launch franka_sonde rail-sur_table.launch.py
```

## Mise à Jour des Packages Franka

Si vous souhaitez mettre à jour les packages Franka vers une version plus récente :

```bash
cd ~/robotics/probo-11
vcs pull src
colcon build --symlink-install
```

## Structure Git

Ce dépôt utilise la méthode standard ROS2 pour gérer les dépendances externes :

- **Nos packages** (`controleurs/`, `franka_sonde/`) sont versionnés dans ce dépôt
- **Les packages Franka** sont ignorés par git (voir `.gitignore`) et récupérés via les fichiers `.repos`

Cette approche évite les conflits de sous-modules git et permet une gestion propre des dépendances.

## Dépannage

### Erreur : "No such file or directory: franka/async_control/..."

Cela signifie que libfranka n'a pas été correctement installé. Vérifiez que :
1. Vous avez bien exécuté `vcs import src < src/franka_ros2/dependency.repos`
2. Vous avez initialisé les sous-modules : `cd src/libfranka && git submodule update --init --recursive`

### Erreur : "add_subdirectory: The source directory .../common does not contain a CMakeLists.txt"

Les sous-modules de libfranka ne sont pas initialisés. Voir la solution ci-dessus.

### Warnings pendant la compilation

Les warnings (stderr output) des packages `franka_*` et `libfranka` sont normaux et n'empêchent pas le fonctionnement.

## Contact

Pour toute question sur ce workspace, contactez Hamza.
