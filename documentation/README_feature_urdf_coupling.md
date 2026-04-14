# Branche `feature/urdf-coupling` — Rapport d'investigation

## Objectif

Faire bouger visuellement le robot FR3 (`probo11`) avec le rail dans Gazebo,
en résolvant la contrainte qui rendait le modèle statique sur la branche précédente.

---

## Cause racine identifiée

Dans la branche `feature/integration-rail`, le lien racine de `srr.xacro` s'appelait
`world`. Dans Ignition Gazebo (Fortress), un modèle dont le lien racine porte ce nom
est automatiquement converti en **modèle statique** (`static=true` dans le SDF généré).
Résultat : `ign service set_pose` retournait `data: true` mais ne déplaçait rien —
le modèle était soudé au référentiel monde.

Le curseur (`rail_curseur`) fonctionnait car son lien racine s'appelle `base`.

---

## Ce qui a été tenté

### 1. Renommage `world` → `robot_base` dans `srr.xacro`

**Résultat :** le robot suit le rail. Le `set_pose` fonctionne.

**Problème introduit :** le modèle n'est plus statique → la gravité s'applique
à `robot_base` → le robot tombe entre deux appels `set_pose`.

### 2. Tags URDF `<kinematic>1</kinematic>` + `<gravity>0</gravity>` sur `robot_base`

**Résultat :** non fonctionnel. Le convertisseur URDF→SDF d'Ignition Fortress
n'honore pas ces tags de façon fiable pour le lien racine d'un modèle libre.
Le robot continue de tomber.

### 3. Remplacement du `rail_mover.py` (subprocess) par `rail_mover.cpp` (ignition transport)

**Objectif :** appeler `set_pose` via `ignition::transport::Node::Request()` directement,
sans spawner un nouveau processus à chaque appel.

| | Python subprocess | C++ ignition transport |
|---|---|---|
| Latence par appel | ~150 ms | < 5 ms |
| Fréquence effective | ~5 Hz | 100 Hz |
| Chute gravité entre appels | ~11 cm | ~0.5 mm |

**Résultat :** mouvement du rail visuellement fluide. Le robot suit le curseur
sans chute perceptible pendant le déplacement.

**Problème résiduel :** vibrations (±2 cm) à l'arrêt et pendant le mouvement du bras.

### 4. Analyse des vibrations

Les vibrations sont causées par la **collision entre deux systèmes de contrôle** :

- `ros2_control` (1000 Hz) commande les joints en position dans le référentiel monde.
- `set_pose` (100 Hz) téléporte `robot_base` → déplace la base sous les joints.
- `ros2_control` perçoit une erreur de position → applique des couples correctors → oscille.
- Pendant la trajectoire du bras, l'interférence est encore plus marquée
  (trajectoire planifiée pour une base fixe, base qui bouge → résonance).

Ce comportement est une **limitation fondamentale** d'Ignition Gazebo Fortress :
`set_pose` est conçu pour la téléportation one-shot, pas pour le maintien continu
d'une position contre la physique d'un modèle à controllers actifs.
La seule solution propre serait un **plugin Gazebo natif** opérant dans la boucle
physique (1000 Hz), hors scope du projet.

---

## État final de la branche

| Composant | Changement | État |
|---|---|---|
| `srr.xacro` | Lien racine `robot_base` + tags kinematic/gravity | Inclus (investigation) |
| `rail_mover.cpp` | Nouveau nœud C++ ign-transport, 100 Hz | **Fonctionnel** |
| `rail_mover.py` | Interpolation latest-value queue | Supersédé par .cpp |
| `CMakeLists.txt` | Build `rail_mover.cpp` + deps ign-transport11/msgs8 | **Fonctionnel** |
| `mission_coordinator.py` | Ne ré-optimise pas le rail si Δpos < 3 cm | **Fonctionnel** |

---

## Améliorations conservées

### `mission_coordinator.py` — Stabilité orientation

Correction d'un bug : changer seulement le yaw dans le GUI déclenchait une
ré-optimisation du rail (même position XYZ → rail différent → robot change
de configuration de l'autre côté de la table).

**Fix :** si `‖p_nouvelle − p_précédente‖ < 3 cm`, le rail n'est pas ré-optimisé
et la position courante est conservée.

### `rail_mover.cpp` — Mouvement fluide du curseur

Le curseur rouge (`rail_curseur`) se déplace désormais avec une interpolation
linéaire à vitesse constante (défaut 0.35 m/s), appelée à 100 Hz via
ignition transport direct. Plus aucun saut visible.

---

## Conclusion et recommandation

Sur le **matériel réel**, le robot se déplace physiquement sur le rail — cette
contrainte de simulation est sans impact en production.

Pour la démonstration en simulation, la posture adoptée sur la branche suivante :

- Lien racine `world` → modèle statique, aucune chute, aucune vibration
- Curseur rouge déplacé par `rail_mover.cpp` → indicateur visuel de la position rail
- IK calculée pour la bonne position rail → résultat algorithmique correct
- Argument jury : limitation documentée et comprise d'Ignition Fortress + ros2_control

---

## Commandes de test

```bash
# Build
colcon build --packages-select franka_sonde controleurs

# Lancement
ros2 launch franka_sonde gazebo_complet.launch.py

# Test set_pose direct (curseur fonctionne, probo11 statique)
ign service -s /world/empty/set_pose \
  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
  --timeout 3000 \
  --req 'name: "rail_curseur" position: {x:0, y:-0.3, z:1.03}'
```
