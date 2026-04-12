"""
optimal_rail_finder.py
======================
Détermine la position rail optimale pour atteindre une cible donnée
en maximisant la manipulabilité du FR3.

Principe
--------
La carte de capacité est calculée en frame fr3_link0.
Pour chaque position rail candidate, on convertit la cible (frame world)
en frame fr3_link0, on trouve le voxel le plus proche dans la carte,
et on lit sa manipulabilité. Le rail_position qui donne la meilleure
manipulabilité est retenu.

Conversion world → fr3_link0
-----------------------------
fr3_link0 est à (0, -0.85 + rail_position, 1.04) dans world.
Donc :
  x_robot = x_world
  y_robot = y_world - (-0.85 + rail_position) = y_world + 0.85 - rail_position
  z_robot = z_world - 1.04

Usage
-----
  python3 optimal_rail_finder.py
  → modifier TARGET_WORLD en bas du fichier selon la cible souhaitée

Sortie
------
  - Tableau des manipulabilités par position rail
  - Position rail optimale + commande de lancement Gazebo
  - Graphe manipulabilité vs position rail
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt
import os

# ===========================
# Chargement de la carte
# ===========================
# Charge toutes les strates pkl et construit deux tableaux numpy :
#   voxel_positions : (N, 3) coordonnées en frame fr3_link0
#   manip_values    : (N,)   manipulabilité moyenne par voxel
RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")
Z_VALUES = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30]

all_data = []
for z in Z_VALUES:
    path = os.path.join(RESULTS_DIR, f"voxels_Z_{z:.2f}.pkl")
    with open(path, "rb") as f:
        all_data.extend(pickle.load(f))

voxel_positions = np.array([v["voxel"]    for v in all_data])  # (N, 3)
manip_values    = np.array([v["manip_avg"] for v in all_data])  # (N,)
acc_values      = np.array([v["acc"]       for v in all_data])  # (N,)

print(f"Carte chargée : {len(all_data)} voxels")


# ===========================
# Conversion world → fr3_link0
# ===========================
# Offsets fixes issus de l'URDF :
#   rail_joint origin : y = -0.85 + rail_position
#   montage_to_arm    : z = +1.04
Y_OFFSET_BASE = -0.85   # m  (position rail = 0)
Z_OFFSET      =  1.04   # m

def world_to_robot(target_world, rail_position):
    """Convertit une cible (frame world) en frame fr3_link0."""
    x_w, y_w, z_w = target_world
    x_r = x_w
    y_r = y_w - (Y_OFFSET_BASE + rail_position)
    z_r = z_w - Z_OFFSET
    return np.array([x_r, y_r, z_r])


# ===========================
# Recherche du voxel le plus proche
# ===========================
# Distance euclidienne entre la cible (en frame fr3_link0)
# et tous les voxels de la carte. On retourne la manipulabilité
# du voxel le plus proche.
def query_map(target_robot):
    """Retourne (manipulabilité, accessibilité) du voxel le plus proche."""
    dists = np.linalg.norm(voxel_positions - target_robot, axis=1)
    idx = np.argmin(dists)
    return manip_values[idx], acc_values[idx], dists[idx]


# ===========================
# Recherche du rail optimal
# ===========================
# Balayage de la course complète du rail par pas de RAIL_STEP.
# Pour chaque position, on évalue la manipulabilité au voxel
# le plus proche de la cible convertie en frame robot.
RAIL_MIN  = 0.0    # m
RAIL_MAX  = 1.7    # m
RAIL_STEP = 0.05   # m  (même résolution que la carte en Y)

def find_optimal_rail(target_world, verbose=True):
    """
    Retourne la position rail optimale (manipulabilité max)
    pour atteindre target_world = (x, y, z) en frame world.
    """
    rail_candidates = np.arange(RAIL_MIN, RAIL_MAX + RAIL_STEP, RAIL_STEP)

    results = []
    for rail_pos in rail_candidates:
        target_robot = world_to_robot(target_world, rail_pos)
        manip, acc, dist = query_map(target_robot)
        results.append({
            "rail_pos": rail_pos,
            "manip":    manip,
            "acc":      acc,
            "dist":     dist,
            "target_robot": target_robot,
        })

    # Tri par manipulabilité décroissante
    results.sort(key=lambda r: r["manip"], reverse=True)
    best = results[0]

    if verbose:
        print(f"\nCible (world)  : x={target_world[0]:.3f}  y={target_world[1]:.3f}  z={target_world[2]:.3f}")
        print(f"\n{'Rail (m)':>10} {'Cible robot (x,y,z)':>30} {'Manip':>12} {'Acc (%)':>10} {'Dist voxel':>12}")
        print("-" * 80)
        for r in sorted(results, key=lambda r: r["rail_pos"]):
            tr = r["target_robot"]
            print(
                f"{r['rail_pos']:>10.2f}"
                f"  ({tr[0]:+.3f}, {tr[1]:+.3f}, {tr[2]:+.3f})"
                f"{r['manip']:>12.4f}"
                f"{r['acc']:>10.1f}"
                f"{r['dist']:>12.4f}"
            )
        print()
        print(f"→ Rail optimal : {best['rail_pos']:.2f} m  "
              f"(manip={best['manip']:.4f}, acc={best['acc']:.1f}%)")
        print()
        print("Commande Gazebo :")
        print(f"  ros2 launch franka_sonde gazebo_complet.launch.py "
              f"rail_position:={best['rail_pos']:.2f}")

    return best, results


# ===========================
# Visualisation
# ===========================
def plot_results(results, target_world):
    rail_pos = [r["rail_pos"] for r in sorted(results, key=lambda r: r["rail_pos"])]
    manip    = [r["manip"]    for r in sorted(results, key=lambda r: r["rail_pos"])]
    acc      = [r["acc"]      for r in sorted(results, key=lambda r: r["rail_pos"])]

    best_rail = max(results, key=lambda r: r["manip"])["rail_pos"]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax1.plot(rail_pos, manip, "b-o", markersize=4)
    ax1.axvline(best_rail, color="red", linestyle="--", label=f"Optimal : {best_rail:.2f} m")
    ax1.set_ylabel("Manipulabilité")
    ax1.set_title(f"Sélection rail — cible world ({target_world[0]:.2f}, {target_world[1]:.2f}, {target_world[2]:.2f})")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(rail_pos, acc, "g-o", markersize=4)
    ax2.axvline(best_rail, color="red", linestyle="--")
    ax2.set_ylabel("Accessibilité (%)")
    ax2.set_xlabel("Position rail (m)")
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


# ===========================
# Point d'entrée
# ===========================
# Modifier TARGET_WORLD selon la position cible en frame world.
# Format : (x, y, z) en mètres.
# Exemple : face au robot au milieu du rail, à hauteur d'abdomen.
if __name__ == "__main__":
    TARGET_WORLD = (0.3069, 0.0, 1.8303)   # (x, y, z) en frame world

    best, results = find_optimal_rail(TARGET_WORLD, verbose=True)
    plot_results(results, TARGET_WORLD)
