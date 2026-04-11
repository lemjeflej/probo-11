import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import os
import pickle

# ===========================
# Paramètres Panda
# ===========================
a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088]
alpha = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2]
d = [0.333, 0, 0.316, 0, 0.384, 0, 0]

links = [RevoluteDH(d=d[i], a=a[i], alpha=alpha[i]) for i in range(7)]
panda = DHRobot(links, name="Panda")

# Outil prolongé sur Z
tool_length = 0.20
panda.tool = SE3(0, 0, tool_length)

reach_max = panda.reach + tool_length

# ===========================
# Roll propre
# ===========================
gamma_angles = np.deg2rad([0, 90, 180, 270])

# ===========================
# Spiral de Fibonacci
# ===========================
def fibonacci_sphere(n_points):
    points = []
    golden_ratio = (1 + np.sqrt(5)) / 2
    for i in range(n_points):
        z = 1 - 2 * i / (n_points - 1)
        theta = 2 * np.pi * i / golden_ratio
        r = np.sqrt(max(0.0, 1 - z**2))
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        points.append(np.array([x, y, z]))
    return points

def rotation_from_z(direction):
    z = direction / np.linalg.norm(direction)
    ref = np.array([1, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1, 0])
    y = np.cross(z, ref)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack((x, y, z))

def euler_from_R(R):
    ay = np.arcsin(R[0, 2])
    ax = np.arctan2(-R[1, 2], R[2, 2])
    az = np.arctan2(-R[0, 1], R[0, 0])
    return ax, ay, az

# Génération orientations
N_ORIENT = 200
directions = fibonacci_sphere(N_ORIENT)
euler_angles = [euler_from_R(rotation_from_z(d)) for d in directions]

# Ordonnancement pour warm start
euler_angles.sort(key=lambda a: abs(a[0]) + abs(a[1]) + abs(a[2]))

# ===========================
# Grille de voxels
# ===========================
X_vals = np.arange(0, 0.4+0.1, 0.1)
Y_vals = np.arange(-1, 1+0.05, 0.05)
Z_vals = np.array([0.05, 0.15, 0.25])

# Créer dossier results si inexistant
os.makedirs("results", exist_ok=True)

# ===========================
# Boucle sur Z
# ===========================
for z in Z_vals:
    print(f"\n=== Z = {z:.2f} ===")
    voxels_data = []  # stocke tous les voxels pour cette strate Z

    for x in X_vals:
        for y in Y_vals:
            voxel = np.array([x, y, z])

            # Vérification reach
            if np.linalg.norm(voxel) > reach_max:
                voxels_data.append({
                    "voxel": voxel,
                    "acc": 0.0,
                    "manip_avg": 0.0
                })
                continue

            # IK + manipulabilité
            reachable = 0
            manips = []
            p_target = SE3(*voxel)
            q_prev = np.zeros(7)

            for ax, ay, az in euler_angles:
                R_base = SE3.Rx(ax) * SE3.Ry(ay) * SE3.Rz(az)

                for gamma in gamma_angles:
                    T_target = p_target * R_base * SE3.Rz(gamma)

                    sol = panda.ikine_LM(
                        T_target,
                        q0=q_prev,
                        mask=[1, 1, 1, 1, 1, 1]
                    )

                    if sol.success:
                        q_prev = sol.q
                        reachable += 1
                        J = panda.jacob0(sol.q)
                        s = np.linalg.svd(J, compute_uv=False)
                        manips.append(np.prod(s))

            total = len(euler_angles) * len(gamma_angles)
            acc = 100 * reachable / total if total > 0 else 0.0
            manip_avg = np.mean(manips) if manips else 0.0

            voxels_data.append({
                "voxel": voxel,
                "acc": acc,
                "manip_avg": manip_avg
            })

    # Sauvegarde .pkl pour chaque strate Z
    filename = f"results/voxels_Z_{z:.2f}.pkl"
    with open(filename, "wb") as f:
        pickle.dump(voxels_data, f)

    print(f"Strate Z={z:.2f} sauvegardée → {filename}")
