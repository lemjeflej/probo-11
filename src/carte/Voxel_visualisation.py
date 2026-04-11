import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ===========================
# Chargement des strates
# ===========================
# Charge les fichiers .pkl générés par Capacity_Map_Creator.py.
# Chaque fichier contient une liste de voxels avec :
#   "voxel"    : [x, y, z] dans le frame fr3_link0
#   "acc"      : accessibilité (% d'orientations IK réussies)
#   "manip_avg": manipulabilité moyenne (produit valeurs singulières de J)
# Adapter Z_VALUES selon les strates effectivement calculées.
Z_VALUES = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3]

all_data = []

for z in Z_VALUES:
    filename = f"results/voxels_Z_{z:.2f}.pkl"
    with open(filename, "rb") as f:
        data = pickle.load(f)
        all_data.extend(data)
    print(f"Strate Z={z:.2f} chargée ({len(data)} voxels)")

print(f"\nTotal voxels chargés : {len(all_data)}")

# ===========================
# Extraction et filtrage
# ===========================
# Filtre Y ∈ [-1, 1] : correspond à la course du rail (0 → 1.7 m)
# remappée autour de 0 pour centrer la visualisation.
X = np.array([v["voxel"][0] for v in all_data])
Y = np.array([v["voxel"][1] for v in all_data])
Z = np.array([v["voxel"][2] for v in all_data])
Acc   = np.array([v["acc"]      for v in all_data])
Manip = np.array([v["manip_avg"] for v in all_data])

mask_y = (Y >= -1.0) & (Y <= 1.0)
X, Y, Z, Acc, Manip = X[mask_y], Y[mask_y], Z[mask_y], Acc[mask_y], Manip[mask_y]
print(f"Voxels après filtre Y ∈ [-1,1] : {len(X)}")

# ===========================
# Carte d'accessibilité 3D
# ===========================
# Couleur = % d'orientations atteignables pour chaque position du TCP.
# Un voxel rouge/sombre est difficile d'accès, vert/clair est très accessible.
fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")
sc = ax.scatter(X, Y, Z, c=Acc, cmap="viridis", s=8)
fig.colorbar(sc, ax=ax, label="Accessibilité (%)")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
ax.set_title("Accessibilité 3D (Y ∈ [-1,1])")
plt.show()

# ===========================
# Carte de manipulabilité 3D
# ===========================
# Couleur = manipulabilité moyenne (produit des valeurs singulières du Jacobien).
# Haute manipulabilité → loin des singularités → mouvements fluides et précis.
# C'est cette carte qui guide le choix de la position rail optimale.
fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")
sc = ax.scatter(X, Y, Z, c=Manip, cmap="plasma", s=8)
fig.colorbar(sc, ax=ax, label="Manipulabilité moyenne")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
ax.set_title("Manipulabilité 3D (Y ∈ [-1,1])")
plt.show()

# ===========================
# Workspace utile 3D
# ===========================
# Filtre : ne garde que les voxels avec accessibilité > ACC_THRESH.
# Permet de visualiser la forme réelle de l'espace de travail exploitable,
# indépendamment de la qualité du mouvement.
ACC_THRESH = 0
mask_ws = Acc > ACC_THRESH

fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")
ax.scatter(X[mask_ws], Y[mask_ws], Z[mask_ws], s=6)
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
ax.set_title(f"Workspace utile 3D\nAcc ≥ {ACC_THRESH:.0f} %")
plt.show()
