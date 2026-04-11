import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ===========================
# Strates Z à charger
# ===========================
Z_VALUES = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3]

# ===========================
# Chargement des strates
# ===========================
all_data = []

for z in Z_VALUES:
    filename = f"results/voxels_Z_{z:.2f}.pkl"
    with open(filename, "rb") as f:
        data = pickle.load(f)
        all_data.extend(data)
    print(f"Strate Z={z:.2f} chargée ({len(data)} voxels)")

print(f"\nTotal voxels chargés : {len(all_data)}")

# ===========================
# Extraction des données
# ===========================
X = np.array([v["voxel"][0] for v in all_data])
Y = np.array([v["voxel"][1] for v in all_data])
Z = np.array([v["voxel"][2] for v in all_data])

Acc = np.array([v["acc"] for v in all_data])
Manip = np.array([v["manip_avg"] for v in all_data])

# ===========================
# Filtre Y ∈ [-1, 1]
# ===========================
mask_y = (Y >= -1.0) & (Y <= 1.0)

X = X[mask_y]
Y = Y[mask_y]
Z = Z[mask_y]
Acc = Acc[mask_y]
Manip = Manip[mask_y]

print(f"Voxels après filtre Y ∈ [-1,1] : {len(X)}")

# ===========================
# Accessibilité 3D
# ===========================
fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")

sc = ax.scatter(X, Y, Z, c=Acc, cmap="viridis", s=8)
fig.colorbar(sc, ax=ax, label="Accessibilité (%)")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Accessibilité 3D (Y ∈ [-1,1])")

plt.show()

# ===========================
# Manipulabilité 3D
# ===========================
fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")

sc = ax.scatter(X, Y, Z, c=Manip, cmap="plasma", s=8)
fig.colorbar(sc, ax=ax, label="Manipulabilité moyenne")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Manipulabilité 3D (Y ∈ [-1,1])")

plt.show()

# ===========================
# Workspace utile 3D
# ===========================
ACC_THRESH = 0
#MANIP_THRESH = 1e-2

mask_ws = (Acc > ACC_THRESH) #& (Manip >= MANIP_THRESH)

fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")

ax.scatter(
    X[mask_ws],
    Y[mask_ws],
    Z[mask_ws],
    s=6
)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title(
    f"Workspace utile 3D\n"
    f"Acc ≥ {ACC_THRESH:.0f} %"
)

plt.show()
