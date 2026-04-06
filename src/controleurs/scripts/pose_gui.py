#!/usr/bin/env python3
"""
pose_gui.py
===========
Interface graphique pour envoyer des poses cartésiennes au robot FR3.

Entrées utilisateur :
  - Position   : x, y, z  (mètres)
  - Orientation: roll, pitch, yaw  (degrés, convertis en quaternion)

Topics ROS 2 :
  PUB  /target_pose                  geometry_msgs/PoseStamped
  SUB  /cartesian_commander/busy     std_msgs/Bool
       → true  : bouton désactivé (mouvement en cours)
       → false : bouton activé (prêt pour nouvelle commande)

Usage :
  ros2 run controleurs pose_gui
"""

import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


# ──────────────────────────────────────────────────────────
# Conversion Euler (degrés) → Quaternion
# Convention : extrinsic RPY (roll autour X, pitch autour Y, yaw autour Z)
# ──────────────────────────────────────────────────────────
def euler_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


# ──────────────────────────────────────────────────────────
# Nœud ROS 2
# ──────────────────────────────────────────────────────────
class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_gui_node')

        self.pub = self.create_publisher(PoseStamped, '/target_pose', 5)

        self.busy = False
        self.busy_callbacks = []  # fonctions à appeler quand busy change

        self.create_subscription(
            Bool,
            '/cartesian_commander/busy',
            self._on_busy,
            10
        )

    def _on_busy(self, msg: Bool):
        self.busy = msg.data
        for cb in self.busy_callbacks:
            cb(self.busy)

    def send_pose(self, x, y, z, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)
        self.get_logger().info(
            f'Pose envoyée : pos=({x:.3f}, {y:.3f}, {z:.3f})  '
            f'quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})'
        )


# ──────────────────────────────────────────────────────────
# Interface graphique
# ──────────────────────────────────────────────────────────
class PoseGUI:
    def __init__(self, node: PosePublisherNode):
        self.node = node

        self.root = tk.Tk()
        self.root.title('PROBO-11 — Commande cartésienne FR3')
        self.root.resizable(False, False)

        # ── Style ──────────────────────────────────────────
        style = ttk.Style()
        style.configure('TLabel',  font=('Helvetica', 11))
        style.configure('TEntry',  font=('Helvetica', 11))
        style.configure('Header.TLabel', font=('Helvetica', 12, 'bold'))
        style.configure('Send.TButton', font=('Helvetica', 12, 'bold'),
                        padding=10)
        style.configure('Status.TLabel', font=('Helvetica', 10, 'italic'))

        pad = {'padx': 10, 'pady': 5}

        # ── Titre ──────────────────────────────────────────
        ttk.Label(self.root, text='Pose cible du TCP (sonde)',
                  style='Header.TLabel').grid(
            row=0, column=0, columnspan=3, pady=(14, 4))

        # ── Position ───────────────────────────────────────
        ttk.Label(self.root, text='Position (m)',
                  style='Header.TLabel').grid(
            row=1, column=0, columnspan=3, sticky='w', padx=10)

        labels_pos = ['x', 'y', 'z']
        defaults_pos = ['0.5', '0.0', '1.5']
        self.pos_vars = []
        for i, (lbl, dflt) in enumerate(zip(labels_pos, defaults_pos)):
            ttk.Label(self.root, text=lbl + ' :').grid(
                row=2 + i, column=0, sticky='e', **pad)
            var = tk.StringVar(value=dflt)
            self.pos_vars.append(var)
            entry = ttk.Entry(self.root, textvariable=var, width=10)
            entry.grid(row=2 + i, column=1, sticky='w', **pad)
            ttk.Label(self.root, text='m').grid(
                row=2 + i, column=2, sticky='w')

        # ── Orientation ────────────────────────────────────
        ttk.Label(self.root, text='Orientation (degrés — RPY extrinsic)',
                  style='Header.TLabel').grid(
            row=5, column=0, columnspan=3, sticky='w', padx=10, pady=(12, 0))

        labels_ori = ['roll (X)', 'pitch (Y)', 'yaw (Z)']
        defaults_ori = ['180.0', '0.0', '0.0']
        self.ori_vars = []
        for i, (lbl, dflt) in enumerate(zip(labels_ori, defaults_ori)):
            ttk.Label(self.root, text=lbl + ' :').grid(
                row=6 + i, column=0, sticky='e', **pad)
            var = tk.StringVar(value=dflt)
            self.ori_vars.append(var)
            entry = ttk.Entry(self.root, textvariable=var, width=10)
            entry.grid(row=6 + i, column=1, sticky='w', **pad)
            ttk.Label(self.root, text='°').grid(
                row=6 + i, column=2, sticky='w')

        # ── Affichage quaternion (lecture seule) ───────────
        ttk.Label(self.root, text='Quaternion résultant',
                  style='Header.TLabel').grid(
            row=9, column=0, columnspan=3, sticky='w', padx=10, pady=(12, 0))

        self.quat_var = tk.StringVar(value='—')
        ttk.Label(self.root, textvariable=self.quat_var,
                  style='Status.TLabel').grid(
            row=10, column=0, columnspan=3, padx=10, pady=2)

        # Mettre à jour le quaternion affiché quand les angles changent
        for var in self.ori_vars:
            var.trace_add('write', lambda *_: self._update_quat_display())
        self._update_quat_display()

        # ── Durée du mouvement ─────────────────────────────
        ttk.Label(self.root, text='Durée mouvement :').grid(
            row=11, column=0, sticky='e', **pad)
        self.duration_var = tk.StringVar(value='5.0')
        ttk.Entry(self.root, textvariable=self.duration_var, width=10).grid(
            row=11, column=1, sticky='w', **pad)
        ttk.Label(self.root, text='s').grid(row=11, column=2, sticky='w')

        # ── Bouton Envoyer ─────────────────────────────────
        self.send_btn = ttk.Button(
            self.root, text='Envoyer la pose',
            style='Send.TButton',
            command=self._on_send
        )
        self.send_btn.grid(row=12, column=0, columnspan=3,
                           padx=10, pady=(14, 6), sticky='ew')

        # ── Barre de statut ────────────────────────────────
        self.status_var = tk.StringVar(value='En attente de cartesian_commander...')
        status_lbl = ttk.Label(self.root, textvariable=self.status_var,
                               style='Status.TLabel',
                               relief='sunken', anchor='w')
        status_lbl.grid(row=13, column=0, columnspan=3,
                        sticky='ew', padx=0, pady=(0, 0))
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)

        # ── Abonnement aux changements busy ────────────────
        self.node.busy_callbacks.append(self._on_busy_changed)

        # ── Fermeture propre ───────────────────────────────
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    # ── Helpers ────────────────────────────────────────────

    def _update_quat_display(self):
        try:
            r = float(self.ori_vars[0].get())
            p = float(self.ori_vars[1].get())
            y = float(self.ori_vars[2].get())
            qx, qy, qz, qw = euler_deg_to_quaternion(r, p, y)
            self.quat_var.set(
                f'x={qx:.4f}  y={qy:.4f}  z={qz:.4f}  w={qw:.4f}')
        except ValueError:
            self.quat_var.set('(valeurs invalides)')

    def _on_busy_changed(self, busy: bool):
        # Appelé depuis le thread ROS — planifier la mise à jour tkinter
        self.root.after(0, self._apply_busy, busy)

    def _apply_busy(self, busy: bool):
        if busy:
            self.send_btn.state(['disabled'])
            self.status_var.set('Mouvement en cours... ⏳')
        else:
            self.send_btn.state(['!disabled'])
            self.status_var.set('Prêt — entrez une nouvelle pose.')

    def _on_send(self):
        try:
            x = float(self.pos_vars[0].get())
            y = float(self.pos_vars[1].get())
            z = float(self.pos_vars[2].get())
            roll  = float(self.ori_vars[0].get())
            pitch = float(self.ori_vars[1].get())
            yaw   = float(self.ori_vars[2].get())
        except ValueError:
            messagebox.showerror('Erreur', 'Toutes les valeurs doivent être des nombres.')
            return

        qx, qy, qz, qw = euler_deg_to_quaternion(roll, pitch, yaw)
        self.node.send_pose(x, y, z, qx, qy, qz, qw)

        # Désactiver le bouton immédiatement (avant que busy arrive)
        self.send_btn.state(['disabled'])
        self.status_var.set(
            f'Commande envoyée : ({x:.3f}, {y:.3f}, {z:.3f})'
            f'  RPY=({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)'
        )

    def _on_close(self):
        self.root.destroy()
        rclpy.shutdown()

    def run(self):
        self.root.mainloop()


# ──────────────────────────────────────────────────────────
# Point d'entrée
# ──────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = PosePublisherNode()

    # ROS 2 spin dans un thread séparé (tkinter doit tourner dans le thread principal)
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui = PoseGUI(node)
    gui.run()

    node.destroy_node()


if __name__ == '__main__':
    main()
