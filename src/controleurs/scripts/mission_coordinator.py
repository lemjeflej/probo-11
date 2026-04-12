#!/usr/bin/env python3
"""
mission_coordinator.py
======================
Orchestre la séquence complète pour atteindre une cible anatomique :
  1. Reçoit la pose cible en frame world depuis le GUI
  2. Calcule la position rail optimale (carte de capacité)
  3. Envoie la commande rail → attend le déplacement
  4. Convertit world → fr3_link0 avec le nouveau rail_position
  5. Envoie la pose au cartesian_commander

Topics
------
  SUB  /target_pose              geometry_msgs/PoseStamped  (frame world, GUI)
  SUB  /rail_mover/done          std_msgs/Bool
  SUB  /current_rail_position    std_msgs/Float64
  SUB  /cartesian_commander/busy std_msgs/Bool
  PUB  /target_rail_pos          std_msgs/Float64
  PUB  /target_pose_robot        geometry_msgs/PoseStamped  (frame fr3_link0)
  PUB  /commander/busy           std_msgs/Bool              (GUI busy state)

Paramètres ROS 2
----------------
  carte_results_dir  (string)  Chemin vers src/carte/results/
"""

import os
import pickle
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64


# ─── Carte de capacité ──────────────────────────────────────────────────────

Z_VALUES   = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
Y_BASE     = -0.85   # y du robot quand rail_position = 0
Z_OFFSET   =  1.04   # z du robot dans world
RAIL_MIN   =  0.0
RAIL_MAX   =  1.7
RAIL_STEP  =  0.05


def load_map(results_dir: str):
    all_data = []
    for z in Z_VALUES:
        path = os.path.join(results_dir, f'voxels_Z_{z:.2f}.pkl')
        with open(path, 'rb') as f:
            all_data.extend(pickle.load(f))
    positions = np.array([v['voxel']     for v in all_data])
    manips    = np.array([v['manip_avg'] for v in all_data])
    return positions, manips


def world_to_robot(target_world, rail_position):
    x_w, y_w, z_w = target_world
    return np.array([x_w,
                     y_w - (Y_BASE + rail_position),
                     z_w - Z_OFFSET])


def find_optimal_rail(target_world, voxel_positions, manip_values):
    """Retourne la position rail (m) qui maximise la manipulabilité."""
    candidates = np.arange(RAIL_MIN, RAIL_MAX + RAIL_STEP, RAIL_STEP)
    best_pos, best_manip = 0.0, -1.0
    for rail_pos in candidates:
        target_robot = world_to_robot(target_world, rail_pos)
        dists = np.linalg.norm(voxel_positions - target_robot, axis=1)
        idx   = np.argmin(dists)
        if manip_values[idx] > best_manip:
            best_manip = manip_values[idx]
            best_pos   = rail_pos
    return best_pos, best_manip


# ─── Nœud ───────────────────────────────────────────────────────────────────

class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')

        # Paramètre : chemin vers les fichiers pkl
        default_results = os.path.join(
            os.path.expanduser('~'),
            'robotics', 'probo-11', 'src', 'carte', 'results'
        )
        self.declare_parameter('carte_results_dir', default_results)
        results_dir = self.get_parameter(
            'carte_results_dir').get_parameter_value().string_value

        # Charger la carte
        self.get_logger().info(f'Chargement carte : {results_dir}')
        self.voxel_positions_, self.manip_values_ = load_map(results_dir)
        self.get_logger().info(
            f'Carte chargée : {len(self.voxel_positions_)} voxels.')

        # État interne
        self.current_rail_pos_  = 0.0
        self.rail_done_         = threading.Event()
        self.arm_done_          = threading.Event()
        self.busy_              = False
        self._pending_pose_     = None   # pose world en attente si busy

        # Publishers
        self.pub_rail_  = self.create_publisher(Float64,     '/target_rail_pos',   5)
        self.pub_robot_ = self.create_publisher(PoseStamped, '/target_pose_robot', 5)
        self.pub_busy_  = self.create_publisher(Bool,        '/commander/busy',    10)

        # Subscribers
        self.create_subscription(PoseStamped, '/target_pose',
                                 self._on_target_pose, 5)
        self.create_subscription(Bool, '/rail_mover/done',
                                 self._on_rail_done, 10)
        self.create_subscription(Float64, '/current_rail_position',
                                 self._on_rail_pos, 10)
        self.create_subscription(Bool, '/cartesian_commander/busy',
                                 self._on_arm_busy, 10)

        self._publish_busy(False)
        self.get_logger().info(
            'mission_coordinator prêt.\n'
            '  Envoyez une pose sur /target_pose (frame world) depuis le GUI.')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_rail_pos(self, msg: Float64):
        self.current_rail_pos_ = msg.data

    def _on_rail_done(self, msg: Bool):
        if msg.data:
            self.rail_done_.set()
        else:
            self.get_logger().error('rail_mover a signalé un échec.')

    def _on_arm_busy(self, msg: Bool):
        if not msg.data and self.busy_:
            # Le bras a fini son mouvement → mission terminée
            self.arm_done_.set()

    def _on_target_pose(self, msg: PoseStamped):
        if self.busy_:
            self.get_logger().warn('Mission en cours — pose ignorée.')
            return
        # Lancer la mission dans un thread séparé pour ne pas bloquer le spin
        threading.Thread(target=self._run_mission, args=(msg,), daemon=True).start()

    # ── Séquence principale ──────────────────────────────────────────────────

    def _run_mission(self, msg: PoseStamped):
        self.busy_ = True
        self._publish_busy(True)

        p = msg.pose.position
        target_world = (p.x, p.y, p.z)

        self.get_logger().info(
            f'Mission : cible world ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

        # ── 1. Rail optimal ─────────────────────────────────────────────────
        rail_pos, manip = find_optimal_rail(
            target_world, self.voxel_positions_, self.manip_values_)
        self.get_logger().info(
            f'Rail optimal : {rail_pos:.2f} m  (manip={manip:.4f})')

        # ── 2. Déplacer le rail ──────────────────────────────────────────────
        self.rail_done_.clear()
        cmd = Float64()
        cmd.data = rail_pos
        self.pub_rail_.publish(cmd)

        if not self.rail_done_.wait(timeout=10.0):
            self.get_logger().error('Timeout rail_mover — mission annulée.')
            self._publish_busy(False)
            self.busy_ = False
            return

        # ── 3. Convertir world → fr3_link0 ──────────────────────────────────
        target_robot = world_to_robot(target_world, rail_pos)

        pose_robot = PoseStamped()
        pose_robot.header.stamp    = self.get_clock().now().to_msg()
        pose_robot.header.frame_id = 'fr3_link0'
        pose_robot.pose.position.x = float(target_robot[0])
        pose_robot.pose.position.y = float(target_robot[1])
        pose_robot.pose.position.z = float(target_robot[2])
        pose_robot.pose.orientation = msg.pose.orientation  # même orientation

        self.get_logger().info(
            f'Pose fr3_link0 : ({target_robot[0]:.3f}, '
            f'{target_robot[1]:.3f}, {target_robot[2]:.3f})')

        # ── 4. Commander le bras ─────────────────────────────────────────────
        self.arm_done_.clear()
        self.pub_robot_.publish(pose_robot)

        if not self.arm_done_.wait(timeout=30.0):
            self.get_logger().warn('Timeout cartesian_commander.')

        self._publish_busy(False)
        self.busy_ = False
        self.get_logger().info('Mission terminée.')

    # ── Helper ───────────────────────────────────────────────────────────────

    def _publish_busy(self, busy: bool):
        msg = Bool()
        msg.data = busy
        self.pub_busy_.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(MissionCoordinator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
