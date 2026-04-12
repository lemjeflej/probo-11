#!/usr/bin/env python3
"""
rail_mover.py
=============
Déplace le robot et le curseur sur le rail via gz service set_pose.

Topics
------
  SUB  /target_rail_pos          std_msgs/Float64
       → position rail souhaitée (m), [0.0 – 1.7]
  PUB  /rail_mover/done          std_msgs/Bool
       → True quand le déplacement est terminé
  PUB  /current_rail_position    std_msgs/Float64
       → position rail courante (mise à jour après chaque déplacement)
"""

import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


# Offsets fixes issus de l'URDF (identiques à optimal_rail_finder.py)
Y_BASE  = -0.85   # y du robot quand rail_position = 0.0
Z_ARM   =  0.0    # z de l'origine du modèle probo11 dans Gazebo
Z_CURS  =  1.03   # z du curseur (sur le rail : table 1m + rail 0.02m + demi-carriage)


def gz_set_pose(model_name: str, x: float, y: float, z: float, logger=None) -> bool:
    """Appelle ign service set_pose pour téléporter un modèle Gazebo (Ignition 6 / Humble)."""
    req = (
        f'name: "{model_name}" '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}'
    )
    result = subprocess.run(
        ['ign', 'service',
         '-s', '/world/empty/set_pose',
         '--reqtype',  'ignition.msgs.Pose',
         '--reptype',  'ignition.msgs.Boolean',
         '--timeout',  '3000',
         '--req',      req],
        capture_output=True, text=True
    )
    if logger:
        logger.info(f'set_pose {model_name} → code={result.returncode} '
                    f'stdout={result.stdout.strip()!r} stderr={result.stderr.strip()!r}')
    return result.returncode == 0


class RailMover(Node):
    def __init__(self):
        super().__init__('rail_mover')

        self.declare_parameter('initial_rail_position', 0.0)
        self.rail_pos_ = self.get_parameter(
            'initial_rail_position').get_parameter_value().double_value

        self.pub_done_ = self.create_publisher(Bool,    '/rail_mover/done',       10)
        self.pub_pos_  = self.create_publisher(Float64, '/current_rail_position', 10)

        self.create_subscription(Float64, '/target_rail_pos',
                                 self._on_target, 5)

        # Publier la position initiale
        self._publish_current()

        self.get_logger().info(
            f'rail_mover prêt. Position initiale : {self.rail_pos_:.2f} m')

    # ----------------------------------------------------------------
    def _on_target(self, msg: Float64):
        new_pos = float(msg.data)
        new_pos = max(0.0, min(1.7, new_pos))   # clamp

        if abs(new_pos - self.rail_pos_) < 1e-4:
            self.get_logger().info('Rail déjà en position.')
            self._publish_done(True)
            return

        self.get_logger().info(
            f'Déplacement rail : {self.rail_pos_:.2f} → {new_pos:.2f} m')

        y = Y_BASE + new_pos

        ok1 = gz_set_pose('probo11',      0.0, y, Z_ARM,  self.get_logger())
        ok2 = gz_set_pose('rail_curseur', 0.0, y, Z_CURS, self.get_logger())

        if ok1 and ok2:
            self.rail_pos_ = new_pos
            self.get_logger().info(f'Rail positionné à {new_pos:.2f} m.')
            self._publish_done(True)
        else:
            self.get_logger().error(
                f'Échec set_pose (probo11={ok1}, curseur={ok2}). '
                'Gazebo en cours de démarrage ?')
            self._publish_done(False)

        self._publish_current()

    # ----------------------------------------------------------------
    def _publish_done(self, success: bool):
        msg = Bool()
        msg.data = success
        self.pub_done_.publish(msg)

    def _publish_current(self):
        msg = Float64()
        msg.data = self.rail_pos_
        self.pub_pos_.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(RailMover())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
