#!/usr/bin/env python3
"""
rail_state_publisher.py
=======================
Publie l'état du rail (rail_joint) sur /joint_states.

Le rail n'est pas géré par ros2_control (incompatibilité avec
FrankaHardwareInterface). Ce nœud simule le rail côté TF :
robot_state_publisher reçoit la position du joint et met à jour
les transformations géométriques en conséquence.

Topics ROS 2 :
  SUB  /target_rail_pos   std_msgs/Float64   position cible (m)
  PUB  /joint_states      sensor_msgs/JointState
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


RAIL_JOINT = 'rail_joint'
PUBLISH_RATE = 50.0   # Hz
RAIL_MIN = 0.0        # m
RAIL_MAX = 1.7        # m


class RailStatePublisher(Node):
    def __init__(self):
        super().__init__('rail_state_publisher')

        self.position_ = 0.0

        self.pub_ = self.create_publisher(JointState, '/joint_states', 10)

        self.create_subscription(
            Float64,
            '/target_rail_pos',
            self._on_target,
            10
        )

        self.create_timer(1.0 / PUBLISH_RATE, self._publish)

        self.get_logger().info(
            'rail_state_publisher démarré. '
            'Envoyez une position (m) sur /target_rail_pos.'
        )

    def _on_target(self, msg: Float64):
        pos = max(RAIL_MIN, min(RAIL_MAX, msg.data))
        if pos != msg.data:
            self.get_logger().warn(
                f'Position rail clampée : {msg.data:.3f} → {pos:.3f} m '
                f'(limites [{RAIL_MIN}, {RAIL_MAX}] m)'
            )
        self.position_ = pos
        self.get_logger().info(f'Rail → {self.position_:.3f} m')

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [RAIL_JOINT]
        msg.position = [self.position_]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.pub_.publish(msg)


def main():
    rclpy.init()
    node = RailStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
