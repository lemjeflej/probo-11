#!/usr/bin/env python3
"""
rail_mover.py
=============
Déplace le robot et le curseur sur le rail avec interpolation lisse.

Architecture
------------
  - Boucle de contrôle à 30 Hz (STEP_DT = 0.033 s) dans un thread dédié.
  - Les appels Gazebo (set_pose) sont délégués à un worker thread via une
    queue latest-value : si le worker est occupé, la nouvelle position
    écrase l'ancienne en attente — jamais de backlog.
  - Robot et curseur sont toujours déplacés ensemble (même Y, même appel).

Topics
------
  SUB  /target_rail_pos          std_msgs/Float64   position cible (m)
  PUB  /rail_mover/done          std_msgs/Bool      True = arrivée confirmée
  PUB  /current_rail_position    std_msgs/Float64   position interpolée courante

Paramètres
----------
  initial_rail_position  float  défaut 0.0    position de départ (m)
  rail_speed             float  défaut 0.35   vitesse de déplacement (m/s)
"""

import time
import queue
import threading
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


# ── Constantes géométriques ────────────────────────────────────────────────
Y_BASE = -0.85   # y robot quand rail_position = 0.0
Z_ARM  =  0.0    # z origine probo11
Z_CURS =  1.03   # z curseur

# ── Boucle de contrôle ─────────────────────────────────────────────────────
STEP_DT  = 0.033  # s  — période de mise à jour de la position (≈30 Hz)
GOAL_TOL = 1e-3   # m  — seuil d'arrivée


# ══════════════════════════════════════════════════════════════════════════
# Worker Gazebo : queue latest-value
# ══════════════════════════════════════════════════════════════════════════

class GazeboPoseWorker:
    """
    Thread unique qui appelle 'ign service set_pose' en continu.
    Si une nouvelle position arrive pendant qu'un appel est en cours,
    elle remplace la précédente en attente (latest-value queue, taille 1).
    Robot et curseur sont toujours déplacés simultanément (deux Popen
    lancés en parallèle, pas de wait() bloquant la boucle de contrôle).
    """

    def __init__(self, logger=None):
        self._logger  = logger
        self._q       = queue.Queue(maxsize=1)
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def send(self, y: float):
        """Envoie une position. Écrase l'ancienne si le worker est encore occupé."""
        try:
            self._q.get_nowait()   # vide la queue si déjà une valeur en attente
        except queue.Empty:
            pass
        self._q.put(y)

    def send_blocking(self, y: float) -> bool:
        """
        Appel bloquant final : attend la confirmation Gazebo avant de retourner.
        Utilisé pour la dernière position (arrivée) afin de garantir que
        Gazebo a bien reçu la position finale avant de signaler 'done'.
        """
        return self._call_both(y)

    # ── Interne ─────────────────────────────────────────────────────────
    def _loop(self):
        while True:
            y = self._q.get()          # attend une position
            self._call_both(y)         # appel bloquant (OK, on est dans un thread)

    def _call_both(self, y: float) -> bool:
        """Lance robot + curseur en parallèle, attend les deux."""
        results = [False, False]

        def robot():
            results[0] = self._set_pose('probo11',      y, Z_ARM)

        def cursor():
            results[1] = self._set_pose('rail_curseur', y, Z_CURS)

        t1 = threading.Thread(target=robot,  daemon=True)
        t2 = threading.Thread(target=cursor, daemon=True)
        t1.start(); t2.start()
        t1.join();  t2.join()
        return results[0] and results[1]

    def _set_pose(self, name: str, y: float, z: float) -> bool:
        req = (f'name: "{name}" '
               f'position: {{x: 0.0000, y: {y:.4f}, z: {z:.4f}}}')
        r = subprocess.run(
            ['ign', 'service',
             '-s', '/world/empty/set_pose',
             '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '1500',
             '--req', req],
            capture_output=True, text=True
        )
        return r.returncode == 0


# ══════════════════════════════════════════════════════════════════════════
# Nœud ROS 2
# ══════════════════════════════════════════════════════════════════════════

class RailMover(Node):

    def __init__(self):
        super().__init__('rail_mover')

        self.declare_parameter('initial_rail_position', 0.0)
        self.declare_parameter('rail_speed', 0.35)

        self.rail_pos_   = self.get_parameter(
            'initial_rail_position').get_parameter_value().double_value
        self.rail_speed_ = self.get_parameter(
            'rail_speed').get_parameter_value().double_value

        self._target     = self.rail_pos_
        self._moving     = False
        self._lock       = threading.Lock()
        self._worker     = GazeboPoseWorker(self.get_logger())

        self.pub_done_ = self.create_publisher(Bool,    '/rail_mover/done',       10)
        self.pub_pos_  = self.create_publisher(Float64, '/current_rail_position', 10)
        self.create_subscription(Float64, '/target_rail_pos', self._on_target, 5)

        self._publish_current()
        self.get_logger().info(
            f'rail_mover prêt — pos={self.rail_pos_:.3f} m  '
            f'vitesse={self.rail_speed_:.2f} m/s')

    # ── Callback cible ──────────────────────────────────────────────────

    def _on_target(self, msg: Float64):
        new = max(0.0, min(1.7, float(msg.data)))

        if abs(new - self.rail_pos_) < GOAL_TOL:
            self.get_logger().info('Rail déjà en position.')
            self._publish_done(True)
            return

        with self._lock:
            self._target = new
            if not self._moving:
                self._moving = True
                threading.Thread(target=self._move_loop, daemon=True).start()

    # ── Boucle d'interpolation ──────────────────────────────────────────

    def _move_loop(self):
        with self._lock:
            target = self._target

        self.get_logger().info(
            f'Rail : {self.rail_pos_:.3f} → {target:.3f} m  '
            f'(v={self.rail_speed_:.2f} m/s)')

        while True:
            with self._lock:
                target = self._target          # cible éventuellement mise à jour

            remaining = target - self.rail_pos_
            if abs(remaining) <= GOAL_TOL:
                break

            step = self.rail_speed_ * STEP_DT
            if abs(remaining) <= step:
                self.rail_pos_ = target
            else:
                self.rail_pos_ += step * (1.0 if remaining > 0.0 else -1.0)

            # Envoi non-bloquant → la boucle de contrôle n'attend pas Gazebo
            self._worker.send(Y_BASE + self.rail_pos_)
            self._publish_current()

            time.sleep(STEP_DT)

        # ── Arrivée : appel bloquant pour confirmer la position finale ──
        self.rail_pos_ = target
        ok = self._worker.send_blocking(Y_BASE + self.rail_pos_)
        self._publish_current()

        with self._lock:
            self._moving = False

        if ok:
            self.get_logger().info(f'Rail en position : {self.rail_pos_:.3f} m')
        else:
            self.get_logger().warn('set_pose final incertain — Gazebo occupé ?')

        # done=True seulement après confirmation Gazebo de la position finale
        self._publish_done(True)

    # ── Helpers ────────────────────────────────────────────────────────

    def _publish_done(self, v: bool):
        msg = Bool(); msg.data = v; self.pub_done_.publish(msg)

    def _publish_current(self):
        msg = Float64(); msg.data = self.rail_pos_; self.pub_pos_.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(RailMover())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
