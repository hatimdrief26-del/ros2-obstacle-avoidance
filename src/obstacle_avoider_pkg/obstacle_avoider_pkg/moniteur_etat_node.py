#!/usr/bin/env python3
"""
Noeud 2 : moniteur_etat_node
------------------------------
Ce noeud surveille l'etat general du robot et publie
un rapport d'etat periodique sur le topic /robot_status.

Architecture ROS 2 :
  - Subscriber : /scan     (sensor_msgs/LaserScan)
  - Subscriber : /cmd_vel  (geometry_msgs/Twist)
  - Publisher  : /robot_status (std_msgs/String)
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math


class MoniteurEtatNode(Node):
    """
    Noeud de surveillance et de rapport d'etat du robot.
    Publie un message texte resume toutes les secondes.
    """

    def __init__(self):
        super().__init__('moniteur_etat_node')

        # ---- Seuils ----
        self.declare_parameter('seuil_alerte', 0.5)
        self.seuil_alerte = self.get_parameter('seuil_alerte').value

        # ---- Variables internes ----
        self.dist_frontale  = float('inf')
        self.dist_min_scan  = float('inf')
        self.vitesse_lin    = 0.0
        self.vitesse_ang    = 0.0
        self.compteur_evite = 0
        self.en_rotation    = False

        # ---- Subscriber LiDAR ----
        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.callback_scan,
            10
        )

        # ---- Subscriber cmd_vel ----
        self.sub_cmd = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback_cmd_vel,
            10
        )

        # ---- Publisher etat ----
        self.pub_status = self.create_publisher(String, 'robot_status', 10)

        # ---- Timer publication (1 Hz) ----
        self.timer_pub = self.create_timer(1.0, self.publier_etat)

        self.get_logger().info('[moniteur_etat_node] Demarre.')

    # ------------------------------------------------------------------
    def callback_scan(self, msg: LaserScan):
        """Mise a jour des distances depuis le LiDAR."""
        ranges_valides = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max
        ]
        if not ranges_valides:
            return

        n = len(msg.ranges)
        half = n // 8   # 45 deg de chaque cote

        frontal = (
            [msg.ranges[i] for i in range(half) if msg.range_min < msg.ranges[i] < msg.range_max] +
            [msg.ranges[i] for i in range(n - half, n) if msg.range_min < msg.ranges[i] < msg.range_max]
        )
        self.dist_frontale = min(frontal) if frontal else float('inf')
        self.dist_min_scan = min(ranges_valides)

    def callback_cmd_vel(self, msg: Twist):
        """Mise a jour des vitesses commandees."""
        self.vitesse_lin = msg.linear.x
        self.vitesse_ang = msg.angular.z

        # Detecter les evitements (passage vitesse lin=0, ang!=0)
        if abs(msg.angular.z) > 0.05 and abs(msg.linear.x) < 0.01:
            if not self.en_rotation:
                self.en_rotation = True
                self.compteur_evite += 1
        else:
            self.en_rotation = False

    def publier_etat(self):
        """Publie le rapport d'etat formate."""
        if self.dist_frontale < self.seuil_alerte:
            niveau = 'ALERTE'
        elif self.dist_frontale < self.seuil_alerte * 2.0:
            niveau = 'ATTENTION'
        else:
            niveau = 'OK'

        msg = String()
        msg.data = (
            f'[ETAT:{niveau}] '
            f'dist_frontale={self.dist_frontale:.2f}m | '
            f'dist_min={self.dist_min_scan:.2f}m | '
            f'vlin={self.vitesse_lin:.2f}m/s | '
            f'vang={self.vitesse_ang:.2f}rad/s | '
            f'evitements={self.compteur_evite}'
        )
        self.pub_status.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MoniteurEtatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
