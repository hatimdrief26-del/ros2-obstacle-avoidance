#!/usr/bin/env python3
"""
Noeud 1 : evitement_obstacles_node
------------------------------------
Ce noeud est le cerveau du robot eviteur d'obstacles.
Il souscrit aux donnees LiDAR (/scan) et publie des
commandes de vitesse (/cmd_vel).

Architecture ROS 2 :
  - Subscriber : /scan  (sensor_msgs/LaserScan)
  - Publisher  : /cmd_vel (geometry_msgs/Twist)
  - Service    : /set_safety_distance (SetSafetyDistance)
  - Action     : /advance_distance (AdvanceDistance)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from obstacle_avoider_interfaces.srv import SetSafetyDistance
from obstacle_avoider_interfaces.action import AdvanceDistance

import math
import threading
import time


class ObstacleAvoiderNode(Node):
    """
    Noeud principal d'evitement d'obstacles.

    Comportement :
      - Etat AVANCER   : le robot avance en ligne droite.
      - Etat TOURNER   : le robot tourne pour eviter un obstacle.
      - Etat ARRET     : arret d'urgence (securite).

    Transitions :
      AVANCER -> TOURNER  si dist_frontale < distance_securite
      TOURNER -> AVANCER  si dist_frontale > distance_securite * 1.5
      * -> ARRET          si dist_frontale < distance_arret_urgence
      ARRET   -> AVANCER  apres tempo
    """

    # --- Etats internes ---
    ETAT_AVANCER = 'AVANCER'
    ETAT_TOURNER = 'TOURNER'
    ETAT_ARRET   = 'ARRET'

    def __init__(self):
        super().__init__('evitement_obstacles_node')

        # ---- Parametres ----
        self.declare_parameter('distance_securite',       0.5)
        self.declare_parameter('distance_arret_urgence',  0.20)
        self.declare_parameter('vitesse_lineaire',        0.18)
        self.declare_parameter('vitesse_angulaire',       0.6)
        self.declare_parameter('angle_detection_frontal', 45.0)

        self.distance_securite      = self.get_parameter('distance_securite').value
        self.distance_arret_urgence = self.get_parameter('distance_arret_urgence').value
        self.vitesse_lineaire       = self.get_parameter('vitesse_lineaire').value
        self.vitesse_angulaire      = self.get_parameter('vitesse_angulaire').value
        self.angle_frontal          = self.get_parameter('angle_detection_frontal').value

        # ---- Etat machine ----
        self.etat_courant  = self.ETAT_AVANCER
        self.sens_rotation = 1.0   # +1 gauche, -1 droite
        self.temps_arret   = 0.0
        self._arret_duree  = 1.5   # secondes d'arret d'urgence

        # ---- Valeurs LiDAR ----
        self.dist_frontale = float('inf')
        self.dist_gauche   = float('inf')
        self.dist_droite   = float('inf')

        # ---- Groupe de callbacks ----
        cb_group = ReentrantCallbackGroup()

        # ---- Subscriber LiDAR ----
        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.callback_scan,
            10,
            callback_group=cb_group
        )

        # ---- Publisher cmd_vel ----
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # ---- Service : reglage distance de securite ----
        self.srv_distance = self.create_service(
            SetSafetyDistance,
            'set_safety_distance',
            self.callback_set_safety_distance,
            callback_group=cb_group
        )

        # ---- Action : avancer d'une distance precisee ----
        self._action_server = ActionServer(
            self,
            AdvanceDistance,
            'advance_distance',
            execute_callback=self.callback_action_advance,
            goal_callback=self.callback_goal,
            cancel_callback=self.callback_cancel,
            callback_group=cb_group
        )

        # ---- Timer de controle (10 Hz) ----
        self.timer_controle = self.create_timer(
            0.1,
            self.boucle_controle,
            callback_group=cb_group
        )

        self.get_logger().info(
            f'[evitement_obstacles_node] Demarre | '
            f'dist_securite={self.distance_securite} m | '
            f'vitesse={self.vitesse_lineaire} m/s'
        )

    # ------------------------------------------------------------------
    # Callback LiDAR
    # ------------------------------------------------------------------
    def callback_scan(self, msg: LaserScan):
        """
        Traite les donnees LiDAR pour extraire les distances
        frontale, gauche et droite.
        """
        n = len(msg.ranges)
        if n == 0:
            return

        # Convertir l'angle frontal en nombre d'echantillons
        half_samples = int((self.angle_frontal / 360.0) * n)

        # -- Distance frontale : secteur centre sur l'angle 0 --
        frontal_indices = (
            list(range(0, half_samples + 1)) +
            list(range(n - half_samples, n))
        )
        frontal_vals = [
            msg.ranges[i]
            for i in frontal_indices
            if msg.range_min < msg.ranges[i] < msg.range_max
        ]
        self.dist_frontale = min(frontal_vals) if frontal_vals else float('inf')

        # -- Distance gauche (90 deg) --
        idx_gauche = n // 4
        gauche_vals = [
            msg.ranges[i % n]
            for i in range(idx_gauche - 15, idx_gauche + 16)
            if msg.range_min < msg.ranges[i % n] < msg.range_max
        ]
        self.dist_gauche = min(gauche_vals) if gauche_vals else float('inf')

        # -- Distance droite (270 deg) --
        idx_droite = (3 * n) // 4
        droite_vals = [
            msg.ranges[i % n]
            for i in range(idx_droite - 15, idx_droite + 16)
            if msg.range_min < msg.ranges[i % n] < msg.range_max
        ]
        self.dist_droite = min(droite_vals) if droite_vals else float('inf')

    # ------------------------------------------------------------------
    # Boucle de controle principale (timer 10 Hz)
    # ------------------------------------------------------------------
    def boucle_controle(self):
        """
        Machine a etats qui genere les commandes de vitesse.
        """
        cmd = Twist()

        if self.etat_courant == self.ETAT_AVANCER:
            if self.dist_frontale < self.distance_arret_urgence:
                # Transition vers arret d'urgence
                self.etat_courant = self.ETAT_ARRET
                self.temps_arret  = time.time()
                self.get_logger().warn(
                    f'[SECURITE] Arret urgence ! dist={self.dist_frontale:.3f} m'
                )

            elif self.dist_frontale < self.distance_securite:
                # Choisir le sens de rotation (vers l'espace libre)
                self.sens_rotation = (
                    1.0 if self.dist_gauche >= self.dist_droite else -1.0
                )
                self.etat_courant = self.ETAT_TOURNER
                self.get_logger().info(
                    f'[TOURNER] dist_frontale={self.dist_frontale:.2f} m | '
                    f'sens={"GAUCHE" if self.sens_rotation > 0 else "DROITE"}'
                )

            else:
                # Avancer normalement
                cmd.linear.x  = self.vitesse_lineaire
                cmd.angular.z = 0.0

        elif self.etat_courant == self.ETAT_TOURNER:
            if self.dist_frontale > self.distance_securite * 1.5:
                # Voie libre : reprendre l'avance
                self.etat_courant = self.ETAT_AVANCER
                self.get_logger().info('[AVANCER] Voie libre, reprise de l avance.')
            else:
                # Tourner sur place
                cmd.linear.x  = 0.0
                cmd.angular.z = self.vitesse_angulaire * self.sens_rotation

        elif self.etat_courant == self.ETAT_ARRET:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            # Apres la duree d'arret, passer en mode tournage
            if time.time() - self.temps_arret > self._arret_duree:
                self.etat_courant = self.ETAT_TOURNER
                self.sens_rotation = (
                    1.0 if self.dist_gauche >= self.dist_droite else -1.0
                )
                self.get_logger().info('[TOURNER] Reprise apres arret urgence.')

        self.pub_cmd_vel.publish(cmd)

    # ------------------------------------------------------------------
    # Callback Service : SetSafetyDistance
    # ------------------------------------------------------------------
    def callback_set_safety_distance(self, request, response):
        """
        Modifie dynamiquement la distance de securite.
        Valeurs acceptees : 0.1 m a 2.0 m.
        """
        nouvelle_distance = request.distance

        if 0.1 <= nouvelle_distance <= 2.0:
            ancienne = self.distance_securite
            self.distance_securite = nouvelle_distance
            response.success = True
            response.message = (
                f'Distance de securite mise a jour : '
                f'{ancienne:.2f} m -> {nouvelle_distance:.2f} m'
            )
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = (
                f'Valeur invalide ({nouvelle_distance:.2f} m). '
                f'Plage acceptee : 0.1 - 2.0 m'
            )
            self.get_logger().warn(response.message)

        return response

    # ------------------------------------------------------------------
    # Callbacks Action : AdvanceDistance
    # ------------------------------------------------------------------
    def callback_goal(self, goal_request):
        if goal_request.distance_metres <= 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def callback_cancel(self, goal_handle):
        self.get_logger().info('[ACTION] Annulation recue.')
        return CancelResponse.ACCEPT

    async def callback_action_advance(self, goal_handle):
        """
        Fait avancer le robot d'une distance donnee (en metres).
        Publie un feedback regulier et arrete si un obstacle est detecte.
        """
        distance_cible = goal_handle.request.distance_metres
        self.get_logger().info(
            f'[ACTION] Avancer de {distance_cible:.2f} m'
        )

        feedback_msg   = AdvanceDistance.Feedback()
        result_msg     = AdvanceDistance.Result()
        distance_parcourue = 0.0
        vitesse        = self.vitesse_lineaire

        # Desactiver la boucle de controle automatique pendant l'action
        # en envoyant les commandes directement
        dt = 0.1
        start_time = time.time()

        while distance_parcourue < distance_cible:
            # Verification annulation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.distance_parcourue = distance_parcourue
                result_msg.succes = False
                result_msg.message = 'Action annulee par le client.'
                self._stopper_robot()
                return result_msg

            # Verification obstacle
            if self.dist_frontale < self.distance_securite:
                self.get_logger().warn(
                    f'[ACTION] Obstacle detecte a {self.dist_frontale:.2f} m, arret.'
                )
                self._stopper_robot()
                goal_handle.abort()
                result_msg.distance_parcourue = distance_parcourue
                result_msg.succes = False
                result_msg.message = (
                    f'Obstacle detecte a {self.dist_frontale:.2f} m. '
                    f'Distance parcourue : {distance_parcourue:.2f} m.'
                )
                return result_msg

            # Commande d'avance
            cmd = Twist()
            cmd.linear.x = vitesse
            self.pub_cmd_vel.publish(cmd)

            # Calcul distance parcourue (odometrie approximative)
            distance_parcourue += vitesse * dt

            # Feedback
            feedback_msg.distance_restante = distance_cible - distance_parcourue
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(dt)

        # Arret final
        self._stopper_robot()
        goal_handle.succeed()
        result_msg.distance_parcourue = distance_parcourue
        result_msg.succes = True
        result_msg.message = (
            f'Distance atteinte : {distance_parcourue:.2f} m en '
            f'{time.time() - start_time:.1f} s.'
        )
        self.get_logger().info(f'[ACTION] {result_msg.message}')
        return result_msg

    def _stopper_robot(self):
        """Publie une commande de vitesse nulle."""
        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stopper_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
