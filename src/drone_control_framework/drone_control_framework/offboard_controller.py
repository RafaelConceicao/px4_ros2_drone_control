#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from px4_msgs.msg import (
    VehicleOdometry,
    VehicleCommand,
    VehicleCommandAck,
    OffboardControlMode,
    TrajectorySetpoint
)

from drone_control_framework.state_machine import (
    StateMachine,
    FlightState
)

from drone_control_framework.px4_interface import PX4Interface
from px4_msgs.msg import VehicleStatus


class OffboardController(Node):

    ###########################################################

    def __init__(self):

        super().__init__("offboard_controller")

        self.get_logger().info("=====================================")
        self.get_logger().info(" Drone Control Framework")
        self.get_logger().info(" Sprint 4")
        self.get_logger().info("=====================================")

        self.vehicle_status = VehicleStatus()

        #######################################################

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        #######################################################
        # Subscribers
        #######################################################

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos
        )

        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.vehicle_status_callback,
            qos
        )


        self.ack_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack_v1",
            self.command_ack_callback,
            qos
        )

        #######################################################
        # Publishers
        #######################################################

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos
        )

        #######################################################

        self.px4 = PX4Interface(
            self,
            self.command_pub,
            self.offboard_pub,
            self.trajectory_pub
        )

        #######################################################
        # Drone state
        #######################################################

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        #######################################################
        # Reference
        #######################################################

        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_z = -2.0

        #######################################################

        self.kp = 1.0

        #######################################################

        self.state_machine = StateMachine()

        #######################################################

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        self.print_timer = self.create_timer(
            1.0,
            self.print_status
        )

        self.get_logger().info("Framework iniciado.")

    ###########################################################

    def odom_callback(self, msg):

        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = msg.position[2]

        self.vx = msg.velocity[0]
        self.vy = msg.velocity[1]
        self.vz = msg.velocity[2]

        if not self.state_machine.odometry_received:
            self.get_logger().info("ODOMETRIA RECEBIDA")

        self.state_machine.update_odometry()

    def is_armed(self):

        return (
            self.vehicle_status.arming_state ==
            VehicleStatus.ARMING_STATE_ARMED
        )

    ###########################################################
    def vehicle_status_callback(self, msg):

        self.vehicle_status = msg

    def command_ack_callback(self, msg):

        self.get_logger().info(
            f"ACK -> comando={msg.command} resultado={msg.result}"
        )

    ###########################################################

    def position_controller(self):

        ex = self.ref_x - self.x
        ey = self.ref_y - self.y
        ez = self.ref_z - self.z

        vx = self.kp * ex
        vy = self.kp * ey
        vz = self.kp * ez

        return vx, vy, vz

    ###########################################################

    def control_loop(self):

        #######################################################
        # Publicação contínua (necessária para OFFBOARD)
        #######################################################

        vx_cmd, vy_cmd, vz_cmd = self.position_controller()

        self.px4.publish_offboard_mode()

        self.px4.publish_setpoint(
            self.ref_x,
            self.ref_y,
            self.ref_z,
            vx_cmd,
            vy_cmd,
            vz_cmd,
            yaw=0.0
        )

        #######################################################

        self.state_machine.run()

        state = self.state_machine.state

        #######################################################

        if state == FlightState.WAIT_FOR_ODOMETRY:
            return

        elif state == FlightState.SEND_INITIAL_SETPOINTS:
            return

        elif state == FlightState.REQUEST_OFFBOARD:
            self.px4.set_offboard_mode()

        elif state == FlightState.REQUEST_ARM:

            self.px4.arm()

            if self.is_armed():
                self.state_machine.goto(FlightState.TAKEOFF)

        elif state == FlightState.TAKEOFF:
            pass

        elif state == FlightState.HOVER:
            pass

        elif state == FlightState.MISSION:
            pass

        elif state == FlightState.LAND:
            self.px4.land()

    ###########################################################

    def print_status(self):

        vx_cmd, vy_cmd, vz_cmd = self.position_controller()

        self.get_logger().info("--------------------------------")

        self.get_logger().info(
            f"Estado: {self.state_machine.state.name}"
        )

        self.get_logger().info(
            f"Posição: "
            f"{self.x:.2f} "
            f"{self.y:.2f} "
            f"{self.z:.2f}"
        )

        self.get_logger().info(
            f"Velocidade: "
            f"{self.vx:.2f} "
            f"{self.vy:.2f} "
            f"{self.vz:.2f}"
        )

        self.get_logger().info(
            f"Referência: "
            f"{self.ref_x:.2f} "
            f"{self.ref_y:.2f} "
            f"{self.ref_z:.2f}"
        )

        self.get_logger().info(
            f"Controle: "
            f"{vx_cmd:.2f} "
            f"{vy_cmd:.2f} "
            f"{vz_cmd:.2f}"
        )


###############################################################


def main():

    rclpy.init()

    node = OffboardController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()