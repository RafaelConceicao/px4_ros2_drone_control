#!/usr/bin/env python3

from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint
)


class PX4Interface:
    """
    Interface responsável por toda comunicação com o PX4.
    """

    ##########################################################

    def __init__(
        self,
        node,
        command_publisher,
        offboard_publisher,
        trajectory_publisher
    ):

        self.node = node

        self.command_pub = command_publisher
        self.offboard_pub = offboard_publisher
        self.trajectory_pub = trajectory_publisher

    ##########################################################

    def timestamp(self):

        return int(
            self.node.get_clock().now().nanoseconds / 1000
        )

    ##########################################################
    # VEHICLE COMMAND
    ##########################################################

    def send_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = command

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    ##########################################################
    # ARM
    ##########################################################

    def arm(self):

        self.node.get_logger().info(">>> ARM")

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    ##########################################################
    # DISARM
    ##########################################################

    def disarm(self):

        self.node.get_logger().info(">>> DISARM")

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )

    ##########################################################
    # OFFBOARD MODE
    ##########################################################

    def engage_offboard_mode(self):

        self.node.get_logger().info(">>> OFFBOARD")

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )

    ##########################################################

    def set_offboard_mode(self):
        self.engage_offboard_mode()

    ##########################################################
    # LAND
    ##########################################################

    def land(self):

        self.node.get_logger().info(">>> LAND")

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

    ##########################################################
    # TAKEOFF
    ##########################################################

    def takeoff(self, altitude=2.0):

        self.node.get_logger().info(">>> TAKEOFF")

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7=float(altitude)
        )

    ##########################################################
    # RTL
    ##########################################################

    def return_to_launch(self):

        self.node.get_logger().info(">>> RTL")

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        )

    ##########################################################
    # KILL
    ##########################################################

    def kill(self):

        self.node.get_logger().warning(">>> KILL")

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=21196.0
        )

    ##########################################################
    # OFFBOARD CONTROL MODE
    ##########################################################

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    ##########################################################
    # TRAJECTORY SETPOINT
    ##########################################################

    def publish_setpoint(
        self,
        x,
        y,
        z,
        vx,
        vy,
        vz,
        yaw=0.0
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        msg.position = [
            float(x),
            float(y),
            float(z)
        ]

        msg.velocity = [
            float(vx),
            float(vy),
            float(vz)
        ]

        msg.acceleration = [
            float("nan"),
            float("nan"),
            float("nan")
        ]

        msg.jerk = [
            float("nan"),
            float("nan"),
            float("nan")
        ]

        msg.yaw = float(yaw)
        msg.yawspeed = float("nan")

        self.trajectory_pub.publish(msg)