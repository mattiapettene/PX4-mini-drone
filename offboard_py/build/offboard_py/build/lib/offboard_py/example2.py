import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node): 
    def __init__(self):
        super().__init__('offboard_control')

        # Publisher for offboard control mode
        self._control_mode_pub = self.create_publisher(
            OffboardControlMode, '/px4/offboard/offboard_control_mode', 10)
        
        # Publisher for setpoint trajectory
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/px4/offboard/trajectory_setpoint', 10)

        # Subscriber for vehicle status
        self._vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/px4/vehicle_status', self.vehicle_status_callback, 10)

        # Setpoint trajectory message
        self._setpoint_msg = TrajectorySetpoint()

        # Offboard control mode message
        self._control_mode_msg = OffboardControlMode()
        self._control_mode_msg.position = True
        self._control_mode_msg.velocity = False
        self._control_mode_msg.acceleration = False
        self._control_mode_msg.attitude = False
        self._control_mode_msg.body_rate = False

        # Timer for control loop
        self._control_loop_timer = self.create_timer(0.1, self.control_loop)

        # Flag for offboard control
        self._offboard_enabled = False

    def vehicle_status_callback(self, msg):
        # Set offboard mode to enabled when vehicle is armed
        if msg.armed and not self._offboard_enabled:
            self._offboard_enabled = True
            self.get_logger().info('Offboard mode enabled')
            self._control_mode_pub.publish(self._control_mode_msg)

    def control_loop(self):
        if self._offboard_enabled:
            # Setpoint trajectory message
            self._setpoint_msg.header.stamp = self.get_clock().now().to_msg()
            self._setpoint_msg.type = TrajectorySetpoint.TYPE_TRAJECTORY
            self._setpoint_msg.dimension = 3
            self._setpoint_msg.yaw_valid = True

            # Setpoints for circle trajectory
            radius = 5
            omega = 0.5
            dt = 0.1

            x = radius * np.cos(omega * self.get_clock().now().to_msg().to_sec())
            y = radius * np.sin(omega * self.get_clock().now().to_msg().to_sec())
            z = 2

            self._setpoint_msg.x = x
            self._setpoint_msg.y = y
            self._setpoint_msg.z = z

            # Publish setpoint trajectory
            self._setpoint_pub.publish(self._setpoint_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
