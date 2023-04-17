import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry


class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')

        # qualisty of service
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscriber
        self.vehicle_odometry_subscriber_ = self.create_subscription(VehicleOdometry, 
                                                                        "/fmu/out/vehicle_odometry", self.vehicle_odometry, qos_profile)
        self.vehicle_local_position_subscriber_ = self.create_subscription(VehicleLocalPosition, 
                                                                        "/fmu/out/vehicle_local_position", self.get_vehicle_position, qos_profile)
        self.vehicle_status_subscriber_ = self.create_subscription(VehicleStatus, 
                                                                        "/fmu/out/vehicle_status", self.get_vehicle_status, qos_profile)
        
        # publisher
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                        "/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, 
                                                                        "/fmu/in/vehicle_command", qos_profile)
        self.vehicle_mocap_publisher_ = self.create_publisher(VehicleOdometry, 
                                                                        "/fmu/in/vehicle_mocap_odometry", qos_profile)
        

        self.offboard_setpoint_counter_ = 0
        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.land_to_initial_position = False
        self.status = 0

        # Vehicle position (m)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Odometry position (m)
        self.x_mocap = 0.0
        self.y_mocap = 0.0
        self.z_mocap = 0.0

        # Point list definition
        p2 = Point(3.0, 0.0, -2.5)
        p1 = Point(0.0, 0.0, -2.5)
        self.point_list = [p1, p2]
        self.n = len(self.point_list)
        
        self.range = 0.3 # 30 cm
        self.end = False
        self.i = 0
        self.temp = 0
        self.takeoff_finished = 0


    def timer_callback(self):

        # Arm and takeoff
        if (self.offboard_setpoint_counter_ == 0):

            #Print initial position
            self.get_logger().info("Initial position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))

            # Arm the vehicle and takeoff
            self.arm()
            self.takeoff()         

        # Check takeoff finished
        if (self.offboard_setpoint_counter_ >= 10 and self.status == 4 and self.takeoff_finished == 0):
            self.get_logger().info("Takeoff completed")
            self.takeoff_finished = 1
        
        # Trajectory setpoint
        if (self.takeoff_finished == 1 and self.i < self.n and self.temp < 1):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()

        # Land
        if(self.end == True and self.temp == 1):

            if(self.land_to_initial_position == False):
                self.land()
            else:
                self.land_to_initial_pos()
            
            self.temp += 1
        
        # Disarm
        if(abs(self.z) <= self.range and self.temp == 2):
            self.get_logger().info("Final position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
            self.disarm()
            self.temp += 1

        self.offboard_setpoint_counter_ += 1

    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")
    
    # Takeoff
    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan, 3.0)
        self.get_logger().info("Takeoff command sent")

    # Loiter
    def loiter(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LOITER_TIME, 5.0)
        self.get_logger().info("Loitering around mission")

    # Land
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent")

    # Land to initial position
    def land_to_initial_pos(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("Landing to initial position")   


    '''
	Publish the offboard control mode.
	For this example, only position and altitude controls are active.
    '''
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)


    '''
	Publish a trajectory setpoint
	For this example, it sends a trajectory setpoint to make the
	vehicle hover at 5 meters with a yaw angle of 180 degrees.
    '''
    def publish_trajectory_setpoint(self):
        
        msg = TrajectorySetpoint()
        point = self.point_list
        range = self.range

        if(self.distance(point[self.i]) <= range):
            if self.i < self.n - 1:
                self.get_logger().info("Position reached: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
                self.i += 1
            elif(self.temp<1):
                self.end = True
                self.temp = 1
                self.get_logger().info("Position reached: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
        

        msg.position = [point[self.i].x, point[self.i].y, point[self.i].z]
        #msg.yaw = 0.0 

        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

    '''
	Publish a trajectory setpoint
	For this example, it sends a trajectory setpoint to make the
	vehicle hover at 5 meters with a yaw angle of 180 degrees.
    '''
    def publish_trajectory_setpoint(self):
        
        msg = TrajectorySetpoint()
        point = self.point_list
        range = self.range

        if(self.distance(point[self.i]) <= range):
            if self.i < self.n - 1:
                self.get_logger().info("Position reached: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
                self.i += 1
            elif(self.temp<1):
                self.end = True
                self.temp = 1
                self.get_logger().info("Position reached: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
        

        msg.position = [point[self.i].x, point[self.i].y, point[self.i].z]
        #msg.yaw = 0.0 

        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)


    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def get_vehicle_position(self, msg):
        # x, y, z are updated at each time step
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        #self.get_logger().info("Actual position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
        vx = msg.vx
        vy = msg.vy
        vz = msg.vz
        #self.get_logger().info("Actual velocity: ({:.2f}, {:.2f}, {:.2f})".format(vx, vy, vz))

    def vehicle_odometry(self, msg):
        self.x_mocap = msg.position[0]
        self.y_mocap = msg.position[1]
        self.z_mocap = msg.position[2]
        #self.get_logger().info("Odometry position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))

    def get_vehicle_status(self, msg):
        self.status = msg.nav_state
        #print("status = ", self.status)

    def distance(self, p):
        d = np.sqrt((p.x- self.x)**2 + (p.y- self.y)**2 + (p.z- self.z)**2)
        return d
        
def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


class Point:
    
    """
        Store position of the drone as a point in 3D space
    """


    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def set_point(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

#   __  __    _    ___ _   _ 
#  |  \/  |  / \  |_ _| \ | |
#  | |\/| | / _ \  | ||  \| |
#  | |  | |/ ___ \ | || |\  |
#  |_|  |_/_/   \_\___|_| \_|
                           
if __name__ == '__main__':
    main()
