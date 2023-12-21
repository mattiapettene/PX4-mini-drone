import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleConstraints
# from geometry_msgs.msg import Pose

class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # self.vehicle_status_subscriber_ = self.create_subscription(VehicleStatus, 
        #                                                             "/fmu/out/vehicle_status", self.get_vehicle_status, qos_profile)
        self.vehicle_local_position_subscriber_ = self.create_subscription(VehicleOdometry, 
                                                                    "/fmu/out/vehicle_odometry", self.get_vehicle_position, qos_profile)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                    "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, 
                                                                    "/fmu/in/vehicle_command", qos_profile)
        # self.mocap_position_publisher_ = self.create_publisher(VehicleOdometry, 
        #                                                             "/fmu/in/vehicle_visual_odometry", qos_profile)
        self.vehicle_constraints_publisher_ = self.create_publisher(VehicleConstraints, 
                                                                    "/fmu/in/vehicle_constraints", qos_profile)
        
        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.offboard_setpoint_counter_ = 0
        self.land_to_initial_position = False
        self.print_position = False
        self.print_velocity = False
        self.print_status = False 

        # Vehicle actual position (m)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Point list tajectory definition
        p1 = Point(0.0, 0.0, -1.5)
        p2 = Point(2.0, 0.0, -1.5)
        self.point_list = [p1, p2]

        # Velocity control (m/s)
        self.velx = 0.1
        self.vely = 0.1
        self.velz = 0.1

        self.height_max = p1
        
        self.range = 0.20 # Set tolerance range to 50 cm

        self.actual_status = 0
        self.takeoff_finished = 0
        self.landing_flag = 0

        # UWB anchors position
        # self.position_mocap = Pose

    def timer_callback(self):

        self.publish_velocity_constraints()

        # Arm and takeoff
        if (self.offboard_setpoint_counter_ == 0):

            # Print initial position
            self.get_logger().info("Initial position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))

            # Arm the vehicle and takeoff
            self.arm()
            self.takeoff()

        if (self.distance(self.height_max) < 0.50):
            self.actual_status = 4
            
        # Check takeoff finished
        if (self.offboard_setpoint_counter_ >= 10 and self.actual_status == 4 and self.takeoff_finished == 0):
            self.get_logger().info("Takeoff completed")
            self.takeoff_finished = 1

        # Trajectory setpoint
        if (self.takeoff_finished == 1 and self.point_list):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()

        # Land
        if(len(self.point_list) == 0 and self.landing_flag == 0):  # when the list is empty all points are reached

            if(self.land_to_initial_position == False):
                self.land()
            else:
                self.land_to_initial_pos()
            
            self.landing_flag = 1
        
        # Disarm
        if(abs(self.z) <= 0.20 and self.landing_flag == 1):
            self.get_logger().info("Final position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
            self.disarm()
            rclpy.shutdown()

        self.offboard_setpoint_counter_ += 1


    # ------ FUNCTIONS -------                                                                    

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
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, math.nan, math.nan, math.nan, math.nan, math.nan, 1.5)
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

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_velocity_constraints(self):
        msg = VehicleConstraints() 
        msg.speed_up = 0.1
        msg.speed_down = 0.05
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_constraints_publisher_.publish(msg)

	# Publish a trajectory setpoint in position
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        point = self.point_list
        range = self.range

        if len(point) > 0:   # check the list is not empty
            
            msg.position = [point[0].x, point[0].y, point[0].z]
            msg.velocity = [self.velx, self.vely, self.velz]
            msg.yaw = math.nan

            if self.distance(point[0]) <= range:    # point is reached
                self.get_logger().info("Position reached: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
                self.loiter()
                point.pop(0)    # delete the reached point from the list

        msg.timestamp = int(Clock().now().nanoseconds / 1000)   # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg) 

    # Publish a trajectory setpoint in velocity
    def publish_trajectory_setpoint_vel(self,x_dot,y_dot,z_dot,yaw_dot):
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [x_dot, y_dot, z_dot]
        msg.timestamp = int(Clock().now().nanoseconds / 1000)   # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg) 


    # Publish vehicle commands
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command     # command ID
        msg.target_system = 1     # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1     # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    # Get vehicle position and velocity
    def get_vehicle_position(self, msg):
        
        # x, y, z are updated at each time step
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = msg.position[2]
        
        if(self.print_position):
            self.get_logger().info("Actual position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
            
        # vx, vy, vz are updated at each time step
        vx = msg.velocity[0]
        vy = msg.velocity[1]
        vz = msg.velocity[2]
        
        if(self.print_velocity):
            self.get_logger().info("Actual velocity: ({:.2f}, {:.2f}, {:.2f})".format(vx, vy, vz))

    # Get vehicle status (dont work anymore bug in the firmware)
    # def get_vehicle_status(self, msg):
    #     self.actual_status = msg.nav_state
        
    #     if(self.print_status):
    #         self.get_logger().info("Actual status: {:d}".format(self.actual_status))


    # Distance between a point and the current position
    def distance(self, p):
        d = np.sqrt((p.x- self.x)**2 + (p.y- self.y)**2 + (p.z- self.z)**2)
        return d
    
    # # save geometry msgs provided by mocap
    # def get_mocap_pose(self, msg):
    #     self.position_mocap = msg
    
    # # save drone position into VehicleOdometry message
    # def publish_pos_mocap(self):

    #     msg = VehicleOdometry()
    #     pose = self.position_mocap
        
    #     msg.position[0] = pose.position.x  
    #     msg.position[1] = pose.position.y  
    #     msg.position[2] = pose.position.z

    #     msg.q[0] = pose.orientation.x
    #     msg.q[1] = pose.orientation.y
    #     msg.q[2] = pose.orientation.z
    #     msg.q[3] = pose.orientation.w

    #     self.mocap_position_publisher_.publish(msg) 

    #     # Print drone coordinates from UWB
    #     print("Drone position:")
    #     print("x =", pose.position.x)
    #     print("y =", pose.position.y)
    #     print("z =", pose.position.z)

    #     print("Drone orientation:")
    #     print("x =", pose.orientation.x)
    #     print("y =", pose.orientation.y)
    #     print("z =", pose.orientation.z)
    #     print("w =", pose.orientation.w)        

# Define a class Point 
class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
