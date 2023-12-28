import rclpy
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from rosmsgs.msg import RangingArray
from visualization_msgs.msg import MarkerArray

class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.uwb_anchors_subscriber_ = self.create_subscription(MarkerArray, 
                                                                       "/uwb_anchors", self.uwb_anchors, 1)
        self.uwb_ranging_subscriber_ = self.create_subscription(RangingArray, 
                                                                       "/uwb_ranging", self.uwb_ranging, 1)
        self.vehicle_local_position_subscriber_ = self.create_subscription(VehicleOdometry, 
                                                                       "/fmu/out/vehicle_odometry", self.get_vehicle_position, qos_profile)
        # self.vehicle_status_subscriber_ = self.create_subscription(VehicleStatus, 
        #                                                                "/fmu/out/vehicle_status", self.get_vehicle_status, qos_profile)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, 
                                                                "/fmu/in/vehicle_command", qos_profile)
        self.uwb_position_publisher_ = self.create_publisher(VehicleOdometry, 
                                                                "/fmu/in/vehicle_visual_odometry", qos_profile)
        

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
        p1 = Point(0.0, 0.0, -2.0)
        p2 = Point(1.0, 0.0, -2.0)
        self.point_list = [p1, p2]
        
        self.range = 0.3 # Set tolerance range to 30 cm
        self.height_max = p1

        self.actual_status = 0
        self.takeoff_finished = 0
        self.landing_flag = 0 

        # UWB anchors position
        self.n_anchors = 6
        self.anchors_position = [None] * self.n_anchors
        self.anchors_range = [None] * self.n_anchors
        self.anchors_id = [None] * self.n_anchors
        self.ground_flag = 0
        self.ground_position = [None]

        # Plots
        self.xgps_vec = []
        self.ygps_vec = []
        self.zgps_vec = []

        self.xuwb_vec = []
        self.yuwb_vec = []
        self.zuwb_vec = []

    def timer_callback(self):

        self.publish_pos_uwb()

        # Arm and takeoff
        if (self.offboard_setpoint_counter_ == 0):

            # Print initial position
            self.get_logger().info("Initial position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))

            # Arm the vehicle and takeoff
            self.arm()
            self.takeoff()
            
            self.multilateration()   
            
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
        if(abs(self.z) <= self.range and self.landing_flag == 1):
            self.get_logger().info("Final position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
            self.disarm()
            self.plot()
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


	# Publish the offboard control mode
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)
    

	# Publish a trajectory setpoint
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        point = self.point_list
        range = self.range

        if len(point) > 0:   # check the list is not empty
            
            msg.position = [point[0].x, point[0].y, point[0].z]
            msg.yaw = math.nan

            if self.distance(point[0]) <= range:    # point is reached
                self.get_logger().info("Position reached: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
                point.pop(0)    # delete the reached point from the list
           

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
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
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


    # Get vehicle status
    def get_vehicle_status(self, msg):
        self.status = msg.nav_state
        
        if(self.print_status):
            self.get_logger().info("Actual status: {:d}".format(self.status))


    # Distance between a point and the current position
    def distance(self, p):
        d = np.sqrt((p.x- self.x)**2 + (p.y- self.y)**2 + (p.z- self.z)**2)
        return d
    

    # UWB anchors position import from plugin
    def uwb_anchors(self, msg):
        
        self.n_anchors = len(msg.markers)
        #print("ANC_LEN = ", len(msg.markers))
        
        # Fill vector only one time since anchors position is fixed

        for i in range(self.n_anchors):
            get_X = msg.markers[i].pose.position.x
            get_Y = msg.markers[i].pose.position.y
            get_Z = msg.markers[i].pose.position.z
            self.anchors_position[i] = (get_X, get_Y, get_Z)
            self.anchors_id[i] = msg.markers[i].id

            # print("ID = ", i)
            # print("POSITION X = ", msg.markers[i].pose.position.x)
            # print("POSITION Y = ", msg.markers[i].pose.position.y)
            # print("POSITION Z = ", msg.markers[i].pose.position.z)
            # print("POSITION = ", msg.markers[i].pose.position)

            if self.anchors_id[i] == 0:
                x_ground = msg.markers[i].pose.position.x
                y_ground = msg.markers[i].pose.position.y
                z_ground = msg.markers[i].pose.position.z
                self.anchor_ground = (x_ground, y_ground, z_ground)
            
        # print("x_ground = ", self.anchor_ground[0])
        # print("y_ground = ", self.anchor_ground[1])
        # print("z_ground = ", self.anchor_ground[2])


    # UWB anchors distance
    def uwb_ranging(self, msg):
            
            self.n_anchors = len(msg.ranging)
            #print("DIS_LEN = ", len(msg.ranging))

            for i in range(self.n_anchors):
                get_range = msg.ranging[i].range
                self.anchors_range[i] = get_range/1000  # transform to meters


    def multilateration(self):

        num_anchors = self.n_anchors
        anchors_pos = self.anchors_position
        distances = self.anchors_range

        if num_anchors < 4:
            raise ValueError("Numero insufficiente di ancore per la multilaterazione.")
        
        A = np.zeros((num_anchors - 1, 3))
        b = np.zeros((num_anchors - 1))

        # Multilateration algorithm -> solve A*x = b
        # A = [ x1-x0  y1-y0  z1-z0       b = 0.5 * [ d0^2 - d1^2 + x1^2-x0^2  y1^2-y0^2  z1^2-z0^2
        #       ...                 ]                 ...                                          ]
        
        for i in range(num_anchors - 1):
            A[i, 0] = anchors_pos[i+1][0] - anchors_pos[0][0]       
            A[i, 1] = anchors_pos[i+1][1] - anchors_pos[0][1]       
            A[i, 2] = anchors_pos[i+1][2] - anchors_pos[0][2]      
            
            b[i] = 0.5*(
            (distances[0] ** 2 - distances[i+1] ** 2) +
            (anchors_pos[i+1][0] ** 2 - anchors_pos[0][0] ** 2) +
            (anchors_pos[i+1][1] ** 2 - anchors_pos[0][1] ** 2) +
            (anchors_pos[i+1][2] ** 2 - anchors_pos[0][2] ** 2))

        target_position, _residuals, _rank, _singular_values = np.linalg.lstsq(A, b, rcond=None)

        if self.ground_flag == 0:
            self.ground_position = target_position
            self.ground_flag = 1
        
        drone_position = (target_position[0] - self.ground_position[0],
                          target_position[1] - self.ground_position[1],
                          target_position[2] - self.ground_position[2])

        return drone_position


    def publish_pos_uwb(self):

        msg = VehicleOdometry()

        drone_position = self.multilateration()
        
        msg.position[0] = - drone_position[0]   #x
        msg.position[1] = - drone_position[1]   #y
        msg.position[2] = - drone_position[2]   #z

        self.uwb_position_publisher_.publish(msg) 

        # Print drone coordinates from UWB
        # print("Posizione del drone:")
        # print("x =", - drone_position[0])
        # print("y =", - drone_position[1])
        # print("z =", - drone_position[2])

        self.xgps_vec.append(self.x)
        self.ygps_vec.append(self.y)
        self.zgps_vec.append(self.z)

        self.xuwb_vec.append(drone_position[0])
        self.yuwb_vec.append(drone_position[1])
        self.zuwb_vec.append(- drone_position[2])


    def plot(self):

        plt.figure(1)
        plt.title('x position')
        plt.plot(self.xgps_vec)
        plt.plot(self.yuwb_vec)
        plt.legend(['x_gps', 'x_uwb'])

        plt.figure(2)
        plt.title('y position')
        plt.plot(self.ygps_vec)
        plt.plot(self.xuwb_vec)
        plt.legend(['y_gps', 'y_uwb'])

        plt.figure(3)
        plt.title('z position')
        plt.plot(self.zgps_vec)
        plt.plot(self.zuwb_vec)
        plt.legend(['z_gps', 'z_uwb'])

        plt.show()


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
