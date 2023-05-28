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
        self.vehicle_local_position_subscriber_ = self.create_subscription(VehicleLocalPosition, 
                                                                       "/fmu/out/vehicle_local_position", self.get_vehicle_position, qos_profile)
        self.vehicle_status_subscriber_ = self.create_subscription(VehicleStatus, 
                                                                       "/fmu/out/vehicle_status", self.get_vehicle_status, qos_profile)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, 
                                                                "/fmu/in/vehicle_command", qos_profile)
        

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
        p1 = Point(0.0, 0.0, -2.5)
        p2 = Point(3.0, 0.0, -2.5)
        self.point_list = [p1, p2]
        
        self.range = 0.3 # Set tolerance range to 30 cm

        self.status = 0
        self.takeoff_finished = 0
        self.landing_flag = 0 

        # UWB anchors position
        self.n_anchors = 5
        self.anchors_position = [None] * self.n_anchors
        self.anchors_range = [None] * self.n_anchors

    def timer_callback(self):

        # Arm and takeoff
        if (self.offboard_setpoint_counter_ == 0):

            # Print initial position
            self.get_logger().info("Initial position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))

            # Arm the vehicle and takeoff
            self.arm()
            self.takeoff()         
            #self.print_drone_pos_uwb()

        # Check takeoff finished
        if (self.offboard_setpoint_counter_ >= 10 and self.status == 4 and self.takeoff_finished == 0):
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
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        
        if(self.print_position):
            self.get_logger().info("Actual position: ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))
            
        vx = msg.vx
        vy = msg.vy
        vz = msg.vz
        
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
        
        # Fill vector only one time since anchors position is fixed
        if len(self.anchors_position) < self.n_anchors:

            for i in range(self.n_anchors):
                get_pos = msg.markers[i].pose.position
                self.anchors_position[i] = get_pos


    # UWB anchors distance
    def uwb_ranging(self, msg):

            for i in range(self.n_anchors):
                get_range = msg.ranging[i].range
                self.anchors_range[i] = get_range

                # print("III = ", i)
                # print("LEN DIS = ", len(self.anchors_range))
                # print("LEN POS = ", len(self.anchors_position))
                # print("RANGING = ", get_range)
     

    # Ora ho un vettore di position dove ogni elemento contiene x,y,z di ogni ancora e un altro 
    # vettore di range dove ogni elemento è la distanza del drone da quell'ancora
    # TO DO -> trilateration

    def trilateration(self):

        num_anchors = self.n_anchors
        anchors = self.anchors_position
        distances = self.anchors_range

        A = np.zeros((num_anchors - 1, 3))
        b = np.zeros((num_anchors - 1,))
        
        for i in range(1, num_anchors):
            A[i - 1] = 2 * (np.array(anchors[i]) - np.array(anchors[0]))
            b[i - 1] = distances[0]**2 - distances[i]**2 + np.linalg.norm(anchors[i])**2 - np.linalg.norm(anchors[0])**2
        
        result, residuals, rank, singular_values = np.linalg.lstsq(A, b, rcond=None)
        centroid = np.mean(np.array(anchors), axis=0)
        result += centroid
        
        return result


    def print_drone_pos_uwb(self):

        # Calcola la posizione del drone
        drone_position = self.trilateration()

        # Stampa le coordinate x, y, z del drone
        print("Posizione del drone:")
        print("x =", drone_position[0])
        print("y =", drone_position[1])
        print("z =", drone_position[2])



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
