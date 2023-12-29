import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped

class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_status_subscriber_ = self.create_subscription(VehicleStatus, 
                                                                    "/fmu/out/vehicle_status", self.get_vehicle_status, qos_profile)
        self.vehicle_odometry_position_subscriber_ = self.create_subscription(VehicleOdometry, 
                                                                    "/fmu/out/vehicle_odometry", self.get_vehicle_position, qos_profile)
        self.vehicle_mb1202_position_subscriber_ = self.create_subscription(VehicleOdometry, 
                                                                    "/Drone/mb1202", self.get_mb1202_position, qos_profile)
        self.vehicle_uwb_position_subscriber_ = self.create_subscription(VehicleOdometry, 
                                                                    "/Drone/uwb_pose", self.get_uwb_position, qos_profile)
        self.vehicle_mocap_position_subscriber_ = self.create_subscription(VehicleOdometry, 
                                                                    "/Drone/mocap_pose", self.get_mocap_position, qos_profile)
        
        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.print_position = False
        self.print_velocity = False
        self.print_status = False 

        # Vehicle actual position (m)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.actual_status = 0

        # Print values to file

        self.x_mocap = 0.0
        self.y_mocap = 0.0
        self.z_mocap = 0.0

        self.x_uwb = 0.0
        self.y_uwb = 0.0

        self.z_mb1202 = 0.0

        self.f = open("dati_read_sensor.csv", "w")
        self.f.write("x_est,y_est,z_est,x_mocap, y_mocap, z_mocap, x_uwb, y_uwb, z_mb1202\n")

    def timer_callback(self):

        self.print_to_file()


    # ------ FUNCTIONS -------                                                                    

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
        self.actual_status = msg.nav_state
        
        if(self.print_status):
            self.get_logger().info("Actual status: {:d}".format(self.actual_status))


    # Distance between a point and the current position
    def distance(self, p):
        d = np.sqrt((p.x- self.x)**2 + (p.y- self.y)**2 + (p.z- self.z)**2)
        return d
    
    # Get mocap, uwb and mb1202 positions

    def get_mocap_position(self, msg):
        if(msg):
            self.x_mocap = msg.position[0]
            self.y_mocap = msg.position[1]
            self.z_mocap = msg.position[2]

    def get_uwb_position(self, msg):
        if(msg):
            self.x_uwb = msg.position[0]
            self.y_uwb = msg.position[1]

    def get_mb1202_position(self, msg):
       if(msg):
            self.z_mb1202 = msg.position[2]

    # Print to file
    def print_to_file(self):
        self.f.write("{0},{1},{2},{3},{4},{5},{6},{7},{8}\n".format(self.x,self.y,self.z,self.x_mocap, self.y_mocap, self.z_mocap, self.x_uwb, self.y_uwb, self.z_mb1202))

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
