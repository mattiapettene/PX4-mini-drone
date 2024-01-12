import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from rosmsgs.msg import RangingArray
from visualization_msgs.msg import MarkerArray

class UWB_Bridge_SITL(Node):

    def __init__(self):
        super().__init__('UWB_Bridge_SITL')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.uwb_anchors_subscriber_            = self.create_subscription(MarkerArray,"/uwb_anchors", self.uwb_anchors, 1)
        self.uwb_ranging_subscriber_            = self.create_subscription(RangingArray,"/uwb_ranging", self.uwb_ranging, 1)
        self.uwb_position_publisher_            = self.create_publisher(VehicleOdometry,"/fmu/in/vehicle_visual_odometry", qos_profile)       
        self.vehicle_local_position_subscriber_ = self.create_subscription(VehicleOdometry,"/fmu/out/vehicle_odometry", self.get_vehicle_position, qos_profile)

        timer_period = 0.02

        self.print_position = False
        self.print_velocity = False

        # Vehicle actual position (m)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # UWB anchors position
        self.n_anchors = 6
        self.anchors_position = [None] * self.n_anchors
        self.anchors_range = [None] * self.n_anchors
        self.anchors_id = [None] * self.n_anchors
        self.ground_flag = 0
        self.ground_position = [None]

        self.timer_ = self.create_timer(timer_period, self.publish_pos_uwb)

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

    # UWB anchors position import from plugin
    def uwb_anchors(self, msg):
        
        self.n_anchors = len(msg.markers)
        
        # Fill vector only one time since anchors position is fixed
        for i in range(self.n_anchors):
            get_X = msg.markers[i].pose.position.x
            get_Y = msg.markers[i].pose.position.y
            get_Z = msg.markers[i].pose.position.z
            self.anchors_position[i] = (get_X, get_Y, get_Z)
            self.anchors_id[i] = msg.markers[i].id

            if self.anchors_id[i] == 0:
                x_ground = msg.markers[i].pose.position.x
                y_ground = msg.markers[i].pose.position.y
                z_ground = msg.markers[i].pose.position.z
                self.anchor_ground = (x_ground, y_ground, z_ground)

    # UWB anchors distance
    def uwb_ranging(self, msg):
            
        self.n_anchors = len(msg.ranging)

        for i in range(self.n_anchors):
            get_range = msg.ranging[i].range
            self.anchors_range[i] = get_range/1000


    def multilateration(self):

        num_anchors = self.n_anchors
        anchors_pos = self.anchors_position
        distances = self.anchors_range

        if num_anchors < 4:
            raise ValueError("Anchors number not sufficient!")
        
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
        
        msg.pose_frame = msg.POSE_FRAME_NED
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp_sample = msg.timestamp

        # Adapt UWB position to drone reference frame
        msg.position[0] = drone_position[1]     #x
        msg.position[1] = drone_position[0]     #y
        msg.position[2] = - drone_position[2]   #z

        self.uwb_position_publisher_.publish(msg) 

        # Print drone coordinates from UWB
        print("Posizione del drone:")
        print("x =", - drone_position[0])
        print("y =", - drone_position[1])
        print("z =", - drone_position[2])

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    uwb_Bridge_sitl = UWB_Bridge_SITL()
    rclpy.spin(uwb_Bridge_sitl)
    # Destroy the node explicitly
    uwb_Bridge_sitl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
