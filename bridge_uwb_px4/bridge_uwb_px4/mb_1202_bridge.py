import serial
from smbus import SMBus
import time

import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleOdometry

class Mb1202PX4Bridge(Node):

    def __init__(self):
        super().__init__('Mb1202PX4Bridge')

        # definiton of the quality of service
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # create publisher to vehicle visual odometry topic
        self.vehicle_pose_publisher_ = self.create_publisher(VehicleOdometry,"/fmu/in/vehicle_visual_odometry",qos_profile)    
        self.mb1202_pose_publisher_ = self.create_publisher(VehicleOdometry,"/Drone/mb1202",qos_profile)    


        # batch of data for moving average filter (collect a batch of data of 50 elements)
        self.batch_mb1202 = []

        # sensor I2C address
        self.address = 0x70

        # timer period callback
        timer_period = 0.1  # 100 milliseconds

        # initial position
        self.pos_0 = [0.,0.,0.]

        # flag to print uwb estimated position
        self.flag_print = False
        self.flag_file = True

        # timer callback
        self.timer_ = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):

        # read mb1202 and obtain z coordinate
        z_coord_mb1202 = self.lettura_mb1202(self.address)

        # outlier rejection
        if len(self.batch_mb1202) != 0:
            z_coord_mb1202_rj = self.reject_outliers(z_coord_mb1202,self.batch_mb1202)
        else:
            z_coord_mb1202_rj = z_coord_mb1202

        # collect batch of data
        if(not(np.isnan(z_coord_mb1202_rj))):
            self.batch_mb1202.append(z_coord_mb1202_rj)

        if(len(self.batch_mb1202) > 50):
            self.batch_mb1202.pop(0)

        # publish vehicle visual odometry topic
        self.publish_vehicle_visual_odometry([math.nan, math.nan, z_coord_mb1202_rj])

    # ------ FUNCTIONS -------           

    def lettura_mb1202(self,address):
        try:
            i2cbus = SMBus(1)
            i2cbus.write_byte(address, 0x51)
            time.sleep(0.1)  #100ms
            val = i2cbus.read_word_data(address, 0xe1)

            z = (val >> 8) & 0xff | (val & 0xff)

            if z!=25:
                return z * 0.01
            else:
                return math.nan
    
        except IOError as e:
            print(e)
            return math.nan
        
    def reject_outliers(value, data, m = 3.):
        mva = sum(data)/len(data)
        std = []
        for i in range(len(data)):
            std.append(np.sqrt(sum((data[i] - mva)**2)/len(data)))
        z_score = abs((value - mva)/std)
        if(z_score < m):
            return value
        else:
            return math.nan

    def publish_vehicle_visual_odometry(self, pos = [math.nan,math.nan,math.nan], quater = [math.nan,math.nan,math.nan,math.nan]):
        
        msg = VehicleOdometry()

        msg.pose_frame = msg.POSE_FRAME_NED

        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp_sample = msg.timestamp

        # Convert uwb reference frame into PX4 reference frame
        
        msg.position[0] = pos[0]
        msg.position[1] = pos[1]
        msg.position[2] = pos[2]

        msg.q[0] = quater[0]
        msg.q[1] = quater[1]
        msg.q[2] = quater[2]
        msg.q[3] = quater[3]

        if (self.flag_print):
            self.get_logger().info("Actual position: ({:.2f}, {:.2f}, {:.2f})".format(msg.position[0], msg.position[1], msg.position[2]))      

        self.vehicle_pose_publisher_.publish(msg)
        self.mb1202_pose_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting uwb bridge node...\n")
    mb1202_bridge = Mb1202PX4Bridge()
    rclpy.spin(mb1202_bridge)
    # Destroy the node explicitly
    mb1202_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
