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

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

class DTOA:
    '''
        class to calcculate the position of the drone using the signal given by
        the uwb module
    '''
    def __init__(self):

        # initialize infrastructure
        A_n1 = [0.083, 2.046, -0.074]
        A_n2 = [1.725, 7.830, 0.040]
        A_n3 = [4.730, 9.571, 0.005]
        A_n4 = [5.764, 4.277, 0.103]
        A_n5 = [5.584, 0.098, 0.111]
        A_n6 = [0.0, 0.0, 0.0]
        self.A_n = np.array((A_n6, A_n1, A_n2, A_n3, A_n4, A_n5))
        self.n = len(self.A_n)

        # number of mesurements
        self.n_mes = 1

        # Speed of light
        self.c = 299792458

    def TDoA(self, ts, skew):

        '''
            ts: signal coming from uwb antennas
            skew: drift in the measurement
        '''

        t6_rx1 = float(int(ts[0],2)) * 15.65e-12
        t1_rx1 = float(int(ts[1],2)) * 15.65e-12
        t2_rx1 = float(int(ts[2],2)) * 15.65e-12
        t3_rx1 = float(int(ts[3],2)) * 15.65e-12
        t4_rx1 = float(int(ts[4],2)) * 15.65e-12
        t5_rx1 = float(int(ts[5],2)) * 15.65e-12

        t6_rx2 = float(int(ts[6],2)) * 15.65e-12
        t1_rx2 = float(int(ts[7],2)) * 15.65e-12
        t2_rx2 = float(int(ts[8],2)) * 15.65e-12
        t3_rx2 = float(int(ts[9],2)) * 15.65e-12
        t4_rx2 = float(int(ts[10],2)) * 15.65e-12
        t5_rx2 = float(int(ts[11],2)) * 15.65e-12

        t6_tx1 = float(int(ts[12],2)) * 15.65e-12
        t1_tx1 = float(int(ts[13],2)) * 15.65e-12
        t2_tx1 = float(int(ts[14],2)) * 15.65e-12
        t3_tx1 = float(int(ts[15],2)) * 15.65e-12
        t4_tx1 = float(int(ts[16],2)) * 15.65e-12
        t5_tx1 = float(int(ts[17],2)) * 15.65e-12

        t6_tx2 = float(int(ts[18],2)) * 15.65e-12
        t1_tx2 = float(int(ts[19],2)) * 15.65e-12
        t2_tx2 = float(int(ts[20],2)) * 15.65e-12
        t3_tx2 = float(int(ts[21],2)) * 15.65e-12
        t4_tx2 = float(int(ts[22],2)) * 15.65e-12
        t5_tx2 = float(int(ts[23],2)) * 15.65e-12

        # Real measurements
        toa_tx = np.array([[t6_tx1,t6_tx2],[t1_tx1,t1_tx2],[t2_tx1,t2_tx2],[t3_tx1,t3_tx2],[t4_tx1,t4_tx2],[t5_tx1,t5_tx2]])
        toa_rx = np.array([[t6_rx1,t6_rx2],[t1_rx1,t1_rx2],[t2_rx1,t2_rx2],[t3_rx1,t3_rx2],[t4_rx1,t4_rx2],[t5_rx1,t5_rx2]])

        # Drift calculation (recursive mean method)
        skew_actual = (toa_rx[:,1]- toa_rx[:,0]) /(toa_tx[:,1] - toa_tx[:,0])
        skew_mean = skew + (1/self.n_mes * (skew_actual - skew))
        
        tmp_rx = np.zeros((self.n,2))
        tmp_rx[:,0] = toa_rx[:,0] - toa_rx[0,0] - (toa_tx[:,0] * skew_mean - toa_tx[0,0] * skew_mean[0])
        tmp_rx[:,1] = toa_rx[:,1] - toa_rx[0,1] - (toa_tx[:,1] * skew_mean - toa_tx[0,1] * skew_mean[0])
        
        # TDoA
        tdoa = np.zeros((self.n,2))
        tdoa = tmp_rx[:,1]
        tdoa = np.delete(tdoa,[0])

        D = self.c*tdoa
        
        # Multilateration
        #------Trilateration linear equations system-------------------
        A_ref = self.A_n[0]
        A = 2 * np.array((A_ref[0] - self.A_n[1:self.n, 0], A_ref[1] - self.A_n[1:self.n, 1], A_ref[2] - self.A_n[1:self.n, 2], D)).T
        b = D**2 + np.linalg.norm(A_ref)**2 - np.sum(np.square(self.A_n[1:self.n, :]), axis=1)
        x_t0 = np.matmul(np.linalg.pinv(A), b.T)
        
        #-----Non linear correction (Taylor Expansion)-----------------
        x_t_0 = np.array((x_t0[0], x_t0[1], x_t0[2]))
        f = np.zeros((self.n-1,1))
        del_f = np.zeros((self.n-1,3))
       
        for ii in range(1,self.n) :
            f[ii-1]= np.linalg.norm((x_t_0 - self.A_n[ii,:]), ord=2)-np.linalg.norm((x_t_0 - self.A_n[0,:]), ord = 2)
            del_f[ii-1,0] = (x_t_0[0] - self.A_n[ii,0])*np.linalg.norm((x_t_0 - self.A_n[ii,:]),ord=2)**-1 - (x_t_0[0]-self.A_n[0,0])*np.linalg.norm((x_t_0-self.A_n[0,:]), ord=2)**-1
            del_f[ii-1,1] = (x_t_0[1] - self.A_n[ii,1])*np.linalg.norm((x_t_0 - self.A_n[ii,:]),ord=2)**-1 - (x_t_0[1]-self.A_n[0,1])*np.linalg.norm((x_t_0-self.A_n[0,:]), ord=2)**-1
            del_f[ii-1,2] = (x_t_0[2] - self.A_n[ii,2])*np.linalg.norm((x_t_0 - self.A_n[ii,:]),ord=2)**-1 - (x_t_0[2]-self.A_n[0,2])*np.linalg.norm((x_t_0-self.A_n[0,:]), ord=2)**-1 
     
        Pos = (np.matmul(np.linalg.pinv(del_f), (D- f.T).T)).T + x_t_0

        # update number of mesurements
        self.n_mes = self.n_mes + 1

        return Pos[0,0], Pos[0,1], Pos[0,2], skew_mean

class UwbPX4Bridge(Node):

    def __init__(self):
        super().__init__('UwbPX4Bridge')

        # definiton of the quality of service
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # create publisher to vehicle visual odometry topic
        self.vehicle_pose_publisher_ = self.create_publisher(VehicleOdometry,"/fmu/in/vehicle_visual_odometry",qos_profile)
        self.uwb_pose_publisher_ = self.create_publisher(VehicleOdometry,"/Drone/uwb_pose",qos_profile)

        # batch of data for moving average filter (collect a batch of data of 50 elements)
        self.batch_uwb = []

        # uwb module address
        self.mesg = {}
        self.ser = serial.Serial('/dev/ttyACM0',921600,timeout=0)
        self.rl = ReadLine(self.ser)

        # timer period callback
        timer_period = 0.1  # 100 milliseconds

        # initialize dtoa algorithm
        self.skew = 0
        self.tdoa = DTOA()

        # initial position
        self.pos_0 = [0.,0.,0.]

        # flag to print uwb estimated position
        self.flag_print = False
        self.flag_file = False

        # Save data to file
        self.f = open("data_uwb.csv", "w")
        self.f.write('x_uwb,y_uwb')

        # timer callback
        self.timer_ = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):

        # read uwb module and get string data
        times_uwb = self.lettura_uwb(self.rl)
        print(len(times_uwb))

        # calculate x,y,z coordinate and skew using tdoa algorithm
        [x_coord_uwb,y_coord_uwb,z_coord_uwb,self.skew] = self.tdoa.TDoA(times_uwb,self.skew)
        print(self.skew)
        # data_uwb = [x_coord_uwb,y_coord_uwb]

        # [x_coord_uwb_rj, y_coord_uwb_rj] = self.data_outlier_rejection(data_uwb)
        
        # # collect batch of data
        # if(not(np.isnan(x_coord_uwb_rj)) and not(np.isnan(y_coord_uwb_rj))):
        #     self.batch_uwb.append([x_coord_uwb_rj,y_coord_uwb_rj])
        
        # if(len(self.batch_uwb) > 50):
        #     self.batch_uwb.pop(0)

        # # outlier rejection
        # if(self.flag_file):
        #     self.f.write('{0},{1},{2},{3}\n'.format(x_coord_uwb_rj, y_coord_uwb_rj))
        
        # # position and quaternion
        # position = [x_coord_uwb_rj,y_coord_uwb_rj,math.nan]

        # # publish vehicle visual odometry topic
        # self.publish_vehicle_visual_odometry(position)

        # self.offboard_setpoint_counter_ += 1

    # ------ FUNCTIONS -------           

    def lettura_uwb(self,rl):
        
        try:
            # read the line and decode 
            mesg = rl.readline().decode("utf-8")
            mesg = mesg.replace("\r\n", "")
            # split the string
            times = mesg.split(" ")
        except KeyboardInterrupt:
            self.ser.close()
            print ('closed serial port')

        return times
    
    def data_outlier_rejection(self,data_uwb):

        # - check on the skew term
        if(np.mean(self.skew) < 0.9 or np.mean(self.skew) > 1.1):
            x_coord_uwb = math.nan
            y_coord_uwb = math.nan
        else:
            # - check on the uwb coordinate
            data_tmp = np.array(data_uwb)
            batch_tmp = np.array(self.batch_uwb)
            uwb_coord = np.zeros(2)
            for i in range(2):
                uwb_coord[i] = self.reject_outliers(data_tmp[i],batch_tmp[:,i])
            x_coord_uwb = uwb_coord[0]
            y_coord_uwb = uwb_coord[1]
    
        return x_coord_uwb, y_coord_uwb 
    
    def reject_outliers(value, data, m = 3.):
        data_tmp = np.array(data)
        mva = sum(data_tmp)/len(data_tmp)
        std = np.sqrt(sum((data_tmp - mva)**2)/len(data_tmp))
        z_score = abs((value - mva)/std)
        if(z_score < m):
            return value
        else:
            return math.nan

    def publish_vehicle_visual_odometry(self, pos = math.nan, quater = math.nan):
        
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
        self.uwb_pose_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting uwb bridge node...\n")
    uwb_bridge = UwbPX4Bridge()
    rclpy.spin(uwb_bridge)
    # Destroy the node explicitly
    uwb_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
