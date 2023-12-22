import time
import zmq
from blimp_class import *
import numpy as np
import math
from numpy.linalg import norm, inv
from quaternion import Quaternion
from sub_module import *
import global_
import logging
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
 
# Definizione context per zmq
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.1.104:5556")
print("Mi sono connesso al Raspberry pi Blimp")

# Subscribe to zipcode, default is Blimp
zip_code = "Blimp"
socket.setsockopt_string(zmq.SUBSCRIBE, zip_code)

def str_to_float():
            message2 = socket.recv_string()
            zip_code, tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, mesg, zpos = message2.split(",")
            tempo = float(tempo)
            raw_accX = float(raw_accX)
            raw_accY = float(raw_accY)
            raw_accZ = float(raw_accZ)
            raw_gyrX = float(raw_gyrX)
            raw_gyrY = float(raw_gyrY)
            raw_gyrZ = float(raw_gyrZ)
            raw_magX = float(raw_magX)
            raw_magY = float(raw_magY)
            raw_magZ = float(raw_magZ)
            zpos = float(zpos)
            return tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg

def blimp_to_world_rf():   
    # q0 = Quaternion(0, 0, 0, 1)
    q0 = Quaternion(0.7068252, 0, 0, 0.7073883)
    mad = Madgwick(sampleperiod = 0.09, quaternion=q0, beta=0.1)
    psi_list = []
    x_list = []
    y_list = []
    z_list = []

    # per UWB
    tempi_dt = np.zeros((1,6))

    try:
        
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, z_pos, mesg0 = str_to_float()
        ts = mesg0.split(" ")
        if (len(ts)!=25):
          tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, z_pos, mesg0 = str_to_float()
          ts = mesg0.split(" ")
        # --------------- 
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        tempi_dt[0,:] = dt_uwb

        time_zero_mad = tempo
        time_zero_meas = time.perf_counter()
        time_zero_UWB = time.perf_counter()

        print("Allinea il blimp con il sistema di riferimento assoluto per 20 secondi") 
        while  time.perf_counter() < (time_zero_meas + 15):

            # Misuro con UWB la posizione nel piano  frattempo 
            tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, z_pos, mesg = str_to_float()
            ts = mesg.split(" ")
            if (len(ts)!=25):
                tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, z_pos, mesg = str_to_float()
                ts = mesg.split(" ")

            time_current_UWB = time.perf_counter()
            if (time_current_UWB - time_zero_UWB) >= 0.09:

                raw_acc = [raw_accX, raw_accY, raw_accZ]
                raw_gyr = [raw_gyrX, raw_gyrY, raw_gyrZ] 
                raw_mag = [raw_magX, raw_magY, raw_magZ]
                
                
                # Aquisizione magnetometro e calibrazione dei dati:
                mag = calibration(raw_mag)

                raw_acc, raw_gyr, mag = imu_to_uwb(raw_acc, raw_gyr, mag)

                # Creazione vettore input per classe madgwick
                accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
            
                # setting the variable frequency of update of Madgwick alghorithm
                mad.samplePeriod = tempo - time_zero_mad
                quaternion = mad.update(gyroscope, accelerometer, magnetometer)
                time_zero_mad = tempo
                quat = Quaternion(quaternion)
                
                roll, pitch, yaw = quat.to_euler123()  # Result is in rad
                
                psi = yaw*180/np.pi
                if psi < 0:
                    psi = psi + 360
                psi_mapped = psi_map(psi)
                psi_list.append(psi_mapped) # deg

                if len(psi_list) > 5:
                    psi_list.pop(0)

                psi_mean = sum(psi_list)/len(psi_list)
                
                x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
                dt_new = np.reshape(dt_new, (1,6))
                tempi_dt = np.append(tempi_dt,dt_new, axis=0)

                if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0 and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                    x_list.append(x_pos_uwb)
                    y_list.append(y_pos_uwb)
                    z_list.append(z_pos)
                    print(x_pos_uwb,y_pos_uwb,z_pos,psi,psi_mean)

                if len(x_list) > 10:
                    x_list.pop(0)

                if len(y_list) > 10:
                    y_list.pop(0)

                if len(z_list) > 20:
                    z_list.pop(0)
              
                for i in range(6) :
                    dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=2))

                time_zero_UWB = time.perf_counter()
    
    except KeyboardInterrupt:
        
        print ('Serial port closed')

    quat_final = quat # the last quaternion is stored as input for the madgwick later
    psi_0 = sum(psi_list)/len(psi_list) # deg

    x_0 = sum(x_list)/len(x_list) # perform the mean of the last 20 element
    y_0 = sum(y_list)/len(y_list)
    z_0 = sum(z_list)/len(z_list)
    return psi_0, quat_final, x_0, y_0, z_0 # These are the values of initial angles
    # quat_final is the initial input for Madgwick relaunch

def main():

    # get current date and time
    current_datetime = datetime.now().strftime("%Y-%m-%d %H-%M-%S")
    print("Current date & time : ", current_datetime)
    
    # convert datetime obj to string
    str_current_datetime = str(current_datetime)
    
    # create a file object along with extension
    file_name_f = "Blimp_data/log_Blimp_data_"+str_current_datetime+".txt"
    file_name_g = "Blimp_navigation/log_Blimp_navigation_"+str_current_datetime+".txt"
    
    f = open(file_name_f, "w") # Here Timestep, IMU, UWB and SONAR data are stored
    g = open(file_name_g, "w") # timestep, l_pwm, r_pwm, z_pww, distance, psi and psi_target

    # ORIENTAZIONE NEL GLOBAL REFERENCE FRAME
    psi_0, quat_final, x_0, y_0, z_0 = blimp_to_world_rf()
    
    # Printing the final results
    print("psi_0 del blimp per il Madgwick= ", psi_0) # deg
    
    # Goal position
    x_goal = 3
    y_goal = 2
    
    # inizializzo x_pos e y_pos dell'UWB
    x_pos = x_0
    y_pos = y_0
    z_pos = z_0
    
    # creo liste per posizioni x, y e z
    x_list = [x_pos]
    y_list = [y_pos]
    z_list = []
    psi_list = []

    x_counter = 0
    y_counter = 0
    z_counter = 0

     # per UWB
    tempi_dt = np.zeros((1,6))
    # Ricezione dati da ZeroMQ
    tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, z_pos, mesg0 = str_to_float()

    ts = mesg0.split(" ")
    if (len(ts)!=25):
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, z_pos, mesg0 = str_to_float()
        ts = mesg0.split(" ")
    dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
    tempi_dt[0,:] = dt_uwb

    # Madgwick initialization
    q0 = quat_final
    #q0 = Quaternion(0.7068252, 0, 0, 0.7073883)
    mad = Madgwick(sampleperiod = 0.09, quaternion=q0, beta=0.2) 
    
    dist_target = 0.0 # m distance the blimp want to minimize
    z_target = 1.2  # Mantenimento di una data quota
    
    p = 1
    i = 0.01
    d = 0.1
    # PID initialization
    dist_pid = PID_Controller(kp= p*0.0008, ki = i*0.0008, kd = d*0.0008, max_signal =100000, sample_rate=1.0, target=dist_target)
    psi_pid = PID_Controller(kp= p*0.00001, ki = i*0.00001, kd = d*0.00001, max_signal =100000, sample_rate=1.0, target=0.0)
    #z_pid = PID_Controller(kp= 0.1, ki = 0.001, kd = 0.001, max_signal =100000, sample_rate=1.0, target=z_target)

    #dist_pid = PID_Controller(kp= 0.0, ki = 0.000, kd = 0.00, max_signal =100000, sample_rate=0.1, target=dist_target)
    #psi_pid = PID_Controller(kp= 0.00, ki = 0.0000, kd = 0.000, max_signal =100000, sample_rate=0.1, target=0.0)
    z_pid = PID_Controller(kp= p*0.07, ki = i*0.07, kd = d*0.07, max_signal =100000, sample_rate=0.1, target=z_target)

    z_pid.set_new_target(z_target)
    dist_pid.set_new_target(dist_target)

    #accel = []
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)

    #Contatori
    time_zero_mad = tempo
    
    time_zero_UWB = time.perf_counter()
    time_zero_1Hz = time.perf_counter()
    while True: 
        # Misuro con UWB la posizione nel piano  frattempo 
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, z_pos, mesg = str_to_float()
        ts = mesg.split(" ")
        if (len(ts)!=25):
            tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, z_pos, mesg = str_to_float()
            ts = mesg.split(" ")  

        time_current_UWB = time.perf_counter()
        if (time_current_UWB - time_zero_UWB) >= 0.067:

            raw_acc = [raw_accX, raw_accY, raw_accZ]
            raw_gyr = [raw_gyrX, raw_gyrY, raw_gyrZ] 
            raw_mag = [raw_magX, raw_magY, raw_magZ]
            
            # Aquisizione magnetometro e calibrazione dei dati:
            # hard and soft iron calibration of magnetometer
            mag = calibration(raw_mag)
            # rotate the IMU reference frame with the UWB reference fram
            raw_acc, raw_gyr, mag = imu_to_uwb(raw_acc, raw_gyr, mag) 

            # Creazione vettore input per classe madgwick
            accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
            # setting the variable frequency of update of Madgwick alghorithm
            mad.samplePeriod = tempo - time_zero_mad
            quaternion = mad.update(gyroscope, accelerometer, magnetometer)
            time_zero_mad = tempo
            quat = Quaternion(quaternion)
            
            roll, pitch, yaw = quat.to_euler123()  # Result is in rad
            # Qua ruoto le accelerazioni per portarle correttamente nel modello del kalman filter
            #angle_1 = [roll*180/np.pi, pitch*180/np.pi, yaw*180/np.pi]
            
            #acc_UWB, gyro_UWB = rotation_UWB(raw_acc, raw_gyr, angle_1)
            #print((acc_UWB[0,0],acc_UWB[0,1],acc_UWB[0,2]))
            #accel.append((acc_UWB[0,0],acc_UWB[0,1],acc_UWB[0,2]))
            #print(accel)
            #accel_mean = np.mean(accel, axis=0)
            #acc_abs = acc_UWB - accel_mean#- np.array([[0, 0, 10.07]])
            # acc_abs is the argument for the Kalman

            yaw_deg = yaw*180/np.pi # deg
            
            # Put the yaw into 0-360 degree range and then map it
            if yaw_deg < 0:
                yaw_deg = yaw_deg + 360
            psi_mapped = psi_map(yaw_deg) # deg
            
            # Remove the offset to allign with the UWB Reference frame (the room)
            psi = psi_mapped - psi_0 # deg
            if psi < 0: 
                psi = psi + 360
            elif psi > 360:
                psi = psi - 360  
           
            # Mean of the psi value to estimate the yaw more accurately
            psi_list.append(psi)

            if len(psi_list) > 3:
                psi_list.pop(0)

            x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
            
            dt_new = np.reshape(dt_new, (1,6))
            tempi_dt = np.append(tempi_dt,dt_new, axis=0)
            for i in range(6) :
                dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=5))
            
            if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0  and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                x_pos = x_pos_uwb
                y_pos = y_pos_uwb 
            
            if len(x_list) < 5:
                # last 5 x, y, z and psi measurements again a mean is performed to increase the accuracy of the measure
                x_list.append(x_pos)
                y_list.append(y_pos)
            else:
                x_list, flag_x = pos_mean(x_list, x_pos)
                y_list, flag_y = pos_mean(y_list, y_pos)
                
                if flag_x == 1:
                    x_counter = x_counter + 1
                else:
                    x_counter = 0

                if flag_y == 1:
                    y_counter = y_counter + 1
                else:
                    y_counter = 0
            
            if x_counter >= 3:
                x_list.append(x_pos)
                x_list.pop(0)
            
            if y_counter >= 3:
                y_list.append(y_pos)
                y_list.pop(0)
                
            if len(z_list) < 10:
                # last 5 x, y, z and psi measurements again a mean is performed to increase the accuracy of the measure
                z_list.append(z_pos)
            else:
                z_list, flag_z = pos_mean(z_list, z_pos)
                
                if flag_z == 1:
                    z_counter = z_counter + 1
                else:
                    z_counter = 0
            
            if z_counter >= 10:
                z_list.append(z_pos)
                z_list.pop(0)

            # Scrivere i dati  
            stringa_imu = [tempo, raw_acc[0], raw_acc[1], raw_acc[2], raw_gyr[0], raw_gyr[1], raw_gyr[2], mag[0], mag[1], mag[2], x_pos, y_pos, z_pos, psi ]
            res = " ".join([str(j) for j in stringa_imu])
            f.write(res)
            f.write("\n")

            time_zero_UWB = time.perf_counter()
            if (time.perf_counter() - time_zero_1Hz) >= 0.1:

                x_pos = sum(x_list) / len(x_list)
                y_pos = sum(y_list) / len(y_list)
                z_pos = sum(z_list) / len(z_list)
                psi = psi_mean(psi_list,psi)

                # print("State = ", x_pos, y_pos, z_pos, psi)
                # distance from the target calculation
                goal_dist = math.sqrt((x_pos-x_goal)**2 + (y_pos - y_goal)**2)
                
                # psi target calculation
                psi_target = (math.atan2((y_goal-y_pos), (x_goal-x_pos)))*180/np.pi # deg
                if psi_target < 0:
                    psi_target = psi_target + 360
               
                # Questo blocco serve nel caso lo psi target sia vicino allo 0
                psi_target_new = psi_target
                if 0 < psi_target and psi_target <= 180 and 180 < psi and psi <= 360:
                    psi_target_new = psi_target + 360
                if 180 < psi_target and psi_target <= 360 and 0 < psi and psi <= 180:
                    psi_target_new = psi_target - 360
                psi_target = psi_target_new # deg

                # Representation of actual blimp orientation
                x_o = x_pos + 0.3*math.cos(psi*(np.pi/180))
                y_o = y_pos + 0.3*math.sin(psi*(np.pi/180))

                # Representation of yaw target
                x_t = x_pos + 0.3*math.cos(psi_target*(np.pi/180))
                y_t = y_pos + 0.3*math.sin(psi_target*(np.pi/180))

                ax.clear()
                ax.set_aspect(1)
                ax.set(xlim=(0, 6), ylim=(0, 12))
                ax.scatter(x_pos, y_pos, c = 'b')
                ax.plot([x_pos,x_o], [y_pos,y_o], linestyle = '-', c = 'b')
                ax.plot([x_pos,x_t], [y_pos,y_t], linestyle = '-', c = 'r')
                ax.scatter(x_goal, y_goal, marker = 'x', c = 'r')
                
                plt.pause(0.05)

                ####################################################
                # SET INITIAL TARGET FOR PID OBJECT CLASS
                #z_pid.set_new_target(z_target)
                psi_pid.set_new_target(psi_target)
                #dist_pid.set_new_target(dist_target)
                #####################################################

                ##############################################
                # Z_PID block of code 
                ##############################################

                print ("z_pos = ", z_pos)

                signal_z = z_pid.adjust_signal(z_pos)
                force_z = z_pid.get_force_z(signal_z) # Questa serve per il Kalman

                z_pwm = z_pid.pwm_z_motor(force_z) # Questa serve per i motori

                #print("z_pos, signal_z e z_pwm = ", z_pos, signal_z, z_pwm)
            
                # Per variare il sample rate del PID controller durante la run
                signal_psi = psi_pid.adjust_signal(psi)
                signal_dist = dist_pid.adjust_signal(-goal_dist)
                
                if goal_dist > 0.25:
                    if abs(psi - psi_target) > 15:
                        force_l, force_r = dist_pid.get_force_lateral(0, signal_psi) # Questa serve per il Kalman
                    else:
                        force_l, force_r = dist_pid.get_force_lateral(signal_dist, 0) # Questa serve per il Kalman
                        
                    l_pwm = dist_pid.pwm_L_motor(force_l) # Questa serve per i motori

                    r_pwm = dist_pid.pwm_R_motor(force_r) # Questa serve per i motori
                    
                else:
                    print("Target reached: motors off")
                    l_pwm = 0
                    r_pwm = 0
                    
                # Cambia il valore delle PWM globali e mandale al blimp
                if z_pwm < 0:
                    global_.pwm_up = 0
                else:
                    global_.pwm_up = z_pwm

                global_.pwm_right = r_pwm
                global_.pwm_left = l_pwm

                stringa_g = [tempo, l_pwm, r_pwm, z_pwm, goal_dist, psi_target, z_target]
                res_g = " ".join([str(j) for j in stringa_g])
                g.write(res_g)
                g.write("\n")

                pwm_func()
                
                #print("dist, psi e psi_target = ", dist_dist, psi*180/np.pi, psi_target*180/np.pi)
                #print("psi, psi_target e goal distance = ", psi, psi_target, goal_dist)
                #print("l_pwm, r_pwm e z_pwm = ", l_pwm, r_pwm, z_pwm)
                #print("z_pos, z_target e z_pwm = ", z_pos, z_target, z_pwm)
                #print("Accelerazioni ruotate", acc_abs)
                #print(z_pos)

                time_zero_1Hz = time.perf_counter() 
    plt.show() 
                  

if __name__ == "__main__":
    # get current date and time
    current_datetime = datetime.now().strftime("%Y-%m-%d %H-%M-%S")
    print("Current date & time : ", current_datetime)
    
    # convert datetime obj to string
    str_current_datetime = str(current_datetime)
    
    # create a file object along with extension
    file_name_f = "log_Blimp_data_"+str_current_datetime+".txt"
    file_name_g = "log_Blimp_navigation_"+str_current_datetime+".txt"
    
    f = open(file_name_f, "w") # Here Timestep, IMU, UWB and SONAR data are stored
    g = open(file_name_g, "w") # timestep, l_pwm, r_pwm, z_pww, distance, psi and psi_target
   
    # 1. Madgwick

    # 2. Kalman

    '''
    Ricevi gli aggiornamenti e fai i calcoli il più velocemente possibile. 
    Poi aggiorna le forze una volta al secondo nel modello del kalman
    e anche veramente nel modello del blimp. 
    Salva uno storico di log delle info dei sensori e dello stato. 
    '''
    try:    
        main()  

    except TypeError: 
        print("Crashed :/")
       
        print ('Serial port closed')
        print("Motors stopped") 
        f.close()
        g.close()
        logging.exception("message")

    except ValueError: 
        print("Crashed :/") 
        f.close()
        g.close()
        logging.exception("message")

    except KeyboardInterrupt:
        print("Interrupted")
        f.close()
        g.close()
        logging.exception("message")
        
       
        
