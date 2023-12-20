# Rtdoa UWB
import numpy as np
# from numpy.linalg import norm
# from quaternion import Quaternion
# import math
# from math import dist, cos, sin

def trilateration(d1,d2,d3,d4,d5,d6):

    """
        Trilateration function to estimate position of the uwb given distances
        Consider 6 uwb range signal
    """

    # Anchor definition, set their position in space
    A_n1 = np.array([0.00, 7.19, 2.15])
    A_n2 = np.array([0.00, 3.62, 3.15])
    A_n3 = np.array([0.00, 0.00, 2.15])
    A_n4 = np.array([4.79, 1.85, 3.15])
    A_n5 = np.array([4.79, 5.45, 2.15])
    A_n6  = np.array([3.00, 9.35, 3.15])

    An =  A_n1, A_n2, A_n3, A_n4, A_n5, A_n6
    An = np.array(An)  

    d = np.array([ d1, d2, d3, d4, d5, d6])

    # Compute the position of T using trilateration and LS formula
    # Defining initial A matrix and b vector
    A = np.zeros((5,3))
    b = np.zeros((5,1))

    # Definition of vectors X, Y, Z, first position is the target,
    # the others the anchors coordinates
    x = An[:,0]
    y = An[:,1]
    z = An[:,2]

    # Calculation of A and b for the case with 6 anchors
    for c in range(1, len(x)):
        A[c-1, :] = [x[c]-x[0], y[c]-y[0], z[c]-z[0]] 
        #A[c-1, :] = [x[c]-x[0], y[c]-y[0], z[c]-z[0], 0, 0, 0]  
        b[c-1] = d[0]**2 - d[c]**2 + x[c]**2 + y[c]**2 + z[c]**2 - x[0]**2 -y[0]**2 -z[0]**2   
        
    pos = np.matmul(np.linalg.pinv(A), b)/2
    return pos[0], pos[1], pos[2] #pos x,y,z   

def reject_outliers(data, m = 2.):
    
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else np.zeros(len(d))
    return data[s<m]

def TDoA_dt(ts):
    t6_rx1 = float(int( ts[0], 2 )) * 15.65e-12
    t1_rx1 = float(int( ts[1],2  )) * 15.65e-12
    t2_rx1 = float(int(ts[2],2)) * 15.65e-12
    t3_rx1 = float(int(ts[3],2)) * 15.65e-12
    t4_rx1 = float(int(ts[4],2)) * 15.65e-12
    t5_rx1 = float(int(ts[5],2)) * 15.65e-12

    t6_rx2 = float(int(ts[6],2)) * 15.65e-12
    t1_rx2 = float(int(ts[7],2)) * 15.65e-12
    t2_rx2 = float(int(ts[8],2)) * 15.65e-12
    t3_rx2 = float(int(ts[9],2)) * 15.65e-12
    t4_rx2 = float(int(ts[10],2)) * 15.65e-12
    t5_rx2 = float(int(ts[11],2)) * 15.65e-12 #double(1/(63.8976 * 100000000float

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
    
    # Embedded Lab system anchor position
    '''A_n1 = np.array([0.00, 7.19, 2.15])
    A_n2 = np.array([0.00, 3.62, 3.15])
    A_n3 = np.array([0.00, 0.00, 2.15])
    A_n4 = np.array([4.79, 1.85, 3.15])
    A_n5 = np.array([4.79, 5.45, 2.15])
    A_n6 = np.array([3.00, 9.35, 3.15])'''

    A_n1 = [0.00, 7.19, 2.15]
    A_n2 = [0.00, 3.62, 3.15]
    A_n3 = [0.00, 0.00, 2.15]
    A_n4 = [4.79, 1.85, 3.15]
    A_n5 = [4.79, 5.45, 2.15]
    A_n6 = [3.00, 9.35, 3.15]

    A_n = np.array((A_n6, A_n1, A_n2, A_n3, A_n4, A_n5))
    c = 299792458 # Speed of light
    n = len(A_n)

    
    #TOF_MA = np.sqrt(np.sum)

    # Real measurements
    toa_tx = np.array([[t6_tx1,t6_tx2],[t1_tx1, t1_tx2], [t2_tx1,t2_tx2], [t3_tx1,t3_tx2], [t4_tx1,t4_tx2], [t5_tx1,t5_tx2]])
    toa_rx = np.array([[t6_rx1,t6_rx2], [t1_rx1,t1_rx2], [t2_rx1,t2_rx2], [t3_rx1,t3_rx2], [t4_rx1,t4_rx2], [t5_rx1,t5_rx2]])

    #Drift tag
    dt_new = (toa_rx[:,1]- toa_rx[:,0]) /(toa_tx[:,1] - toa_tx[:,0])

    return dt_new

def TDoA(ts, dt):
    # Time definition
    '''t6_rx1 = float(ts[0,0]) * 15.65e-12
    t1_rx1 = float(ts[1,0]) * 15.65e-12
    t2_rx1 = float(ts[2,0]) * 15.65e-12
    t3_rx1 = float(ts[3,0]) * 15.65e-12
    t4_rx1 = float(ts[4,0]) * 15.65e-12
    t5_rx1 = float(ts[5,0]) * 15.65e-12

    t6_rx2 = float(ts[0,1]) * 15.65e-12
    t1_rx2 = float(ts[1,1]) * 15.65e-12
    t2_rx2 = float(ts[2,1]) * 15.65e-12
    t3_rx2 = float(ts[3,1]) * 15.65e-12
    t4_rx2 = float(ts[4,1]) * 15.65e-12
    t5_rx2 = float(ts[5,1]) * 15.65e-12 #double(1/(63.8976 * 100000000float

    t6_tx1 = float(ts[0,2]) * 15.65e-12
    t1_tx1 = float(ts[1,2]) * 15.65e-12
    t2_tx1 = float(ts[2,2]) * 15.65e-12
    t3_tx1 = float(ts[3,2]) * 15.65e-12
    t4_tx1 = float(ts[4,2]) * 15.65e-12
    t5_tx1 = float(ts[5,2]) * 15.65e-12

    t6_tx2 = float(ts[0,3]) * 15.65e-12
    t1_tx2 = float(ts[1,3]) * 15.65e-12
    t2_tx2 = float(ts[2,3]) * 15.65e-12
    t3_tx2 = float(ts[3,3]) * 15.65e-12
    t4_tx2 = float(ts[4,3]) * 15.65e-12
    t5_tx2 = float(ts[5,3]) * 15.65e-12
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
    t5_rx2 = float(int(ts[11],2)) * 15.65e-12 #double(1/(63.8976 * 100000000float

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
    
    # Embedded Lab system anchor position
    '''A_n1 = np.array([0.00, 7.19, 2.15])
    A_n2 = np.array([0.00, 3.62, 3.15])
    A_n3 = np.array([0.00, 0.00, 2.15])
    A_n4 = np.array([4.79, 1.85, 3.15])
    A_n5 = np.array([4.79, 5.45, 2.15])
    A_n6 = np.array([3.00, 9.35, 3.15])'''

    A_n1 = [0.00, 7.19, 2.15]
    A_n2 = [0.00, 3.62, 3.15]
    A_n3 = [0.00, 0.00, 2.15]
    A_n4 = [4.79, 1.85, 3.15]
    A_n5 = [4.79, 5.45, 2.15]
    A_n6 = [3.00, 9.35, 3.15]

    A_n = np.array((A_n6, A_n1, A_n2, A_n3, A_n4, A_n5))
    c = 299792458 # Speed of light
    n = len(A_n)

    
    #TOF_MA = np.sqrt(np.sum)

    # Real measurements
    toa_tx = np.array([[t6_tx1,t6_tx2],[t1_tx1, t1_tx2], [t2_tx1,t2_tx2], [t3_tx1,t3_tx2], [t4_tx1,t4_tx2], [t5_tx1,t5_tx2]])
    toa_rx = np.array([[t6_rx1,t6_rx2], [t1_rx1,t1_rx2], [t2_rx1,t2_rx2], [t3_rx1,t3_rx2], [t4_rx1,t4_rx2], [t5_rx1,t5_rx2]])

    #Drift tag
    dt_new = (toa_rx[:,1]- toa_rx[:,0]) /(toa_tx[:,1] - toa_tx[:,0])
    
    
    tmp_rx = np.zeros((len(toa_rx),2))
    tmp_rx[:,0] = toa_rx[: , 0] - toa_rx[0, 0] - (toa_tx[: ,0] * dt - toa_tx[0, 0]*dt[0])
    tmp_rx[:,1] = toa_rx[: , 1] - toa_rx[0, 1] - (toa_tx[:, 1] * dt - toa_tx[0, 1]*dt[0])
    
    ## TDoA
    #     tdoa = tmp_rx(:,2) - tmp_tx(:,2);
    tdoa = np.zeros((len(tmp_rx), 2))
    tdoa = tmp_rx[:, 1]
    tdoa = np.delete(tdoa, [0])

    D = c*tdoa
    
    #------Trilateration linear equations system-------------------
    A = np.array((A_n6[0] - A_n[1:n, 0], A_n6[1]-A_n[1:n, 1], A_n6[2]-A_n[1:n, 2], D)).T *2
    
    b = D**2 + np.linalg.norm(A_n6)**2 - np.sum(np.square(A_n[1:n, :]), axis=1)
    
   
    
    x_t0 = np.matmul(np.linalg.pinv(A), b.T)
    
    #print("x_t0 =", x_t0) 
    #print("A =", A)
    #print("pinvA = ", sp.linalg.pinv(A))
    #-----Non linear correction (Taylor Expansion)-----------------
    x_t_0 = np.array((x_t0[0], x_t0[1], x_t0[2]))
    f = np.zeros((n-1,1))
    del_f = np.zeros((n-1,3))
    #ii = 1
    
    
    
    for ii in range(1,n) :
        f[ii-1]= np.linalg.norm((x_t_0 - A_n[ii,:]), ord=2)-np.linalg.norm((x_t_0 - A_n[0,:]), ord = 2)
        del_f[ii-1,0] = (x_t_0[0] - A_n[ii,0])*np.linalg.norm((x_t_0 - A_n[ii,:]),ord=2)**-1 - (x_t_0[0]-A_n[0,0])*np.linalg.norm((x_t_0-A_n[0,:]), ord = 2)**-1
        del_f[ii-1,1] = (x_t_0[1] - A_n[ii,1])*np.linalg.norm((x_t_0 - A_n[ii,:]),ord=2)**-1 - (x_t_0[1]-A_n[0,1])*np.linalg.norm((x_t_0-A_n[0,:]), ord=2)**-1
        del_f[ii-1,2] = (x_t_0[2] - A_n[ii,2])*np.linalg.norm((x_t_0 - A_n[ii,:]), ord=2)**-1 - (x_t_0[2]-A_n[0,2])*np.linalg.norm((x_t_0-A_n[0,:]), ord=2)**-1 

    #print("res = ", (D-f.T) )
    apinv = np.linalg.pinv(del_f)
    abho = (D- f.T).T
    
    x_t = (np.matmul(np.linalg.pinv(del_f), (D- f.T).T)).T + x_t_0

    return x_t[0,0], x_t[0,1], x_t[0,2], dt_new  # Cos� abbiamo in input y, x, z (?) 
