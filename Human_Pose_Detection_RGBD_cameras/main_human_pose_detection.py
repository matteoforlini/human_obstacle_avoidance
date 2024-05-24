import threading
import queue
import matlab.engine
import realsense_depth_3camere_senza_socket as rdc
import sys
import time
import socket
import numpy as np

active_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
adress=('169.254.0.25', 2020)
try:
    active_socket.connect(('169.254.0.25', 2020))
except:
    print('server not connected')
    active_socket=None
    
    
eng=matlab.engine.start_matlab()

q1=queue.LifoQueue()
q2=queue.LifoQueue()
q3=queue.LifoQueue()
coda_exit=queue.LifoQueue() 


cam1=threading.Thread(target=rdc.DepthCamera, args=(eng, '046122250287', q1, q2, q3, coda_exit))
cam2=threading.Thread(target=rdc.DepthCamera, args=(eng, '046122251438', q1, q2, q3, coda_exit))
cam3=threading.Thread(target=rdc.DepthCamera, args=(eng, '046122250173', q1, q2, q3, coda_exit))


cam1.setDaemon(True)
cam2.setDaemon(True)
cam3.setDaemon(True)

cam1.start()
cam2.start()
cam3.start() 
 
i=0
ppTime=0
t_start=time.perf_counter()
C_tot_old=[1]
C_tot_old=matlab.double(C_tot_old)
start=np.zeros((4,33,1))
t_vect=[]
trigger_on=False
inizio_acqui=0
while True:

    if not coda_exit.empty():
        sys.exit(0)
        
        
    if not q2.empty() or (not q1.empty()) or (not q3.empty()):
        
        c1=q1.get()
        c1=c1.replace('[', '')
        c1=c1.replace(']', '')
        arr1 = [float(x) for x in c1.split(',')]
        c_1=matlab.double(arr1)
       
        c2=q2.get()
        c2=c2.replace('[', '')
        c2=c2.replace(']', '') 
        arr2 = [float(x) for x in c2.split(',')]
        c_2=matlab.double(arr2)

        c3=q3.get()
        c3=c3.replace('[', '')
        c3=c3.replace(']', '')
        arr3 = [float(x) for x in c3.split(',')]
        c_3=matlab.double(arr3)

    
        C_tot_tupla=eng.data_unification(c_1,c_2,c_3,C_tot_old,nargout=2)
        C_tot_old=matlab.double(C_tot_tupla[0])
        C_tot=C_tot_tupla[0]
        Cam=C_tot_tupla[1]

        Cfin=C_tot
        
        Cfin_np=np.asarray(Cfin)
        Cfin_np=np.reshape(Cfin_np, (4, 33, 1))
        active_socket.send(bytes(str(Cfin).encode('utf8')))
        

        ccTime = time.perf_counter()
        fpss = 1 / (ccTime - ppTime)
        ppTime = ccTime        

        print('Cam',Cam,'VIS',Cfin_np[0,16,0],'X_p_dx',Cfin_np[1,16,0],'Y_p_dx',Cfin_np[2,16,0],'Z_p_dx',Cfin_np[3,16,0],'freq',round(fpss,2))
        i=i+1
        
    else:
        time.sleep(0.01)
        

        
        
        
        
          
          











