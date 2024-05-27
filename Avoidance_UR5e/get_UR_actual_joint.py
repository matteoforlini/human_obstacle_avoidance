import sys
sys.path.append('..')
import logging
import time
import rtde.rtde as rtde
import socket

localIP     = "169.254.0.25"
localPort   = 2000
bufferSize  = 1024

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
serverAddressPort   = (localIP, localPort)
 
ROBOT_HOST = '169.254.0.4'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'

keep_running = True
logging.getLogger().setLevel(logging.INFO)

state_names=["actual_q"]
state_types=["VECTOR6D"]

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types, frequency=500)

def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

#start data synchronization
if not con.send_start():
    sys.exit()
    
i=0
# control loop
time_start=time.perf_counter()
while i<1000:
    # receive the current state
    state = con.receive()
    i=i+1
time_end=time.perf_counter()
print(1000/(time_end-time_start))

# send data
while 1==1:
    state = con.receive()
    Q = state.actual_q
    Qwrite = Q
    Qstr = bytes(str(Qwrite), 'utf-8')
    # Send to server using UDP socket
    UDPServerSocket.sendto(Qstr, serverAddressPort)
    
        
con.send_pause()
con.disconnect()
