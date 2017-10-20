from pylab import *
import rospy
from numpy import linalg
# from ruamel.yaml import YAML
import yaml

def ROVsimulation(procVar,measVar,N,dt):
     #DO NOT CHANGE
     # Function Given for creating simulated data
     acc = zeros((3,N))
     vel = zeros((3,N))
     control_acc = zeros((3,N))
     pos = zeros((3,N))
     meas_pos = zeros((3,N))
     z = zeros((3,N))
     for i in arange(1,N):
             z = normal([0, 0,0], 1.0)- 0.05*control_acc[:,i-1]
             control_acc[:,i] = control_acc[:,i-1] + z - 0.01*vel[:,i-1]
             vel[:,i] = vel[:,i-1] + control_acc[:,i-1]*dt
             pos[:,i] = pos[:,i-1] + vel[:,i-1]*dt + control_acc[:,i-1]*0.5*dt**2

     acc = normal(control_acc, sqrt(procVar))
     pos = zeros((3,N))
     vel = zeros((3,N))

     for i in arange(1,N):

             vel[:,i] = vel[:,i-1] + acc[:,i-1]*dt
             pos[:,i] = pos[:,i-1] + vel[:,i-1]*dt + acc[:,i-1]*0.5*dt**2


     meas_pos = normal(pos, sqrt(measVar))
     real_pos = pos

     return control_acc, meas_pos, real_pos

N = int(1e5)            # number of simulation sample point
dt = 1.0/100.0          # time interval for each sample point
procVar = 0.001         # Process(Prediction) Noise
measVar = 3             # Measurement Noise

[control_acc, meas_pos, real_pos] = ROVsimulation(procVar,measVar,N,dt)

pos= []

for i in range(N)[0::100]:
    pos.append(real_pos[:,i])

code = yaml.load(u"""
{'max_forward_speed': 0.9, 'use_fixed_heading': False, 'heading': 0, 'point': [0, 2, -22]}
""")

pos = pos[:100]

print len(pos)

stream = file('3D_traj_100.yaml', 'w')
print code
for i in pos:
    if (  int(i[2]) > 0):
        i[2] = -1 * int(i[2])
    code['point'] = (int(i[0]), int(i[1]), int(i[2]))
    stream.write("-\n")
    yaml.dump(code, stream, indent= 8 )
