import numpy as np 
import matplotlib.pyplot as plt 
import scipy.integrate as spi 
from Simple_PID import PID_controller

# Instantiate the PID Object
pid = PID_controller(Kp=1.0, Ki=0.3, kd=0.1, setpoint=0)

# Set your external force applied to the system as a dictionary
applied_force = {'F': 0}

# The following equations were derived previously 
    
t_span = np.array([0,20])
z = np.array([100, 0])



sim = spi.solve_ivp(state, t_span, z,t_eval=np.arange(0,20,.1))

def figure1():
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(sim.t, sim.y[0])
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.subplot(2,1,2)
    plt.plot(sim.t, sim.y[1], c="r")
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.show()
    
    return 

figure1()