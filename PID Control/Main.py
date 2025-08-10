import numpy as np 
import matplotlib.pyplot as plt 
import scipy.integrate as spi 
from Simple_PID import PID_controller
from matplotlib.animation import FuncAnimation

# Define your state function
def state(t,z):
    
    # Unpack the state vector or initial conditions
    x = z[0]
    v = z[1]

    ''' 
    or unpack like this:
    x, v = z
    where z is the state vector [position, velocity]
    '''
    
    # Parameters
    m = 5
    k = 1
    

    pid = PID_controller(Kp=.5, Ki=0.45, Kd=0.15, setpoint=0)
    applied_force = {'F': 10}

    dt = 0.1 # Time step for PID update
    control_force = pid.update(x, dt)
    total_force = control_force + applied_force['F']

    #total_force = 10
    # The following equations were derived previously 
    x_prime = v
    v_prime = -(k/m)*x + total_force/m
    
    z_prime = [x_prime, v_prime]
    
    return z_prime


# Initial state: [position, velocity]
z = np.array([100, 0])

dt = 0.1  # timestep in seconds
t_span = [0,20]
times = np.arange(t_span[0], t_span[1], dt)

sim = spi.solve_ivp(state, t_span, z, t_eval=times)

# Create you figure object
fig, ax = plt.subplots(nrows=2, sharex=True)
ax[0].plot(sim.t, sim.y[0])
ax[0].set_title('Position')
ax[0].plot([0,20],[0,0],ls='--',c='black')
ax[0].set_ylabel('m')
ax[1].plot(sim.t, sim.y[1], c='r')
ax[1].plot([0,20],[0,0],ls='--',c='black')
ax[1].set_title('Velocity')
ax[1].set_xlabel('Time')
ax[1].set_ylabel('m/s')
plt.show()
