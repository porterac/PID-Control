import numpy as np 
import matplotlib.pyplot as plt 
import scipy.integrate as spi 
from matplotlib.animation import FuncAnimation

integral = 0
previous_error = 0
last_t = None


# Define your state function
def state(t,z):
    global last_t
    
    # Unpack the state vector or initial conditions
    x = z[0]
    v = z[1]
    E = z[2]

    ''' 
    or unpack like this:
    x, v = z
    where z is the state vector [position, velocity]
    '''
    
    # Parameters
    m = 5
    k = 1

    # Compute dt from solver time
    if last_t is None:
        dt = 0.0
    else:
        dt = t - last_t
    last_t = t

    control_force, error = update(0, x, 1, .5, 0, dt)

    # The following equations were derived previously 
    x_prime = v
    v_prime = -(k/m)*x + control_force/m
    E_prime = error
    
    z_prime = [x_prime, v_prime, E_prime]
    
    return z_prime

def update(setpoint, current_position, Kp, Ki, Kd, dt):
    global integral, previous_error
    # Error Calculation
    error = setpoint - current_position

    # Integral Calculation
    integral += error * dt

    # Derivative Calculation
    if dt > 0: 
        derivative = (error - previous_error) / dt 
    else:
        derivative = 0

    output = (Kp * error) + (Ki * integral) + (Kd * derivative)

    # Set current error to previous error
    previous_error = error
    
    
    return output, error


# Initial state: [position, velocity]
z = np.array([5, 0, 0])

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
