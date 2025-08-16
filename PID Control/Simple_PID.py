import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pynput import keyboard

# PID state variables
integral = 0.0
previous_error = 0.0
setpoint = 0.0

def update_pid(setpoint, current_position, Kp, Ki, Kd, dt):
    global integral, previous_error
    error = setpoint - current_position
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output, error

def on_press(key):
    global setpoint
    try:
        if key.char == 'i':   # increase setpoint
            setpoint += 2
            print(f"Setpoint increased to {setpoint}")
        elif key.char == 'k': # decrease setpoint
            setpoint -= 2
            print(f"Setpoint decreased to {setpoint}")
        elif key.char == 'l': # decrease setpoint
            setpoint += 5
            print(f"Setpoint decreased to {setpoint}")
        elif key.char == 'j': # decrease setpoint
            setpoint -= 5
            print(f"Setpoint decreased to {setpoint}")
    except AttributeError:
        if key == keyboard.Key.esc:
            print("Escape pressed, closing plot...")
            plt.close()

# listener for keyboard events
listener = keyboard.Listener(on_press=on_press)
listener.start()

# System parameters
m = 5.0   # mass
k = 1.0   # spring constant
dt = 0.02 # simulation time step
t_end = 10

# Initial conditions
t = 0.0
x = 10.0  # initial position
v = 5.0  # initial velocity

# Data storage for plotting
times = []
positions = []
velocities = []

# --- Matplotlib setup ---
fig, ax = plt.subplots(2, sharex=True)
line_pos, = ax[0].plot([], [], lw=2)
line_vel, = ax[1].plot([], [], lw=2, color='red')

ax[0].set_ylabel("Position (m)")
ax[1].set_ylabel("Velocity (m/s)")
ax[1].set_xlabel("Time (s)")

ax[0].axhline(0, color='k', linestyle='--')
ax[1].axhline(0, color='k', linestyle='--')

ax[0].set_xlim(0, t_end)
ax[0].set_ylim(-10, 10)
ax[1].set_ylim(-10, 10)

# --- Animation update function ---
def animate(frame):
    global t, x, v, setpoint

    # PID control force
    control_force, _ = update_pid(setpoint, x, Kp=100, Ki=50, Kd=50, dt=dt)

    # Physics update (Euler integration)
    a = -(k/m) * x + control_force / m
    v += a * dt
    x += v * dt
    t += dt

    # Save data
    times.append(t)
    positions.append(x)
    velocities.append(v)

    # Update lines
    line_pos.set_data(times, positions)
    line_vel.set_data(times, velocities)

    return line_pos, line_vel

ani = FuncAnimation(fig, animate, frames=int(t_end/dt), interval=dt*1000, blit=True)
plt.show()
ani.save("my_animation.gif", writer='pillow')
