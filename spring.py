# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x_ver, x_eul = 0, 0
v_ver, v_eul = 1, 1

# simulation time, timestep and time
t_max = 100
dt = 0.25
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list_euler = []
v_list_euler = []

x_list_verlet = []
v_list_verlet = []

# Euler integration
for t in t_array:

    # append current state to trajectories
    x_list_euler.append(x_eul)
    v_list_euler.append(v_eul)    

    # calculate new position and velocity
    a = -k * x_eul / m
    x_eul = x_eul + dt * v_eul
    v_eul = v_eul + dt * a

# Verlet integration

# Keep track of the position 2 iterations ago
x_ver_0 = x_ver

# First point using euler approximation - need 2 previous for verlet
a = -k * x_ver / m
x_ver_1 = x_ver_0 + dt * v_ver + 0.5 * (dt**2) * a
v_ver = v_ver + dt * a

x_list_verlet.append(x_ver_0)
v_list_verlet.append(v_ver)

for t in t_array[1:]:
    x_list_verlet.append(x_ver_1)
    v_list_verlet.append(v_ver)
    
    # calculate new position and velocity
    a = -k * x_ver_1 / m
    x_ver = 2*x_ver_1 - x_ver_0 + a * dt * dt
    
    # Linear approximation for velocity between these 2 points
    v_ver = (x_ver - x_ver_0)/(2 * dt)
    
    # Shift the two previous ones
    x_ver_0 = x_ver_1
    x_ver_1 = x_ver
      
# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array_euler = np.array(x_list_euler)
v_array_euler = np.array(v_list_euler)

x_array_verlet = np.array(x_list_verlet)
v_array_verlet = np.array(v_list_verlet)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array_euler, label='x (m)')
plt.plot(t_array, v_array_euler, label='v (m/s)')
plt.title("Euler method simulation with dt="+str(dt))
plt.legend()

plt.figure(2)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array_verlet, label='x (m)')
plt.plot(t_array, v_array_verlet, label='v (m/s)')
plt.title("Verlet method simulation with dt="+str(dt))
plt.legend()

plt.show()
