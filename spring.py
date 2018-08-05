# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import math

def grav_sim(position, velocity, case_name, fall):
    
    pos_init = position
    vel_init = velocity
    
    # initialise empty lists to record trajectories
    pos_list_euler = []
    vel_list_euler = []
    
    pos_list_verlet = []
    vel_list_verlet = []
    
    # Euler integration
    for t in t_array:
    
        # append current state to trajectories
        pos_list_euler.append(position)
        vel_list_euler.append(velocity)    
    
        # calculate new position and velocity
        a = -G * M * position / (np.linalg.norm(position)**(3))
        position = position + dt * velocity
        velocity = velocity + dt * a
    
    # Verlet integration
    position = pos_init
    velocity = vel_init
    # Keep track of the position 2 iterations ago
    position_0 = position
    
    # First point using euler approximation - need 2 previous for verlet
    a = -G * M * position / (np.linalg.norm(position)**3)
    position_1 = position_0 + dt * velocity + 0.5 * (dt**2) * a
    velocity = velocity + dt * a
    
    pos_list_verlet.append(position_0)
    vel_list_verlet.append(velocity)
    
    for t in t_array[1:]:
        pos_list_verlet.append(position_1)
        vel_list_verlet.append(velocity)
        
        # calculate new position and velocity
        a = -G * M * position / (np.linalg.norm(position)**3)
        position = 2*position_1 - position_0 + a * dt * dt
        
        # Linear approximation for velocity between these 2 points
        velocity = (position - position_0)/(2 * dt)
        
        # Shift the two previous ones
        position_0 = position_1
        position_1 = position
          
    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    pos_array_euler = np.array(pos_list_euler)
    #vel_array_euler = np.array(vel_list_euler)
    
    pos_array_verlet = np.array(pos_list_verlet)
    #vel_array_verlet = np.array(vel_list_verlet)
    
    # plot the position-time graph    
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    if fall:
        # Plots only first dimenstion
        plt.plot(t_array, pos_array_euler[..., 0], label='Euler')
        plt.plot(t_array, pos_array_verlet[..., 0], label='Verlet')
    else:
        # Plots data in the plane of the orbit
        plt.plot(pos_array_euler[..., 0], pos_array_euler[..., 1], label='Euler')
        plt.plot(pos_array_verlet[..., 0], pos_array_verlet[..., 1], label='Verlet') 
        # Ensures scale is the same so that it looks circular
        plt.axis('equal')
    plt.title(case_name)
    plt.legend()
    
    plt.show()
    
# mass, spring constant, initial position and velocity
m = 1
G = 6.67 * 10**(-11)
M = 6.42 * 10**(23)
k = 1
R = 10 * 10**(6)

# simulation time, timestep and time
t_max = 100000
dt = 1
t_array = np.arange(0, t_max, dt)

# Initial conditions for the case of free fall
position_1 = np.multiply(np.ones(3), [R, 0.0, 0.0])
velocity_1 = np.zeros(3)
case_1 = "Free fall"

#grav_sim(position_1, velocity_1, case_1, True)

# Initial conditions for the case of circular orbit
position_2 = np.multiply(np.ones(3), [R, 0.0, 0.0])
velocity_2 = np.multiply(np.ones(3), [0.0, math.sqrt(G*M/position_2[0]), 0.0])
case_2 = "Circular orbit"

grav_sim(position_2, velocity_2, case_2, False)

# Initial conditions for the case of elliptical orbit
position_3 = np.multiply(np.ones(3), [R, 0.0, 0.0])
velocity_3 = np.multiply(np.ones(3), [0.0, 0.6*math.sqrt(G*M/position_2[0]), 0.0])
case_3 = "Elliptical orbit"

grav_sim(position_3, velocity_3, case_3, False)

# Initial conditions for the case of hyperbolic escape
position_4 = np.multiply(np.ones(3), [R, 0.0, 0.0])
velocity_4 = np.multiply(np.ones(3), [0.0, math.sqrt(2*G*M/position_2[0]), 0.0])
case_4 = "Hyperbolic escape"

grav_sim(position_4, velocity_4, case_4, False)