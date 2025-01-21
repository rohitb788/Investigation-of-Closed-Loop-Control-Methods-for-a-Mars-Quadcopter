# -*- coding: utf-8 -*-
"""
Created on Sat Nov  9 18:10:38 2024

@author: rohit
"""
from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt

#constants
g = 3.73 #gravity - 9.81 m/s^2 on Earth. 3.73 m/s^2 on Mars
m = 4 #mass of mars helicopter
I = 1 #inertia of helicopter - placeholder number

# define the time span
t0 = 0 # initial time (seconds)
tf = 170    # final time

time_span = [t0, tf]

#gains
Kp_z   = 0.009
Kd_z   = 0.07
Kp_phi = 1.7
Kd_phi = 1.2
kp_y = 0.01
kd_y = 0.07

# define initial condition
y_p = 5
y_v = 5
z_p = 2
z_v = 1.2
phi_p = 4 
phi_v = 6

#initial state
x0 = [y_p, z_p, phi_p, y_v, z_v, phi_v]

#desired trajectory
y_des = 0
z_des = 4
vy_des = 0
vz_des = 0
ay_des = 0
az_des = 0

#control law
def x_dot(t, x):
    #does calculations in the input vector as shown in the paper
    phi_c = (-1/g) * (ay_des + kd_y*(vy_des - x[3]) + kp_y* (y_des - x[0])) #phi_controller
    F = m*(g + az_des + Kd_z * (vz_des - x[4]) + Kp_z * (z_des - x[1])) #Controller Force
    M = I * (Kd_phi*(-x[5]) + Kp_phi * (phi_c - x[2])) #controlled torque
    
    return [x[3], x[4], x[5], (-F * np.sin(x[2])/m), (F * np.cos(x[2])/m - g), (M/I)] #non-linear dynamics

sol = solve_ivp(x_dot, time_span, x0) #integrator 

### Plotting

#y
plt.figure(0)

plt.plot(sol.t, sol.y[0])# y
plt.plot(sol.t, sol.y[3])# y

plt.title("y")
plt.xlabel("time (sec.)")
plt.ylabel("system states")
plt.legend(['y-Position (m.)','y-Velocity (m./sec.)'])
plt.minorticks_on()
plt.grid(which='minor', color='#DDDDDD', linestyle=':', linewidth=0.5)

#z
plt.figure(1)
plt.plot(sol.t, sol.y[1]) 
plt.plot(sol.t, sol.y[4])

plt.title("z")
plt.xlabel("time (sec.)")
plt.ylabel("system states")
plt.legend(['z-Position (m.)','z-Velocity (m./sec.)'])
plt.minorticks_on()
plt.grid(which='minor', color='#DDDDDD', linestyle=':', linewidth=0.5)

#phi
plt.figure(2)

plt.plot(sol.t, sol.y[2])
plt.plot(sol.t, sol.y[5])

plt.title("phi")
plt.xlabel("time (sec.)")
plt.ylabel("system states")
plt.legend(['Phi (rad.)','Phi-dot (rad./sec.)'])
plt.minorticks_on()
plt.grid(which='minor', color='#DDDDDD', linestyle=':', linewidth=0.5)

#y vs z position
plt.figure(3)

plt.plot(sol.y[1],sol.y[0]) # z vs y

plt.title("z vs y")
plt.xlabel("y-position")
plt.ylabel("z-position")
plt.minorticks_on()
plt.grid(which='minor', color='#DDDDDD', linestyle=':', linewidth=0.5)