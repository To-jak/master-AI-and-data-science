#!/bin/env python3


"""

TP Reactive control for mobile robots

author: Alexandre Chapoutot and David Filliat

"""
import math

import numpy as np
import matplotlib.pyplot as plt

# Parameters


show_animation = True

### Util functions

def angle_wrap (a):
    while a > np.pi:
        a = a - 2.0 * np.pi
    while a < -np.pi:
        a = a + 2.0 * np.pi
    return a


#print (angle_wrap (3.18))

def dist (xTrue, xGoal):
    error = xGoal - xTrue
    if error.size == 3:
        error[2] = angle_wrap(error[2])
    return error


def plot_arrow(x, y, yaw, length=0.1, width=0.05, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


# Simulate vehicle

def simulate_unicycle (xTrue, u, v, w):
    # Display trajectory

    dt = 0.001 # integration time
    dVMax = 0.01 # limit linear acceleration
    dWMax = 0.01 # limit angular acceleration

    # limit acceleration
    delta_v = min(u[0] - v ,dVMax)
    delta_v = max(delta_v, -dVMax)
    delta_w = min(u[1] - w , dWMax)
    delta_w = max(delta_w, -dWMax)
    v = v + delta_v
    w = w + delta_w

    # Limit control
    v = min(1, v)
    v = max(-1, v)
    w = min(np.pi, w)
    w = max(-np.pi, w)
    u = np.array([v,w])

    # update state
    state = np.array([xTrue[0]+v*dt*np.cos(xTrue[2]), xTrue[1]+v*dt*np.sin(xTrue[2]), xTrue[2]+w*dt])
    state[2] = angle_wrap (state[2])
    return (state, u, v, w)

# Compute control

alpha_max = 1
k_dist = 1
k_alpha = 1
k_beta = 0.8

def unicycle_to_pose_control (xTrue, xGoal):
    u = np.array([0.0, 0.0])
    
    dist = ( (xGoal[0] - xTrue[0])**2 + (xGoal[1] - xTrue[1])**2 )**0.5
    alpha = angle_wrap( math.atan2(xGoal[1]-xTrue[1], xGoal[0]-xTrue[0]) - xTrue[2] )
    
        
    v = k_dist * dist
    w = k_alpha * alpha
    
    if dist < 0.05:
        beta = angle_wrap( xGoal[2] - xTrue[2] )
        w = k_beta * beta
        
    if abs(alpha) > alpha_max:
        v = 0
        
    u[0] = v
    u[1] = w
    
    return u



#### Main function

def main():
    print("reactive control of Unicycle start")

    # Goal and random starting position
    xGoal = np.array([0.0, 0.0, 0.0])
    a = np.random.rand() * 2.0 * np.pi
    xTrue = np.array([2.0 * np.cos(a), 2.0 * np.sin(a), np.random.rand(1) * 2.0 * np.pi])

    # Initial speed and angle
    v = 0
    w = 0

    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    if show_animation:
        # display xGoal
        #plt.plot(xGoal[0], xGoal[1], "*r")
        plot_arrow (xGoal[0], xGoal[1], xGoal[2], fc='b')
        # display initial position
        plot_arrow (xTrue[0], xTrue[1], xTrue[2], fc='r')
        #plt.plot(xTrue[0], xTrue[1], "*b")

    k = 0
    while np.amax(np.absolute(dist(xTrue,xGoal))) > 0.06 and k < 10000:
        # Compute Control
        u = unicycle_to_pose_control (xTrue, xGoal)

        # Simultion of the vehicle motion
        [xTrue, u, v, w] = simulate_unicycle(xTrue,u, v, w)

        if show_animation:
            if k % 100 == 1:
                plot_arrow (xTrue[0], xTrue[1], xTrue[2], fc='g');
                plt.plot(xTrue[0], xTrue[1], ".g")
                plt.pause(0.000001)

        k = k + 1

    print ("k = ", k)
    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
