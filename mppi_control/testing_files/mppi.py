#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.signal import savgol_filter

class mppi():

    def __init__(self, initial_state, initial_action, goal, horizon, lam, sig, N, Q, R, P1):
        """
        initial_state: the starting position of the robot
        initial_action: an inital guess at the controls for the first horizon
        goal: target waypoint
        horizon: defines the time step, dt = 1/horizon
        lam: mppi parameter
        sig: sampling distribution varience
        N: number of rollouts
        """
        self.lam = lam
        self.sig = sig

        self.horizon = horizon
        self.dt = 1/horizon

        self.a = initial_action # 2xN
        self.a0 = initial_action[:,0] # action to append to a after robot has been issued a control
        self.cur_state = initial_state # 1x3 or just (3,)

        self.fin_path = [initial_state] # final path taken by the robot
        self.fin_control = [] # all of the control commands resulted in the final path
        self.fin_time = [0] # time stamp of the position/control

        self.N = N

        self.goal = goal

        self.t_cur = 0 # tracks the current time

        self.Q = Q
        self.R = R
        self.P1 = P1

    def step(self, x, u, f):
        """
        Function to propogate the robot forward one time step
        x: the current state of the robot
        """
        return x + f(x[:,2], u).T * self.dt

    def diff_drive(self, th, u):
        """
        Diff Drive Kinemtic model

        th:the angular position of the robot
        u: the control matrix, u1 is the left wheel, u0 is the right

        """
        # wheel radius
        radius = 0.033

        # wheel base
        wheel_base = 0.16/2

        return np.array([(radius / 2.0) * np.cos(th) * (u[:,0] + u[:,1]),
                         (radius / 2.0) * np.sin(th) * (u[:,0] + u[:,1]),
                         (radius / wheel_base) * (u[:,1] - u[:,0])])

    def unicycle(self, th, u):
        """
        unicycle kinematic model

        th and u should have the same number of rows.

        th: angular position of the robot
        u: control matrix, where each row is a unique control
        """
        return np.array([np.cos(th)*u[:,0], np.sin(th)*u[:,0], u[:,1]])

    def l(self, x, a, eps):
        """
        cost function

        All inputs should correspond to the same time stamp

        x: the state of the robot
        a: the control taken
        eps: the sampled control pertubations
        """
        output = np.zeros([self.N, 1])

        a = a.reshape(1,2)

        for n in range(self.N):
            deltax = (x[n,:] - self.goal).reshape(len(x[n,:]), 1)
            output[n,:] = (deltax).T.dot(self.Q).dot(deltax) + a.dot(self.R).dot(a.T) + self.lam * a.dot(self.sig).dot(eps[n,:].reshape(len(eps[n,:]),1))

        return output

    def m(self, x):
        """
        terminal cost function

        x: the state of the robot at the end of the horizon
        """
        output = np.zeros([self.N, 1])

        for n in range(self.N):
            deltax = (x[n,:] - self.goal).reshape(len(x[n,:]), 1)
            output[n,:] = (deltax).T.dot(self.P1).dot(deltax)

        return output

    def set_goal(self, goal, action_seq):
        """
        Function used to update the goal to drive towards
        """

        self.goal = goal
        self.a = action_seq # 2xN
        self.a0 = action_seq[:,0] # action to append to a after robot has been issued a control

    def go_to_goal(self):
        """
        Perform mppi algo, apply the control to the robot for 1 time step, and shift the control vecotr accordingly
        """

        J = [] # cost list
        eps = [] # samples list

        temp_state = np.tile(self.cur_state, (self.N,1))

        for t in range(self.horizon):
            # each element of eps is N x 2 matrix
            eps.append(np.random.normal(0, self.sig, size=(self.N, self.a.shape[0])))

            # calc cost - each element of J is Nx1
            J.append(self.l(temp_state, self.a[:,t], eps[-1]))

            # calc next state
            temp_state = self.step(temp_state, self.a[:,t] + eps[-1], self.diff_drive)

        J.append(self.m(temp_state))

        J = np.flip(np.cumsum(np.flip(J, 0), axis=0), 0)

        for t in range(self.horizon):

            J[t] -= np.amin(J[t]) # log sum exp trick

            w = np.exp(-J[t]/self.lam) + 1e-8
            w /= np.sum(w)

            # print(np.dot(w.T, eps[t]).shape)
            self.a[:,t] = self.a[:,t] + np.dot(w.T, eps[t])

        # filter to smooth the control
        self.a = savgol_filter(self.a, self.horizon - 1, 3, axis=1)

        # Apply control to robot
        self.cur_state = np.squeeze(self.step(self.cur_state.reshape(1,len(self.cur_state)), self.a[:,0].reshape(1,len(self.a[:,0])), self.diff_drive))

        # print(self.cur_state)
        self.fin_path.append(self.cur_state)
        self.fin_control.append(self.a[:,0])
        self.fin_time.append(self.fin_time[-1] + self.dt)

        # advance control vector
        self.a = np.concatenate([self.a[:, 1:], np.array(self.a0).reshape(2,1)], axis=1)

    def get_plot(self):
        """
        plot the final path and controls result
        """
        path = np.array(self.fin_path)
        control = np.array(self.fin_control)

        fig, (ax1, ax2, ax3) = plt.subplots(3)
        ax1.plot(path[:,0], path[:,1])
        ax1.set(xlabel="X position (m)", ylabel="Y position (m)")

        ax2.plot(self.fin_time[0:], path[:,0])
        ax2.plot(self.fin_time[0:], path[:,1])
        ax2.plot(self.fin_time[0:], path[:,2])
        ax2.legend(["x","y",r"$\theta$"])
        ax2.set(xlabel="Time (s)", ylabel="State (m or rad)")

        ax3.plot(self.fin_time[1:], control[:,0])
        ax3.plot(self.fin_time[1:], control[:,1])
        # ax3.legend(["v",r"$\omega$"])
        # ax3.set(xlabel="Time (s)", ylabel="Velocity (m/s or rad/s)")
        ax3.legend([r"$\phi_r$",r"$\phi_l$"])
        ax3.set(xlabel="Time (s)", ylabel="Velocity (rad/s)")

        plt.show()

    def get_animation(self):

        print("generating animation...")

        path = np.array(self.fin_path) # final path of robot
        control = np.array(self.fin_control) # controls used to generate the path

        fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, figsize=(10,10))

        fig.suptitle("MPPI Control Results", fontsize=16)

        ax1.set(xlabel="X position (m)", ylabel="Y position (m)")
        ax1.set_xlim(np.amin(path[:,0])-0.1 , np.amax(path[:,0])+0.1)
        ax1.set_ylim(np.amin(path[:,1])-0.1 , np.amax(path[:,1])+0.1)
        line11, = ax1.plot([], [])

        ax2.set(xlabel="Time (s)", ylabel="State (m or rad)")
        ax2.set_xlim(0 , self.fin_time[-1]) # fin_time is my time vector
        ax2.set_ylim(np.amin(path)-0.1 , np.amax(path)+0.1)
        line21, = ax2.plot([],[])
        line22, = ax2.plot([],[])
        line23, = ax2.plot([],[])
        ax2.legend(["x","y",r"$\theta$"])

        ax3.set_xlim(0 , self.fin_time[-1])
        ax3.set_ylim(np.amin(control)-0.1 , np.amax(control)+0.1)
        line31, = ax3.plot([], [])
        line32, = ax3.plot([], [])
        # ax3.legend(["v",r"$\omega$"])
        # ax3.set(xlabel="Time (s)", ylabel="Velocity (m/s or rad/s)")
        ax3.legend([r"$\phi_r$",r"$\phi_l$"])
        ax3.set(xlabel="Time (s)", ylabel="Velocity (rad/s)")

        def animate(i):

            ax1.clear()
            ax1.set(xlabel="X position (m)", ylabel="Y position (m)")
            ax1.set_xlim(np.amin(path[:,0])-0.1 , np.amax(path[:,0])+0.1)
            ax1.set_ylim(np.amin(path[:,1])-0.1 , np.amax(path[:,1])+0.1)

            ax1.arrow(x=path[i,0],y=path[i,1], dx=0.025*np.cos(path[i,2]), dy=0.025*np.sin(path[i,2]), head_width=0.03, length_includes_head=True, overhang=0.1, facecolor="black", zorder=0)

            line11, = ax1.plot([], [])

            line11.set_data(path[0:i,0], path[0:i,1])

            line21.set_data(self.fin_time[0:i], path[0:i,0])
            line22.set_data(self.fin_time[0:i], path[0:i,1])
            line23.set_data(self.fin_time[0:i], path[0:i,2])

            line31.set_data(self.fin_time[0:i], control[0:i,0])
            line32.set_data(self.fin_time[0:i], control[0:i,1])

            return line11, line21, line22, line23, line31, line32

        # may want to change the interval to match your dt, the number is in milliseconds
        anim = animation.FuncAnimation(fig, animate, frames=len(self.fin_time), interval=10, repeat=False)

        print("saving animation as mp4...this might take a while...")
        anim.save(filename='sim.mp4',fps=100,dpi=300)

    def made_it(self, lim):
        """
        determine the robot is at the goal location
        """
        # print(np.linalg.norm(self.cur_state[0:2] - self.goal[0:2], 2) + abs(self.cur_state[2] - self.goal[2]))
        print(np.linalg.norm(self.cur_state - self.goal, 2))
        return np.linalg.norm(self.cur_state - self.goal, 2) < lim

def main():

    # unicycle model params - parallel parking problem
    # lam = 0.1
    # sig = 0.3
    # N = 10
    # horizon = 100
    # Q = np.diagflat([1000., 10000., 0.1])
    # R = np.diagflat([1., .1])
    # P1 = np.diagflat([10000., 10000., 10000.])

    # unicycle model params - waypoint following
    # lam = 0.1
    # sig = 0.3
    # N = 10
    # horizon = 100
    # Q = np.diagflat([10000., 10000., 0.1])
    # R = np.diagflat([1., .1])
    # P1 = np.diagflat([10000., 10000., 1000.])
    #
    # waypoints = np.array([[4, 0, np.pi/2], [4, 4, np.pi/2], [2, 6, 5*np.pi/4], [0, 4, 3*np.pi/2], [0, 0, 3*np.pi/2]])

    # diff drive model params - parallel parking problem
    lam = 0.2
    sig = 0.3
    N = 13
    horizon = 100
    Q = np.diagflat([1000., 100000., 0])
    R = np.diagflat([10., 10.])
    P1 = np.diagflat([100000., 100000., 100.])

    x0 = np.array([0,0,np.pi/2])
    a0 = np.ones([2, horizon]) * np.array([[0, 0]]).T

    goal = np.array([4, 0, np.pi/2])
    control = mppi(x0, a0, goal, horizon, lam, sig, N, Q, R, P1)

    waypoints = np.array([[4, 0, np.pi/2]])

    threshold = 0.2

    # for i in range(100):
    for i, point in enumerate(waypoints):

        print(point)

        control.set_goal(point, a0)
        while not control.made_it(threshold):
            control.go_to_goal()

        print("Waypoint Reached: ", point)

    control.get_plot()
    control.get_animation()

main()
