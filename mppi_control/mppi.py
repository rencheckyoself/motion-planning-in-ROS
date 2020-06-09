#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

class mppi():

    def __init__(self, initial_state, initial_action, goal, horizon, lam, sig, N):

        self. lam = lam
        self. sig = sig

        self.horizon = horizon
        self.dt = 1/horizon

        self.a = initial_action # 2xN
        self.cur_state = initial_state # 1x3 or just (3,)
        self.fin_path = [initial_state]
        self.fin_control = []
        self.fin_time = [0]

        self.N = N

        self.goal = goal

        self.t_cur = 0

    def step(self, x, u):
        return x + self.f(x[:,2], u).T * self.dt

    def f(self, th, u):
        return np.array([np.cos(th)*u[:,0], np.sin(th)*u[:,0], u[:,1]])

    def l(self, x, a, eps):
        output = np.zeros([self.N, 1])

        a = a.reshape(1,2)

        Q = np.diagflat([.100, .100, 0.01]) # keep small otherwise w will result in overflow errors

        R = np.diagflat([0.01, 0.01])

        for n in range(self.N):
            deltax = (x[n,:] - self.goal).reshape(len(x[n,:]), 1)
            output[n,:] = (deltax).T.dot(Q).dot(deltax) + a.dot(R).dot(a.T) + self.lam * a.dot(self.sig).dot(eps[n,:].reshape(len(eps[n,:]),1))

        return output

    def m(self, x):

        output = np.zeros([self.N, 1])

        P1 = np.diagflat([10.0, 10.0, 0.1])

        for n in range(self.N):
            deltax = (x[n,:] - self.goal).reshape(len(x[n,:]), 1)
            output[n,:] = (deltax).T.dot(P1).dot(deltax)

        return output

    def go_to_goal(self):

        J = []
        eps = []

        temp_state = np.tile(self.cur_state, (self.N,1))

        for t in range(self.horizon):
            # each element of eps is N x 2 matrix
            eps.append(np.random.normal(0, self.sig, size=(self.N, self.a.shape[0])))

            # calc cost - each element of J is Nx1
            J.append(self.l(temp_state, self.a[:,t], eps[-1]))

            # calc next state
            temp_state = self.step(temp_state, self.a[:,t] + eps[-1])

        J.append(self.m(temp_state))

        J = np.flip(np.cumsum(np.flip(J, 0), axis=0), 0)

        for t in range(self.horizon):

            J[t] -= np.min(J[t]) # log sum exp trick

            w = np.exp(J[t]/self.lam) + 1e-8
            w /= np.sum(w)

            # print(np.dot(w.T, eps[t]).shape)
            self.a[:,t] = self.a[:,t] + np.dot(w.T, eps[t])

        # Apply control to robot
        self.cur_state = np.squeeze(self.step(self.cur_state.reshape(1,len(self.cur_state)), self.a[:,0].reshape(1,len(self.a[:,0]))))

        print(self.cur_state)
        self.fin_path.append(self.cur_state)
        self.fin_control.append(self.a[:,0])
        self.fin_time.append(self.fin_time[-1] + self.dt)

        # advance control vector
        self.a = np.concatenate([self.a[:, 1:], np.array([1,0]).reshape(2,1)], axis=1)

    def get_plot(self):

        path = np.array(self.fin_path)
        control = np.array(self.fin_control)

        fig = plt.figure()
        plt.plot(path[:,0], path[:,1])
        plt.xlim([-2,2])

        fig = plt.figure()
        plt.plot(self.fin_time[0:], path[:,0])
        plt.plot(self.fin_time[0:], path[:,1])
        plt.plot(self.fin_time[0:], path[:,2])

        fig = plt.figure()
        plt.plot(self.fin_time[1:], control[:,0])
        plt.plot(self.fin_time[1:], control[:,1])

        plt.show()

    def made_it(self):

        print(np.linalg.norm(self.cur_state[0:2] - self.goal[0:2], 2))

        return np.linalg.norm(self.cur_state[0:2] - self.goal[0:2], 2) < 0.2

def main():
    lam = 0.1
    sig = 0.2
    N = 5
    horizon = 100

    x0 = np.array([0,0,np.pi/2])
    a0 = np.ones([2, horizon]) * np.array([[1, 0]]).T
    goal = np.array([0, 4, np.pi/2])

    control = mppi(x0, a0, goal, horizon, lam, sig, N)

    for i in range(100):
    # while not control.made_it():
        control.go_to_goal()

    # control.get_plot()

main()
