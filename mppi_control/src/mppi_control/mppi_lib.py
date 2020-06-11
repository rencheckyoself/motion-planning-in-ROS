## @package mppi_lib
# This file contains a class to perform mppi control with a diff drive robot


## class to represent a simple diff drive robot
class diff_drive_robot():

    ## The constructor.
    # @param self object pointer
    # @param radius the wheel radius
    # @param wheel_base the distance from the centerline to the wheel
    # @param wheel_speed_limit the max velocity the wheel can spin
    def__init__(self, radius, wheel_base, wheel_speed_limit):

        self.radius = radius
        self.wheel_base = wheel_base
        self.wheel_speed_limit = wheel_speed_limit

    ## The differential drive kinematic model
    #
    # @param th the angular position of the robot, an nx1 array
    # @param u the control, an nx2 array
    def model(self, th, u):
        # wheel radius
        radius = 0.033

        # wheel base
        wheel_base = 0.16

        return np.array([(radius / 2.0) * np.cos(th) * (u[:,0] + u[:,1]),
                         (radius / 2.0) * np.sin(th) * (u[:,0] + u[:,1]),
                         (radius / wheel_base) * (u[:,1] - u[:,0])])

## class for controlling a diff drive robot
class mppi():

    ## The constructor.
    # @param self object pointer
    # @param initial_action an inital guess at the controls for the first horizon, a 2xN array where row 0 is the right wheel velcity and row 1 is the left wheel Velocity
    # @param goal target waypoint, np.array([x, y, theta])
    # @param horizon_time defines the time to compute the control over
    # @param horizon_steps defines the number of descrete steps durint the time horizon
    # @param lam mppi parameter
    # @param sig sampling distribution varience
    # @param N number of rollouts/samples
    # @param Q 3x3 array of the cost function state weights
    # @param R 2x2 array of the cost function control weights
    # @param P1 3x3 array of the terminal cost function state weights
    # @robot a diff_drive_robot object, or something with a similar format
    def __init__(self, initial_action, goal, horizon_time, horizon_steps, lam, sig, N, Q, R, P1, robot):

        self.robot = robot

        self.lam = lam
        self.sig = sig

        self.horizon = horizon_steps
        self.dt = horizon_time/horizon_steps
        self.last_time = 0 # last time an action was sent to the robot

        self.a = initial_action # 2xN
        self.a0 = initial_action[:,0] # action to append to a after robot has been issued a control

        self.N = N

        self.goal = goal

        self.t_cur = 0 # tracks the current time

        self.Q = Q
        self.R = R
        self.P1 = P1


    ## Function to perform Euler integration to take one step in time.
    #
    # x and u should have the same number of rows
    # @param self the object pointer
    # @param x the current state, an nx3 array.
    # @param u the control state, an nx2 array.
    # @param f the function to use that defines the kinematics of the system being controlled
    #
    # returns the simulated next state of the robot
    def step(self, x, u, f):
        return x + f(x[:,2], u).T * self.dt

    ## Cost function
    #
    # Uses a quadratic cost function with a regularization term for the mppi samples
    #
    # All inputs should correspond to the same time stamp
    # @param self the object pointer
    # @param x the state of the robot, an Nx3 array
    # @param a the mean control, an array with 2 elements
    # @param eps the sampled control pertubations, an Nx2 array
    #
    # @ returns the cost at the ith step in the horizon, an Nx1 array
    def l(self, x, a, eps):
        output = np.zeros([self.N, 1])

        a = a.reshape(1,2)

        for n in range(self.N):
            deltax = (x[n,:] - self.goal).reshape(len(x[n,:]), 1)
            output[n,:] = (deltax).T.dot(self.Q).dot(deltax) + a.dot(self.R).dot(a.T) + self.lam * a.dot(self.sig).dot(eps[n,:].reshape(len(eps[n,:]),1))

        return output

    ## Terminal Cost function
    #
    # Uses a quadratic cost
    #
    # All inputs should correspond to the same time stamp
    # @param self the object pointer
    # @param x the state of the robot, an Nx3 array
    #
    # @ returns the cost at the end of the horizon, an Nx1 array
    def m(self, x):
        output = np.zeros([self.N, 1])

        for n in range(self.N):
            deltax = (x[n,:] - self.goal).reshape(len(x[n,:]), 1)
            output[n,:] = (deltax).T.dot(self.P1).dot(deltax)

        return output
    ## Function to change the target waypoint and inital action (if desired)
    #
    # @param self the object pointer
    # @param goal target waypoint, np.array([x, y, theta])
    # @param action_seq the inital guess for the new target. Defaults to None and will just use the existing values in the action array
    def set_goal(self, goal, action_seq=None):

        self.goal = goal

        if action_seq != None:
            self.a = action_seq # 2xN
            self.a0 = action_seq[:,0] # action to append to self.a after robot has been issued a control

    ## The main MPPI alogrithm
    #
    # @param self the object pointer
    # @param cur_state the current position of the robot, np.array([x, y, theta])
    # @param cur_time the current position of the robot, np.array([x, y, theta])
    #
    # @returns wheel velocities to apply to the robot
    # # TODO: Update how the control vector is advanced
    def get_control(self, cur_state, cur_time):

        J = [] # cost list
        eps = [] # samples list

        temp_state = np.tile(cur_state, (self.N,1))

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

        cmd = self.a[:,0]

        # advance control vector one step
        self.a = np.concatenate([self.a[:, 1:], np.array(self.a0).reshape(2,1)], axis=1)

        self.last_time = cur_time

        return cmd

    ## Function to determine if the robot is at the goal location
    #
    # @param self the object pointer
    # @param cur_state the current position of the robot, np.array([x, y, theta])
    # @param lim the threshold to determine if the goal has been met
    #
    # @returns True if the robot has arrived at the goal, otherwise False
    def made_it(self, cur_state, lim):
        return np.linalg.norm(self.cur_state - self.goal, 2) < lim
