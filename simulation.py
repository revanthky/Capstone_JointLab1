import numpy as np
import math
import random

import matplotlib.pyplot as plt

# grid formulation (100x100)
x_max = 100
y_max = 100
# diameter of wheel
wheel_d = 5
# north vector
north = [0, 1]
# delta t
delta_t = 0.01


# robot formulation
class Robot:
    def __init__(self, xpos, ypos, theta):
        # angles in radians
        self.state = [xpos, ypos,
                      theta]  # distance from front wall, distance from right wall, orientation as degree value where pi/2 is straight

    def pwmToRotVel(self, input_):
        output = np.array(100 * np.tanh(input_))
        # https://www.geeksforgeeks.org/numpy-tanh-python/ i used np beacuse of this, couldve been wrong though
        # output[0] = 100 * np.tanh(2 * r_vx)
        # output[1] = 100 * np.tanh(2 * r_vy)
        return output

    def updateState(self, nextState):
        self.state[0] = nextState[0]
        self.state[1] = nextState[1]
        self.state[2] = nextState[2]

    def getNextState(self, input_):
        # convert PWM to rotational velocity
        # rot_vel =[0, 0 #placeholder?
        # rot_vel = np.zeros(2) does it need to be instantiated first?
        rot_vel = self.pwmToRotVel(input_)
        # rot_vel[1] = self.pwmToRotVel(input_[1])

        # convert rotational vel to velocity
        # velocity = [0,0] #placeholder?
        # velocity = np.zeros(2)
        # velocity[0] = rot_vel[0] * (wheel_d / 2)
        # velocity[1] = rot_vel[1] * (wheel_d / 2)
        velocity = rot_vel * (wheel_d / 2)

        vbar = (velocity[0] + velocity[1]) / 2

        # delta x1, x2, x3
        delta_x = vbar * np.sin(self.state[2])
        delta_y = vbar * np.cos(self.state[2])
        omega = velocity[0] - velocity[1]

        nState = [0, 0, 0]  # placeholder?

        nState[0] = self.state[0] + (delta_x * delta_t)
        nState[1] = self.state[1] + (delta_y * delta_t)
        nState[2] = self.state[2] + (omega * delta_t)

        return nState

    def state_dynamic_equation(self, input_, noise=None, time=None):
        # update state (member variables) from input_, noise, and time
        if len(input_) != 2:
            return "Please enter a two element input_! [right wheel PWM, left wheel PWM]"
        if noise:
            return "Haven't implemented noise yet, sorry! Please try again."
        if time:
            return "Ignoring time due to Markov property! Please try again."

        # get next state
        nextState = self.getNextState(input_)

        # Update State
        self.updateState(nextState)

    def check_state(self, to_output):
        x1 = self.state[0]

        x2 = self.state[1]
        x3 = self.state[2]
        if x1 < 0 or x1 > x_max or x2 < 0 or x2 > y_max or x3 < 0 or x3 > 2 * np.pi:
            return "Invalid state!"
        return to_output

    def output_equation(self, input_, noise=None, time=None):
        # return output as 5 dimensional vector from state (member variables), input_, noise, and time
        if len(input_) != 2:
            return "Please enter a two element input_! [right wheel PWM, left wheel PWM]"
        if noise:
            return "Haven't implemented noise yet, sorry! Please try again."
        if time:
            return "Ignoring time due to Markov property! Please try again."

        output_vec = [0] * 5

        x = self.state[0]
        y = self.state[1]
        angle = self.state[2]

        def getMainLineIntersection(x, y, angle):
            slope = np.tan(angle)
            intercept = y - (slope * x)

            ywf = slope * x_max + intercept
            ywb = slope * 0 + intercept

            xwf = (y_max - intercept) / slope
            xwb = (0 - intercept) / slope
            return ((xwf, ywf), (xwb, ywb))

        def getPerpLineIntersection(x, y, angle):
            slope = np.tan(np.arctan(y / x) - angle)

            slope = -(1 / slope)
            intercept = y - (slope * x)

            y1 = slope * x_max + intercept
            y2 = slope * 0 + intercept

            x1 = (y_max - intercept) / slope
            x2 = (0 - intercept) / slope
            return ((x1, y1), (x2, y2))


        if angle > np.pi / 4 and angle <= 3 * np.pi / 4:
            ywf = y_max
            xwr = x_max
            xwf = x + np.tan(angle) * (y_max - y)
            ywr = y + np.tan(angle) * (x_max - x)

        elif (angle > (3 * np.pi / 4)) and (angle <= (5 * np.pi / 4)):
            xwf = 0
            ywr = y_max
            xwr = x + np.tan(angle) * (y_max - y)
            ywf = y + np.tan(angle) * (x_max - x)
        elif angle > 5 * np.pi / 4 and angle <= 7 * np.pi / 4:
            ywf = 0
            xwr = 0
            ywr = y + np.tan(angle) * (x_max - x)
            xwf = x + np.tan(angle) * (y_max - y)
        else:
            xwf = x_max
            ywr = 0
            xwr = x + np.tan(angle) * (y_max - y)
            ywf = y + np.tan(angle) * (x_max - x)

        output_vec[0] = np.sqrt((xwf - self.state[0]) ** 2 + (ywf - self.state[1]) ** 2)  # distance to the wall in front
        output_vec[1] = np.sqrt(
            (xwr - self.state[0]) ** 2 + (ywr - self.state[1]) ** 2)  # distance to the wall to the right

        # convert PWM to rotational velocity
        rot_vel = self.pwmToRotVel(input_)
        velocity = rot_vel * (wheel_d / 2)
        omega = velocity[0] - velocity[1]
        output_vec[2] = omega  # in plane rotational speed

        # take dot product of position vector with north, divide by their magnitudes, and take inverse cosine to get angle phi
        phi = np.arccos(
            (self.state[0] * north[0] + self.state[1] * north[1]) / np.linalg.norm([self.state[0], self.state[1]]))
        output_vec[3] = np.cos(phi)  # magnetic field in x direction
        output_vec[4] = np.sin(phi)  # magnetic field in y direction

        return self.check_state(output_vec)


# testing
rob = Robot(2, 3, math.pi / 4)
print(rob.state)
for i in range(10):
    rob.state_dynamic_equation([1, 1])
    print(rob.state)
    print(rob.output_equation([0, 0]))



