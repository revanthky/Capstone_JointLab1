import numpy as np
import math
import random

import pandas as pd

import matplotlib.pyplot as plt

# grid formulation (100x100)
x_max = 100
y_max = 100
# diameter of wheel
wheel_d = 0.5 #0.5 for paperbot, 5 for segway; magnitude of 10 difference
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
        if nextState[0] > 0 and nextState[0] < 100:
            self.state[0] = nextState[0]
        if nextState[1] > 0 and nextState[1] < 100:
            self.state[1] = nextState[1]
        if nextState[2] > 0 and nextState[2] < 100:
            self.state[2] = nextState[2]

    def getNextState(self, input_):
        # convert PWM to rotational velocity
        # rot_vel =[0, 0 #placeholder?
        # rot_vel = np.zeros(2) does it need to be instantiated first?
        #rot_vel = self.pwmToRotVel(input_) COMMENTED THIS OUT FOR LAB 2: now, can directly input angular velocities
        rot_vel = np.array(input_)
        # rot_vel[1] = self.pwmToRotVel(input_[1])

        # convert rotational vel to velocity
        # velocity = [0,0] #placeholder?
        # velocity = np.zeros(2)
        # velocity[0] = rot_vel[0] * (wheel_d / 2)
        # velocity[1] = rot_vel[1] * (wheel_d / 2)
        velocity = rot_vel * (wheel_d / 2)

        vbar = (velocity[0] + velocity[1]) / 2

        # delta x1, x2, x3
        delta_x = vbar * np.cos(self.state[2])
        delta_y = vbar * np.sin(self.state[2])
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
            while angle >= 2 * np.pi:
                angle -= 2 * np.pi
            while angle < 0:
                angle += 2 * np.pi
            if angle == 0:
                xwf = x_max
                ywf = y
            elif angle == np.pi / 2:
                xwf = x
                ywf = y_max
            elif angle == np.pi:
                xwf = 0
                ywf = y
            elif angle == 3 * np.pi / 2:
                xwf = x
                ywf = 0
            else:
                slope = np.tan(angle)
                intercept = y - (slope * x)
                ywf = min(y_max, slope * x_max + intercept)
                ywf = max(ywf, 0)
                # ywb = slope*0 + intercept
                xwf = min(x_max, (y_max - intercept) / slope)
                xwf = max(xwf, 0)
                # xwb = (0 - intercept) / slope
            return (xwf, ywf)

        def getPerpLineIntersection(x, y, angle):
            angle -= np.pi / 2
            while angle >= 2 * np.pi:
                angle -= 2 * np.pi
            while angle < 0:
                angle += 2 * np.pi
            if angle == 0:
                xwr = x_max
                ywr = y
            elif angle == np.pi / 2:
                xwr = x
                ywr = y_max
            elif angle == np.pi:
                xwr = 0
                ywr = y
            elif angle == 3 * np.pi / 2:
                xwr = x
                ywr = 0
            else:
                slope = np.tan(angle)
                intercept = y - (slope * x)
                ywr = min(y_max, slope * x_max + intercept)
                ywr = max(ywr, 0)
                # ywl = slope*0 + intercept
                # xwl = (y_max - intercept) / slope
                xwr = min(x_max, (0 - intercept) / slope)
                xwr = max(xwr, 0)
            return (xwr, ywr)

        xwf, ywf = getMainLineIntersection(x, y, angle)
        xwr, ywr = getPerpLineIntersection(x, y, angle)
        output_vec[0] = np.sqrt(
            (xwf - self.state[0]) ** 2 + (ywf - self.state[1]) ** 2)  # distance to the wall in front
        output_vec[1] = np.sqrt(
            (xwr - self.state[0]) ** 2 + (ywr - self.state[1]) ** 2)  # distance to the wall to the right

        # output_vec[0] = np.linalg.norm(np.array([xwf,ywf]) - np.array([self.state[0],self.state[1]]))
        # output_vec[1] = np.linalg.norm(np.array([xwr,ywr]) - np.array([self.state[0],self.state[1]]))
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

        return output_vec


rob = Robot(0, 0, math.pi / 4)

#loading in the csv files into pandas dataframes
seg1_data = pd.read_csv('Segwaysimulationdata1.csv')
seg2_data = pd.read_csv('Segwaysimulationdata2.csv')

seg3_data = pd.read_csv('Segwaysimulationdata3.csv')
seg4_data = pd.read_csv('Segwaysimulationdata4.csv')

paper1_data = pd.read_csv('Paperbotsimulationdata1.csv')
paper2_data = pd.read_csv('Paperbotsimulationdata2.csv')

paper3_data = pd.read_csv('Paperbotsimulationdata3.csv')
paper4_data = pd.read_csv('Paperbotsimulationdata4.csv')

#extracting the relevant columns, related to wheel velocity, from the data
seg_traj1_left = seg1_data['AngularVelocity2'].tolist() #to list: creates python list var, NOT NUMPY
seg_traj1_right = seg1_data['AngularVelocity3'].tolist()

seg_traj2_left = seg2_data['AngularVelocity2'].tolist()
seg_traj2_right = seg2_data['AngularVelocity3'].tolist()



seg_traj3_left = seg3_data['AngularVelocity2'].tolist()
seg_traj3_right = seg3_data['AngularVelocity3'].tolist()

seg_traj4_left = seg4_data['AngularVelocity2'].tolist()
seg_traj4_right = seg4_data['AngularVelocity3'].tolist()



paper_traj1_left = paper1_data['AngularVelocity2'].tolist()
paper_traj1_right = paper1_data['AngularVelocity3'].tolist()

paper_traj2_left = paper2_data['AngularVelocity2'].tolist()
paper_traj2_right = paper2_data['AngularVelocity3'].tolist()


paper_traj3_left = paper3_data['AngularVelocity2'].tolist()
paper_traj3_right = paper3_data['AngularVelocity3'].tolist()


paper_traj4_left = paper4_data['AngularVelocity2'].tolist()
paper_traj4_right = paper4_data['AngularVelocity3'].tolist()




seg1_len = len(seg_traj1_left)
#print(len(seg_traj1_right))

seg2_len= len(seg_traj2_left)
#print(len(seg_traj2_right))

seg3_len= len(seg_traj3_left)


seg4_len= len(seg_traj4_left)

paper1_len= len(paper_traj1_left)
#print(len(paper_traj1_right))

paper2_len= len(paper_traj2_left)
#print(len(paper_traj2_right))


paper3_len= len(paper_traj3_left)


paper4_len= len(paper_traj4_left)



# four Segway simulations

xposes = []
yposes = []
rob = Robot(0, 0, math.pi / 4)
# print(rob.output_equation([0,0]))
for i in range(seg1_len):
    rob.state_dynamic_equation([seg_traj1_left[i]/10, seg_traj1_right[i]/10])
    xposes.append(rob.state[0])
    yposes.append(rob.state[1])
# print(rob.output_equation([0,0]))
plt.plot(xposes, yposes)
plt.xlabel('x pos')
plt.ylabel('y pos')
plt.title('Segway 1')
plt.show()

# Test 2
rob = Robot(0, 0, math.pi / 4)
xposes = []
yposes = []
# print(rob.output_equation([0,0]))
for i in range(seg2_len):
    rob.state_dynamic_equation([seg_traj2_left[i] / 10, seg_traj2_right[i] / 10])
    xposes.append(rob.state[0])
    yposes.append(rob.state[1])
# print(rob.output_equation([0,0]))
plt.plot(xposes, yposes)
plt.xlabel('x pos')
plt.ylabel('y pos')
plt.title('Segway 2')
plt.show()



rob = Robot(50, 50, math.pi / 4)
xposes = []
yposes = []
# print(rob.output_equation([0,0]))
for i in range(seg3_len):
    rob.state_dynamic_equation([seg_traj3_left[i] / 10, seg_traj3_right[i] / 10])
    xposes.append(rob.state[0])
    yposes.append(rob.state[1])
# print(rob.output_equation([0,0]))
plt.plot(xposes, yposes)
plt.xlabel('x pos')
plt.ylabel('y pos')
plt.title('Segway 3')
plt.show()



rob = Robot(50, 50, math.pi / 4)
xposes = []
yposes = []
# print(rob.output_equation([0,0]))
for i in range(seg4_len):
    rob.state_dynamic_equation([seg_traj4_left[i] / 10, seg_traj4_right[i] / 10])
    xposes.append(rob.state[0])
    yposes.append(rob.state[1])
# print(rob.output_equation([0,0]))
plt.plot(xposes, yposes)
plt.xlabel('x pos')
plt.ylabel('y pos')
plt.title('Segway 4')
plt.show()




#Four paperbot simulations

rob = Robot(90, 50, math.pi / 4)
xposes = []
yposes = []
# print(rob.output_equation([0,0]))
for i in range(paper1_len):
    rob.state_dynamic_equation([paper_traj1_left[i] / 100, paper_traj1_right[i] / 100])
    xposes.append(rob.state[0])
    yposes.append(rob.state[1])
# print(rob.output_equation([0,0]))
plt.plot(xposes, yposes)
plt.xlabel('x pos')
plt.ylabel('y pos')
plt.title('PaperBot 1')
plt.show()



rob = Robot(90, 90, math.pi / 4)
xposes = []
yposes = []
# print(rob.output_equation([0,0]))
for i in range(paper2_len):
    rob.state_dynamic_equation([paper_traj2_left[i] / 10, paper_traj2_right[i] / 10])
    xposes.append(rob.state[0])
    yposes.append(rob.state[1])
# print(rob.output_equation([0,0]))
plt.plot(xposes, yposes)
plt.xlabel('x pos')
plt.ylabel('y pos')
plt.title('PaperBot 2 ')
plt.show()



rob = Robot(70, 20, math.pi / 4)
xposes = []
yposes = []
# print(rob.output_equation([0,0]))
for i in range(paper3_len):
    rob.state_dynamic_equation([paper_traj3_left[i] / 10, paper_traj3_right[i] / 10])
    xposes.append(rob.state[0])
    yposes.append(rob.state[1])
# print(rob.output_equation([0,0]))
plt.plot(xposes, yposes)
plt.xlabel('x pos')
plt.ylabel('y pos')
plt.title('PaperBot 3 ')
plt.show()



rob = Robot(50, 50, math.pi / 4)
xposes = []
yposes = []
# print(rob.output_equation([0,0]))
for i in range(paper4_len):
    rob.state_dynamic_equation([paper_traj4_left[i] / 10, paper_traj4_right[i] / 10])
    xposes.append(rob.state[0])
    yposes.append(rob.state[1])
# print(rob.output_equation([0,0]))
plt.plot(xposes, yposes)
plt.xlabel('x pos')
plt.ylabel('y pos')
plt.title('PaperBot 4 ')
plt.show()


