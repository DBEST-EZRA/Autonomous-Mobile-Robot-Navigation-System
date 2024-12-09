import pybullet as p
import time
import math
import numpy as np

############################################### Environment Setup ####################################################
p.connect(p.GUI)

p.resetSimulation()

p.setGravity(0, 0, -10)
useRealTimeSim = 0

p.setRealTimeSimulation(useRealTimeSim)  # either this

# load plane
track = p.loadURDF("data/plane/plane.urdf")
# load car
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [0, 0, 0])


# load obstacles, in this projects, we used six cube as obstacles

def random_obstacles():
    np.random.seed()
    xy_position = [0, 0]
    xy_position_float = np.random.rand(2)
    x_poistion_range = np.random.randint(1, 10)
    y_poistion_range = np.random.randint(1, 10)

    xy_position[0] = xy_position_float[0] + x_poistion_range
    xy_position[1] = xy_position_float[1] + y_poistion_range

    np.asarray(xy_position)
    position = np.append(xy_position, 0.2)
    return position


cube_1_position = [6.8492311,3.603619,0.2]
cube_2_position = [3.4453734, 2.2388534, 0.2]
cube_3_position = [1.02668577, 5.70975462, 0.2]
cube_4_position = [3.44180162, 9.80349945, 0.2]
cube_5_position = [2.47666004, 8.18834429, 0.2]
cube_6_position = [9.32374415, 5.00996238, 0.2]
cube_7_position = [2.66267346, 4.07675668, 0.2]
cube_8_position = [2.95404594, 3.34534367, 0.2]

# cube_1_position = random_obstacles()
# print('cube_1_position: ',cube_1_position)
cube_1 = p.loadURDF('data/cube_test/marble_cube.urdf',cube_1_position)

# cube_2_position = random_obstacles()
# print('cube_2_position: ',cube_2_position)
cube_2 = p.loadURDF('data/cube_test/marble_cube.urdf',cube_2_position)

# cube_3_position = random_obstacles()
# print('cube_3_position: ',cube_3_position)
cube_3 = p.loadURDF('data/cube_test/marble_cube.urdf',cube_3_position)

# cube_4_position = random_obstacles()
# print('cube_4_position: ',cube_4_position)
cube_4 = p.loadURDF('data/cube_test/marble_cube.urdf',cube_4_position)

# cube_5_position = random_obstacles()
# print('cube_5_position: ',cube_5_position)
cube_5 = p.loadURDF('data/cube_test/marble_cube.urdf',cube_5_position)

# cube_6_position = random_obstacles()
# print('cube_6_position: ',cube_6_position)
cube_6 = p.loadURDF('data/cube_test/marble_cube.urdf',cube_6_position)


# cube_7_position = random_obstacles()
# print('cube_7_position: ',cube_7_position)
cube_7 = p.loadURDF('data/cube_test/marble_cube.urdf',cube_7_position)

# cube_8_position = random_obstacles()
# print('cube_8_position: ',cube_8_position)
cube_8 = p.loadURDF('data/cube_test/marble_cube.urdf',cube_8_position)


for wheel in range(p.getNumJoints(car)):
    # print("joint[", wheel, "]=", p.getJointInfo(car, wheel))
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    p.getJointInfo(car, wheel)

wheels = [8, 15]
print("----------------")

# p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
c = p.createConstraint(car, 9, car, 11, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 10, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 9, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 16, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 16, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 17, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 1, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
c = p.createConstraint(car, 3, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

steering = [0, 2]

hokuyo_joint = 4

#
# targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -50, 50, 0)
# maxForceSlider = p.addUserDebugParameter("maxForce", 0, 50, 20)
# steeringSlider = p.addUserDebugParameter("steering", -1, 1, 0)

replaceLines = True
# numRays = 100
# numRays = 100
numRays = 30
rayFrom = []
rayTo = []
rayIds = []
rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]
rayLen = 8
rayStartLen = 0.25
for i in range(numRays):
    rayFrom.append([rayStartLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                    rayStartLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    rayTo.append([rayLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                  rayLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    if (replaceLines):
        rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car,
                                         parentLinkIndex=hokuyo_joint))
    else:
        rayIds.append(-1)

frame = 0
lineId = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
lineId2 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
lineId3 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
print("lineId=", lineId)
lastTime = time.time()
lastControlTime = time.time()
lastLidarTime = time.time()

frame = 0

while (True):

    nowTime = time.time()
    # render Camera at 10Hertz
    if (nowTime - lastTime > .1):

        lastTime = nowTime

    nowControlTime = time.time()

    nowLidarTime = time.time()
    # lidar at 20Hz
    if (nowLidarTime - lastLidarTime > .3):
        # print("Lidar!")
        numThreads = 0
        results = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        print('result [0]: ',results[0])

        for i in range(numRays):
            hitObjectUid = results[i][0]
            hitFraction = results[i][2]
            hitPosition = results[i][3]
            if (hitFraction == 1.):
                p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
            else:
                localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                              rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                              rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]
                # print(localHitTo)
                p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        lastLidarTime = nowLidarTime

    # control at 100Hz
    if (nowControlTime - lastControlTime > .01):

        # task 2 implement a function to find a path to navigate the mobile robot from start position (0,0) to the target position (11,11)
        # current car position
        carPos, carOrn = p.getBasePositionAndOrientation(car)
        # get lidar information refer back to: line 159 'results' and 163-165, there are total 100 lines of lidar scans.
        # hitObjectUid = results[i][0]
        # hitFraction = results[i][2]
        # hitPosition = results[i][3]

        # task 1 implement a function to control the mobile robot. Input using veclocity of the wheel, and position of the steer
        # design your codes here.
        maxForce = 20
        targetVelocity = 0
        steeringAngle = 0





        for wheel in wheels:
            p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)

        for steer in steering:
            p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=-steeringAngle)



        if (useRealTimeSim == 0):
            frame += 1
            p.stepSimulation()
        lastControlTime = nowControlTime


