import pybullet as p
import time
import math
import numpy as np
import random


############################################### Environment Setup ####################################################
p.connect(p.GUI)

p.resetSimulation()

p.setGravity(0, 0, -10)
useRealTimeSim = 0


p.setRealTimeSimulation(useRealTimeSim)  # either this

# load plane
track = p.loadURDF("data/plane/plane.urdf")
# load car
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [0,0,0])
# load obstacles, in this projects, we used six cube as obstacles

def random_obstacles():
    np.random.seed()
    xy_position = [0,0]
    xy_position_float = np.random.rand(2)
    x_poistion_range = np.random.randint(1,10)
    y_poistion_range = np.random.randint(1,10)

    xy_position[0] = xy_position_float[0]+x_poistion_range
    xy_position[1] = xy_position_float[1]+y_poistion_range

    z_position = np.random.rand(1)
    np.asarray(xy_position)
    position = np.append(xy_position,z_position)
    return position


# Total eight different cubes. The cube's position is changing for each time. 
total_cubes_number = 10
data_path = 'data/'
cube_list = []
cube_name_list = ['cube_black',
                  'cube_green',
                  'cube']

for i in range(total_cubes_number):
    cube_list.append(random.choice(cube_name_list))

# Only two objects are loaded in stable positon. 
# You need to design URDF files to load objects, otherwise the code will not excute.
cube_stable_position_1 = p.loadURDF('data/cube_black/marble_cube.urdf',(5,5,0.2))
cube_stable_position_2 = p.loadURDF('data/cube/marble_cube.urdf',(3,3,0.2))

# 10 objects are loaed in random positions. 
cube_1_position = random_obstacles()
cube_1 = p.loadURDF('data/'+cube_list[0]+'/marble_cube.urdf',cube_1_position)

cube_2_position = random_obstacles()
cube_2 = p.loadURDF('data/'+cube_list[1]+'/marble_cube.urdf',cube_2_position)

cube_3_position = random_obstacles()
cube_3 = p.loadURDF('data/'+cube_list[2]+'/marble_cube.urdf',cube_3_position)

cube_4_position = random_obstacles()
cube_4 = p.loadURDF('data/'+cube_list[3]+'/marble_cube.urdf',cube_4_position)

cube_5_position = random_obstacles()
cube_5 = p.loadURDF('data/'+cube_list[4]+'/marble_cube.urdf',cube_5_position)

cube_6_position = random_obstacles()
cube_6 = p.loadURDF('data/'+cube_list[5]+'/marble_cube.urdf',cube_6_position)

cube_7_position = random_obstacles()
cube_7 = p.loadURDF('data/'+cube_list[6]+'/marble_cube.urdf',cube_7_position)

cube_8_position = random_obstacles()
cube_8 = p.loadURDF('data/'+cube_list[7]+'/marble_cube.urdf',cube_8_position)

cube_9_position = random_obstacles()
cube_9 = p.loadURDF('data/'+cube_list[8]+'/marble_cube.urdf',cube_9_position)

cube_10_position = random_obstacles()
cube_10 = p.loadURDF('data/'+cube_list[9]+'/marble_cube.urdf',cube_10_position)

for wheel in range(p.getNumJoints(car)):
    print("joint[", wheel, "]=", p.getJointInfo(car, wheel))
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
numRays = 100
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


def control_car(target_velocity, steering_angle):
    """
    Control the car's movement.
    Args:
        target_velocity (float): Target velocity for the wheels.
        steering_angle (float): Steering angle for the front wheels.
    """
    max_force = 20  # Maximum force for the wheels
    # Apply velocity to the wheels
    for wheel_ in wheels:
        p.setJointMotorControl2(car, wheel_, p.VELOCITY_CONTROL, targetVelocity=target_velocity, force=max_force)
    # Apply steering angle
    for steer_ in steering:
        p.setJointMotorControl2(car, steer_, p.POSITION_CONTROL, targetPosition=steering_angle)


def navigate_to_target(target_position):
    """
    Navigate the car to the target position.
    Args:
        target_position (tuple): Target x, y position.
    """
    car_position, car_orientation = p.getBasePositionAndOrientation(car)
    car_x, car_y, _ = car_position
    target_x, target_y = target_position

    # Compute the direction to the target
    direction_vector = [target_x - car_x, target_y - car_y]
    distance = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2)

    if distance < 0.1:  # Stop if close to the target
        control_car(0, 0)
        print("Target reached!")
        return True

    # Compute the desired heading
    desired_angle = math.atan2(direction_vector[1], direction_vector[0])

    # Compute the car's current heading
    car_yaw = p.getEulerFromQuaternion(car_orientation)[2]

    # Compute steering angle and velocity
    steering_angle = desired_angle - car_yaw
    steering_angle = max(min(steering_angle, 1.0), -1.0)  # Limit steering angle
    target_velocity = 5.0  # Constant speed

    # Control the car
    control_car(target_velocity, steering_angle)
    return False

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

        target_position = (11, 11)  # Target position
        target_reached = navigate_to_target(target_position)

        if target_reached:
            break  # Exit the loop when the target is reached





        for wheel in wheels:
            p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)

        for steer in steering:
            p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=-steeringAngle)



        if (useRealTimeSim == 0):
            frame += 1
            p.stepSimulation()
        lastControlTime = nowControlTime
