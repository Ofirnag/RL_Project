import numpy as np
import time
import math
from airsim.client import *


class myAirSimClient():

    def __init__(self):
        initX = 0
        initY = 0
        initZ = -3

        # connect to the AirSim simulator
        self.client = MultirotorClient()
        self.client_2 = MultirotorClient()
        self.client.confirmConnection()
        self.client_2.confirmConnection()
        self.client.enableApiControl(True,vehicle_name="SimpleFlight1")
        self.client_2.enableApiControl(True,vehicle_name="SimpleFlight2")
        self.client.armDisarm(True,vehicle_name="SimpleFlight1")
        self.client_2.armDisarm(True,vehicle_name="SimpleFlight2")

        self.client.takeoffAsync(vehicle_name="SimpleFlight1").join()
        self.client_2.takeoffAsync(vehicle_name="SimpleFlight2").join()
        # self.client.moveToPositionAsync(initX, initY, initZ, 5).join()

        self.home_pos = self.client.simGetVehiclePose(vehicle_name="SimpleFlight1").position
        self.home_pos_2 = self.client_2.simGetVehiclePose(vehicle_name="SimpleFlight2").position
        # print(self.home_pos)
        self.home_ori = self.client.simGetGroundTruthKinematics(vehicle_name="SimpleFlight1").orientation
        self.home_ori_2 = self.client_2.simGetGroundTruthKinematics(vehicle_name="SimpleFlight2").orientation
        # print(self.home_ori)
        self.z = -2

    @staticmethod
    def toEulerianAngle(q):
        z = q.z_val
        y = q.y_val
        x = q.x_val
        w = q.w_val
        ysqr = y * y

        # roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        roll = math.atan2(t0, t1)

        # pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        if (t2 > 1.0):
            t2 = 1
        if (t2 < -1.0):
            t2 = -1.0
        pitch = math.asin(t2)

        # yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        yaw = math.atan2(t3, t4)

        return pitch, roll, yaw

    def getPitchRollYaw(self):
        return self.toEulerianAngle(self.client.simGetGroundTruthKinematics(vehicle_name="SimpleFlight1").orientation)

    def getPitchRollYaw_2(self):
        return self.toEulerianAngle(self.client_2.simGetGroundTruthKinematics(vehicle_name="SimpleFlight2").orientation)

    def getPosition_1(self):
        return self.client.simGetVehiclePose(vehicle_name="SimpleFlight1").position

    def getPosition_2(self):
        return self.client_2.simGetVehiclePose(vehicle_name="SimpleFlight2").position

    def straight(self, duration, speed):
        pitch, roll, yaw = self.getPitchRollYaw()
        vx = math.cos(yaw) * speed
        vy = math.sin(yaw) * speed
        self.client.moveByVelocityZAsync(vx, vy, self.z, duration, DrivetrainType.ForwardOnly,vehicle_name="SimpleFlight1").join()
        start = time.time()
        return start, duration

    def straight_2(self, duration, speed):
        pitch, roll, yaw = self.getPitchRollYaw()
        vx = math.cos(yaw) * speed
        vy = math.sin(yaw) * speed
        self.client_2.moveByVelocityZAsync(vx, vy, self.z, duration, DrivetrainType.ForwardOnly,vehicle_name="SimpleFlight2").join()
        start = time.time()
        return start, duration

    def yaw_right(self, duration):
        self.client.rotateByYawRateAsync(30, duration, vehicle_name="SimpleFlight1").join()
        start = time.time()
        return start, duration

    def yaw_right_2(self, duration):
        self.client_2.rotateByYawRateAsync(30, duration, vehicle_name="SimpleFlight2").join()
        start = time.time()
        return start, duration

    def yaw_left(self, duration):
        self.client.rotateByYawRateAsync(-30, duration,vehicle_name="SimpleFlight1").join()
        start = time.time()
        return start, duration

    def yaw_left_2(self, duration):
        self.client_2.rotateByYawRateAsync(-30, duration,vehicle_name="SimpleFlight2").join()
        start = time.time()
        return start, duration


    def stop(self):
        self.client.moveByVelocityAsync(0, 0, 0, 0.01,vehicle_name="SimpleFlight1").join()
        self.client.rotateByYawRateAsync(0, 0.01, vehicle_name="SimpleFlight1").join()

    def stop_2(self):
        self.client_2.moveByVelocityAsync(0, 0, 0, 0.01,vehicle_name="SimpleFlight2").join()
        self.client_2.rotateByYawRateAsync(0, 0.01, vehicle_name="SimpleFlight2").join()

    def take_action(self, action):
        start = time.time()
        duration = 0
        collided = False
        if action == 0:
            start, duration = self.straight(1, 0.5)
            while duration > time.time() - start:
                if self.client.simGetCollisionInfo(vehicle_name="SimpleFlight1").has_collided:
                    return True
            # self.stop()

        if action == 1:
            start, duration = self.yaw_right(0.8)
            while duration > time.time() - start:
                if self.client.simGetCollisionInfo(vehicle_name="SimpleFlight1").has_collided:
                    return True
            # self.stop()

        if action == 2:
            start, duration = self.yaw_left(1)
            while duration > time.time() - start:
                if self.client.simGetCollisionInfo(vehicle_name="SimpleFlight1").has_collided:
                    return True
            # self.stop()

        return collided

    def take_action_2(self, action):
        start = time.time()
        duration = 0
        collided = False
        if action == 0:
            start, duration = self.straight_2(1, 0.5)
            while duration > time.time() - start:
                if self.client_2.simGetCollisionInfo(vehicle_name="SimpleFlight2").has_collided:
                    return True
            # self.stop()

        if action == 1:
            start, duration = self.yaw_right_2(0.8)
            while duration > time.time() - start:
                if self.client_2.simGetCollisionInfo(vehicle_name="SimpleFlight2").has_collided:
                    return True
            # self.stop()

        if action == 2:
            start, duration = self.yaw_left_2(1)
            while duration > time.time() - start:
                if self.client_2.simGetCollisionInfo(vehicle_name="SimpleFlight2").has_collided:
                    return True
            # self.stop()

        return collided

    def goal_direction(self, goal, pos):
        pitch, roll, yaw = self.getPitchRollYaw()
        yaw = math.degrees(yaw)
        pos_angle = math.atan2(goal[1] - pos.y_val, goal[0] - pos.x_val)
        pos_angle = math.degrees(pos_angle) % 360
        track = math.radians(pos_angle - yaw)

        return ((math.degrees(track) - 180) % 360) - 180

    def goal_direction_2(self, goal, pos):
        pitch, roll, yaw = self.getPitchRollYaw_2()
        yaw = math.degrees(yaw)
        pos_angle = math.atan2(goal[1] - pos.y_val, goal[0] - pos.x_val)
        pos_angle = math.degrees(pos_angle) % 360
        track = math.radians(pos_angle - yaw)

        return ((math.degrees(track) - 180) % 360) - 180

    def get_state_from_sim(self, track, distance, now):
        front_dis_sensor = self.client.getDistanceSensorData(distance_sensor_name="Distance",
                                                             vehicle_name="SimpleFlight1").distance

        print('front dis sensor: %s' % front_dis_sensor)
        if front_dis_sensor > 4.5:
            front_dis_sensor = 4.0 + 0.4 * np.random.random(1)
        print('Position now = %s ' % {"x_pos": now.x_val, "y_pos": now.y_val})
        print('Track = %s ' % track)
        print('Distance to target: %s' % distance)
        print('Front distance: %s' % front_dis_sensor)
        if isinstance(front_dis_sensor, float):
            return now.x_val, now.y_val, track, distance, front_dis_sensor
        else:
            return now.x_val, now.y_val, track, distance, front_dis_sensor[0]

    def get_state_from_sim_2(self, track, distance, now):
        front_dis_sensor = self.client_2.getDistanceSensorData(distance_sensor_name="Distance",
                                                             vehicle_name="SimpleFlight2").distance

        print('front dis sensor_2: %s' % front_dis_sensor)
        if front_dis_sensor > 4.5:
            front_dis_sensor = 4.0 + 0.4 * np.random.random(1)
        print('Position now_2= %s ' % {"x_pos": now.x_val, "y_pos": now.y_val})
        print('Track_2 = %s ' % track)
        print('Distance to target_2: %s' % distance)
        print('Front distance_2: %s' % front_dis_sensor)
        if isinstance(front_dis_sensor, float):
            return now.x_val, now.y_val, track, distance, front_dis_sensor
        else:
            return now.x_val, now.y_val, track, distance, front_dis_sensor[0]

    def AirSim_reset(self):

        self.client.reset()
        time.sleep(0.2)
        self.client.enableApiControl(True,vehicle_name="SimpleFlight1")
        time.sleep(0.2)
        self.client.moveToZAsync(self.z, 3,vehicle_name="SimpleFlight1").join()
        time.sleep(0.2)

        self.client_2.enableApiControl(True,vehicle_name="SimpleFlight2")
        time.sleep(0.2)
        self.client_2.moveToZAsync(self.z, 3,vehicle_name="SimpleFlight2").join()
        time.sleep(0.2)

