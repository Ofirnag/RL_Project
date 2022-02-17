import gym
from gym import spaces
from gym.utils import seeding
from gym_airsim.envs.myAirSimClient import *
from airsim.client import *
import random
logger = logging.getLogger(__name__)

airSimClient = myAirSimClient()


class AirSimEnv(gym.Env):

    def __init__(self):
        # x_pos, y_pos, direction to goal, distance to goal, distance_sensor.front
        self.observation_space = spaces.Box(low=-500, high=500, shape=(5,))
        self.state = np.zeros((5,), dtype=np.uint8)
        self.state_2 = np.zeros((5,), dtype=np.uint8)

        self.action_space = spaces.Discrete(3)
        # self.obstacle_list = [[0.0, -7.0], [0.0, 5.0], [10.0, -7.0], [10.0, -1.0], [10.0, 5.0], [5, -3.0], [5.0, 3.0]]
        # index = np.random.randint(0, len(self.goal_list) - 1)
        # self.goal = self.goal_list[index]  # global xy coordinates
        self.x_goal = random.uniform(-2, 16)
        self.y_goal = random.uniform(-10, 8)
        self.goal = [self.x_goal, self.y_goal]
        self.episodeN = 0
        self.stepN = 0
        self.dis = np.sqrt(np.power((self.goal[0] - airSimClient.home_pos.x_val), 2) +
                           np.power((self.goal[1] - airSimClient.home_pos.y_val), 2))

        self.dis_2 = np.sqrt(np.power((self.goal[0] - airSimClient.home_pos_2.x_val), 2) +
                           np.power((self.goal[1] - airSimClient.home_pos_2.y_val), 2))

        self.allLogs = {'reward': [0], 'distance': [self.dis], 'track': [-2], 'action': [1]}
        self.allLogs_2 = {'reward_2': [0], 'distance_2': [self.dis_2], 'track_2': [-2], 'action_2': [1]}
        # print(self.allLogs)

        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def computeReward(self, distance, alpha, beta):

        distance_now = distance
        distance_before = self.allLogs['distance'][-1]
        delta_distance = distance_now - distance_before
        if self.stepN > 8 and len(set(self.allLogs['distance'][len(self.allLogs['distance']):len(self.allLogs['distance']) - 8:-1])) == 1:
            beta = 1
        return 1000 * alpha - 100 * beta - 20 * delta_distance - 1


    def land(self):
        airSimClient.client.landAsync(vehicle_name="SimpleFlight1")
    def land_2(self):
        airSimClient.client_2.landAsync(vehicle_name="SimpleFlight2")

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        self.addToLog('action', action)

        self.stepN += 1
        collided = airSimClient.take_action(action)
        now = airSimClient.getPosition_1()
        # print('X % s' % now.x_val)
        track = airSimClient.goal_direction(self.goal, now)
        # print('track % s' % track)
        self.dis = np.sqrt(np.power((self.goal[0] - now.x_val), 2) + np.power((self.goal[1] - now.y_val), 2))
        alpha = 0
        beta = 0
        done = False
        if self.dis < 1:
            done = True
            # reward = 1000.0
            alpha = 1
        elif self.stepN >= 300:
            done = True
            # reward = -100.0
            beta = 1
        elif collided == True:
            done = True
            # reward = -100.0
            # self.success = False
            beta = 1
        reward = self.computeReward(self.dis, alpha, beta)
        # if collided:
        #     done = True
        #     reward = -100.0
        # elif collided == 99:
        #     done = True
        #     reward = 0.0
        # else:
        #     done = False
        #     reward = self.computeReward(self.dis)
        #
        # # You made it
        # if self.dis < 1:
        #     done = True
        #     reward = 100.0

        self.addToLog('reward', reward)
        self.addToLog('distance', self.dis)
        self.addToLog('track', track)
        info = {"x_pos": now.x_val, "y_pos": now.y_val}
        self.state = airSimClient.get_state_from_sim(track, self.dis, now)

        return self.state, reward, done, info

    def step_2(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        self.addToLog_2('action_2', action)

        self.stepN += 1
        collided = airSimClient.take_action_2(action)
        now_2 = airSimClient.getPosition_2()
        # print('X % s' % now_2.x_val)
        track = airSimClient.goal_direction_2(self.goal, now_2)
        # print('track % s' % track)
        self.dis_2 = np.sqrt(np.power((self.goal[0] - now_2.x_val), 2) + np.power((self.goal[1] - now_2.y_val), 2))
        alpha = 0
        beta = 0
        done = False
        if self.dis < 1:
            done = True
            # reward = 1000.0
            alpha = 1
        elif self.stepN >= 300:
            done = True
            # reward = -100.0
            beta = 1
        elif collided == True:
            done = True
            # reward = -100.0
            # self.success = False
            beta = 1
        reward = self.computeReward(self.dis_2, alpha, beta)
        # if collided:
        #     done = True
        #     reward = -100.0
        # elif collided == 99:
        #     done = True
        #     reward = 0.0
        # else:
        #     done = False
        #     reward = self.computeReward(self.dis_2)
        #
        # # You made it
        # if self.dis_2 < 1:
        #     done = True
        #     reward = 100.0

        self.addToLog_2('reward_2', reward)
        self.addToLog_2('distance_2', self.dis_2)
        self.addToLog_2('track_2', track)
        info = {"x_pos": now_2.x_val, "y_pos": now_2.y_val}
        self.state_2 = airSimClient.get_state_from_sim_2(track, self.dis_2, now_2)

        return self.state_2, reward, done, info

    def addToLog(self, key, value):
        if key not in self.allLogs:
            self.allLogs[key] = []
        self.allLogs[key].append(value)

    def addToLog_2(self, key, value):
        if key not in self.allLogs_2:
            self.allLogs_2[key] = []
        self.allLogs_2[key].append(value)


    def get_path(self):
        current_position = [0, 0, 0]
        pos = airSimClient.getPosition_2()
        current_position[0] = pos.x_val
        current_position[1] = pos.y_val
        current_position[2] = -pos.z_val
        return current_position

    def get_path_2(self):
        current_position = [0, 0, 0]
        pos = airSimClient.getPosition_2()
        current_position[0] = pos.x_val
        current_position[1] = pos.y_val
        current_position[2] = -pos.z_val
        return current_position

    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.

        # Returns
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """
        airSimClient.AirSim_reset()
        self.x_goal = random.uniform(-3, 17)
        self.y_goal = random.uniform(-11, 9)
        self.goal = [self.x_goal, self.y_goal]
        print('Goal = %s' % self.goal)
        now = airSimClient.getPosition_1()
        now_2 = airSimClient.getPosition_2()
        track = airSimClient.goal_direction(self.goal, now)
        track_2 = airSimClient.goal_direction_2(self.goal, now_2)
        self.dis = np.sqrt(np.power((self.goal[0] - now.x_val), 2) + np.power((self.goal[1] - now.y_val), 2))
        self.dis_2 = np.sqrt(np.power((self.goal[0] - now_2.x_val), 2) + np.power((self.goal[1] - now_2.y_val), 2))
        self.state = airSimClient.get_state_from_sim(track, self.dis, now)
        self.state_2 = airSimClient.get_state_from_sim_2(track_2, self.dis_2, now_2)
        self.stepN = 0
        self.episodeN += 1
        self.allLogs = {'reward': [0], 'distance': [self.dis], 'track': [track], 'action': [1]}
        self.allLogs_2 = {'reward_2': [0], 'distance_2': [self.dis_2], 'track_2': [track_2], 'action_2': [1]}
        return self.state, self.state_2
