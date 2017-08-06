import numpy as np
from Model import Model
from CollisionDetection import CollisionDetection as clsdet
class World:
    # field
    @property
    def dim(self):
        return self._dim
    @dim.setter
    def dim(self, val):
        self._dim = val
    @property
    def robots(self):
        return self._robots
    @robots.setter
    def robots(self, val):
        self._robots = val
    @property
    def obs(self):
        return self._obs
    @obs.setter
    def obs(self, val):
        self._obs = val
    @property
    def limits(self):
        return self._limits
    @limits.setter
    def limits(self, val):
        self._limits = val
    @property
    def goal(self):
        return self._goal
    @goal.setter
    def goal(self, val):
        self._goal = val
    # method
    def __init__(self, dim, limits, robots=[], obs=[], goal=None):
        # robots: list of robot
        # obs: list of obstacles
        # for limit l: boundary of [-l, l)
        # goal=None: random generate
        if dim != 2 and dim != 3:
            raise Exception("wrong dimension.")
        if dim != len(limits):
            raise Exception("wrong dimension for world limit.")

        self._dim = dim
        self._limits = limits
        self._robots = robots
        self._obs = obs
        if not goal:
            # random generate
            # first version: consider goal dimension and orientation of robot
            # 2d orientation: [theta]
            # 3d orientation: [theta1, theta2, theta3]
            self._goal = {}
            if dim == 2:
                theta = np.random.random_sample() * 2*np.pi
                self._goal['ori'] = [theta]
            else:
                thetas = np.random.random_sample( (3,) ) * 2*np.pi
                self._goal['ori'] = thetas
            self._goal['pos'] = np.multiply(np.random.random_sample( (dim,) ), 2*limits) - limits
        else:
            self._goal = goal

    def addRobot(self, robot):
        self.robots.append(robot)

    def addOb(self, ob):
        self.obs.append(ob)

    def move(self, robot, velocity, st):
        # model moves for velocity during st time
        robot.pos += velocity['vp'] * st
        robot.ori += velocity['vr'] * st
        T = np.array([[np.cos(robot.ori[0]),np.sin(robot.ori[0]),0],
                      [np.sin(robot.ori[0]),-np.cos(robot.ori[0]),0],
                      [robot.pos[0],robot.pos[1],1]])
        # append one to each row
        geo = np.concatenate((robot.geo['v'],np.ones((robot.geo['v'].shape[0], 1))), axis=1)
        robot.wgeo['v'] = np.dot(geo, T)[:,0:len(geo[0])-1]
        for ob in self.obs:
            if clsdet.convexconvex(robot.wgeo, ob.wgeo):
                print('colliding')
                return False
        return True
