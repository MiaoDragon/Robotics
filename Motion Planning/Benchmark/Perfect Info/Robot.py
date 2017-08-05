import numpy as np
import scipy as sp
from Model import Model
from World import World
class Robot(Model):
    # field
    @property
    def preplan(self):
        return self._preplan
    @preplan.setter
    def preplan(self, val):
        self._preplan = val
    @property
    def ori(self):
        return self._ori
    @ori.setter
    def ori(self, val):
        self._ori = val
    # methods
    def __init__(self, pos, geo):
        Model.__init__(self, pos, geo)
        # preplan is a function
        # t -> [velocity vector, rotation vector] (normalized)
        self._preplan = None
        if self.dim == 2:
            self._ori = np.array([0.0])
        else:
            self._ori = np.array([0.0,0.0,0.0])

    def perfectSensor(self, world):
        # get all info from the world
        # info includes: models, boundary
        # return the info got
        return {'models': world.models, 'limits': world.limits}

    def planner(self, goal, space, t, algo='naive'):
        # motion planning algorithms for Robot from init to goal
        # now is pre plan
        if self.preplan == None:
            # construct preplan
            if algo == 'sample':
                self.preplan = self.samplingBased(goal, space, t)
            elif algo == 'naive':
                self.preplan = self.naiveSearch(goal, space, t)
        return self.preplan(t)

    def naiveSearch(self, goal, space, t):
        pdif = goal['pos'] - self.pos
        odif = goal['ori'] - self.ori
        start = t
        tmax = 150.0  # seconds to finish the work
        def preplan(t):
            # parameters are visible here
            if t >= tmax:
                return {'vp': pdif*0, 'vr': odif*0}
            else:
                return {'vp': pdif/(tmax-start), 'vr': odif/(tmax-start)}
        return preplan

    def samplingBased(self, goal, space, t):
        def init():
            return True
        def sample():
            return True
        def vsm():
            return True
        def lpm():
            return True
        # state space
        G = init()
        while True:
            qnew = sample()
            # look for the nearest vertex in G
            qcur = vsm(G, qnew)
            # connect edge from qcur to qnew
            G = lpm(G, qcur, qnew)
        self.GraphSearch(G, init, goal)
        return True
