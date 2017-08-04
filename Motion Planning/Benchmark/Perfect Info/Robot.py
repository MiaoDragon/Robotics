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

    def planner(self, goal, map, t):
        # motion planning algorithms for Robot from init to goal
        # now is pre plan
        if self.preplan != None:
            return self.preplan(t)
        else:
            # compute preplan
            pdif = goal['pos'] - self.pos
            odif = goal['ori'] - self.ori

            start = t
            tmax = 150.0  # seconds to finish the work
            def preplan(t):
                if t >= tmax:
                    return {'vp': pdif*0, 'vr': odif*0}
                else:
                    return {'vp': pdif/(tmax-start), 'vr': odif/(tmax-start)}
            self.preplan = preplan
        return self.preplan(t)
