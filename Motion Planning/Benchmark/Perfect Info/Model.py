import numpy as np
import scipy as sp
import copy
class Model:
    # field
    @property
    def homotran(self):
        return self._homotran
    @homotran.setter
    def homotran(self, val):
        self._homotran = val
    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self, val):
        self._pos = val
    @property
    def ori(self):
        return self._ori
    @ori.setter
    def ori(self, val):
        self._ori = val
    @property
    def dim(self):
        return self._dim
    @dim.setter
    def dim(self, val):
        self._dim = val
    @property
    def geo(self):
        return self._geo
    @geo.setter
    def geo(self, val):
        self._geo = val
    @property
    def wgeo(self):
        return self._wgeo
    @wgeo.setter
    def wgeo(self, val):
        self._wgeo = val
    # functions
    def __init__(self, geo, pos, ori=np.array([0.0])):
        # init in the pos of the world
        # pos: 1D vector (the position of the coordinate origin)
        # geo: v (vertices)    --- 2d
        # geo: v, f(faces)     --- 3d
        self._pos = pos
        self._ori = ori
        self._dim = len(pos)
        self._geo = geo
        self._wgeo = copy.deepcopy(geo)
        # only handle translate
        self._wgeo['v'] = geo['v'] + pos
