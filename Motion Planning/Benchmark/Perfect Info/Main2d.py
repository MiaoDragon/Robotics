from Robot import Robot
from Model import Model
from World import World
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import numpy as np
import scipy as sp
# parameters
st = 1.0
t = 0.0
tmax = 1200.0
width = 300.0 # for two windows
height = 300.0
collide = False  # not colliding

limits = np.array([width, height])
goal = {'pos': np.array([0.0, 0.0]), 'ori': np.array([0.0]), 'np': np.array([0.0,0.0,0.0])}
robot = Robot({'v': np.array([[10.0,10.0],[-10.0,10.0],[-10.0,-10.0],[10.0,-10.0]])}, np.array([100.0, 100.0]))
#ob = Model({'v': np.array([[10.0,10.0],[-10.0,10.0],[-10.0,-10.0],[10.0,-10.0]])},np.array([200.0, 200.0]))
ob = Model({'v': np.array([[5.0,5.0],[-5.0,5.0],[-5.0,-5.0],[5.0,-5.0]])},np.array([50.0, 50.0]))

world = World(2, limits, [robot], [ob], goal)
# init figure
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)
ax = plt.axes(xlim=(-2*width, 2*width), ylim=(-2*height, 2*height))
fig.add_axes(ax)
patchR = plt.Polygon(robot.geo['v']+robot.pos, fill=True, color='r')
patchOs = []
patchO = plt.Polygon(ob.geo['v']+ob.pos, fill=True, color='b')
ax.add_patch(patchR)
ax.add_patch(patchO)
patchOs.append(patchO)
step = 0
def init():
    global step
    patchR.set_xy(robot.geo['v']+robot.pos)
    patchO.set_xy(ob.geo['v']+ob.pos)
    step = 0
    return [patchR, patchO]
def animate(i):
    #if not collide:
    global step
    velocity = robot.ai(goal, world, step * st, algo='sample')
    step += 1
    world.move(robot, velocity, st)
    patchR.set_xy(robot.wgeo['v'])
    patchO.set_xy(ob.wgeo['v'])
    return [patchR, patchO]

anim = animation.FuncAnimation(fig, animate, init_func=init, frames=360, interval=20, blit=False)
plt.show()
