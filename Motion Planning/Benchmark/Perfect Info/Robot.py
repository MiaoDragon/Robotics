import numpy as np
import scipy as sp
from Model import Model
from World import World
import copy
import queue
from CollisionDetection import CollisionDetection as clsdet
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
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
    def __init__(self, geo, pos, ori=np.array([0.0])):
        Model.__init__(self, geo, pos, ori)
        # preplan is a function
        # t -> [velocity vector, rotation vector] (normalized)
        self._preplan = None

    def distance(self, a, b):
        # a, b: np array
        return np.linalg.norm(a-b)

    def getGeo(self, c):
        T = np.array([[np.cos(c['ori'][0]),np.sin(c['ori'][0]),0],
                      [np.sin(c['ori'][0]),-np.cos(c['ori'][0]),0],
                      [c['pos'][0],c['pos'][1],1]])
        geo = {}
        geo['v'] = np.concatenate((self.geo['v'],np.ones((self.geo['v'].shape[0], 1))), axis=1)
        geo['v'] = np.dot(geo['v'], T)[:,0:len(geo['v'][0])-1]
        return geo

    def ai(self, goal, world, t, algo='naive'):
        world = self.perfectSensor(world)
        return self.planner(goal, world, t, algo)

    def perfectSensor(self, world):
        # get all info from the world
        # info includes: models, boundary
        # return the info got
        return world

    def planner(self, goal, world, t, algo='naive'):
        # motion planning algorithms for Robot from init to goal
        # now is pre plan
        if self.preplan == None:
            # construct preplan
            if algo == 'sample':
                self.preplan = self.samplingBased(goal, world, t)
            elif algo == 'naive':
                self.preplan = self.naiveSearch(goal, world, t)
        return self.preplan(t)

    def graphSearch(self, G, init, goal):
        gid = None
        print('in graphsearch')
        print('goal:  ' + str(goal['np']))
        for v in G['V']:
            print(str(v['np']))
            if all(np.isclose(v['np'],goal['np'])):
                gid = v['id']
        if not gid:
            print('goal not in')
            return None
        # A*
        # g: cost up to now
        # h: huristic cost
        h = list(map(lambda x: self.distance(goal['np'],x['np']), G['V']))
        g = [0] * len(G['V'])
        parent = [-1] * len(G['V'])
        parent[0] = 0
        check = [0] * len(G['V'])
        pq = queue.PriorityQueue(len(G['V'])+1)  # goal may not be inside G
        pq.put((h[0]+g[0], G['V'][0]))   # (priority, obj)
        t = 0
        notFind = True
        while t < len(G['V']) and notFind:
            (f, u) = pq.get()
            i = u['id']
            g[i] = f - h[i]
            check[i] = True
            for j in range(len(G['V'])):
                if G['E'][i][j] and not check[j]:
                    gnew = g[i] + self.distance(u['np'],G['V'][j]['np'])
                    # check if explored
                    if parent[j] != -1:
                        # explored
                        # check if it is a better solution
                        if gnew < g[j]:
                            # update
                            parent[j] = i
                            g[j] = gnew
                            pq.put((h[j]+g[j], G['V'][j]))
                        # else do nothing
                    else:
                        # not explored yet
                        g[j] = gnew
                        parent[j] = i
                        pq.put((h[j]+g[j], G['V'][j]))
                    # check if it is the goal
                    if j == gid:
                        notFind = False
                        break
        # start backtrack from goal
        vcur = gid
        path = []
        while vcur:
            path = [vcur] + path
            vcur = parent[vcur]
        # 0 is not put
        path = [0] + path
        return path

    def naiveSearch(self, goal, world, t):
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

    def samplingBased(self, goal, world, t):
        # search in the coordination space
        # directly search in the real world (shifted coordination space)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xlabel('theta')
        def init(self, t):
            # return the graph (V, E)
            # E has three possible representations: adjacent matrix or list, or set
            # here for simplicity, use adjacent matrix
            vi = {'pos': self.pos, 'ori': self.ori, 'np': np.append(self.pos, self.ori, axis=0),
                  'id': 0}
            vg = {'pos': goal['pos'], 'ori': goal['ori'], 'np': np.append(goal['pos'], goal['ori'], axis=0),
                  'id': 0}
            ax.scatter(vg['np'][0],vg['np'][1],vg['np'][2],c='r',marker='x')
            #V = [vi, vg]
            # show
            ax.scatter(vi['np'][0],vi['np'][1],vi['np'][2],c='b',marker='o')
            V = [vi]
            E = [[0]]
            G = {'V': V, 'E': E}
            return G

        def sample(self, world, i, mode='random'):
            if mode == 'random':
                # uniform sampling in [-limits, limits) * [0, 2*pi)
                # sample until not collide
                success = False
                while not success:
                    q = np.random.rand(3) * np.append(2*world.limits, 2*np.pi) - np.append(world.limits, 0)
                    q[2] = 0.0
                    success = True
                    # check if appear in graph
                    if len(list(filter(lambda v: all(np.isclose(v['np'],q)), G['V']))):
                        success = False
                    qnew = {'pos': q[0:2], 'ori': q[2:3], 'np': q}
                    print(qnew['np'])
                    for ob in world.obs:
                        if clsdet.convexconvex(self.getGeo(qnew), ob.wgeo):
                            print('fail')
                            success = False
                return {'pos': q[0:2], 'ori': q[2:3], 'np': q}
            elif mode == 'det':
                return True
        def vsm(self, G, qnew):
            # naive approach: select from vertex set
            V = G['V']
            qcur = V[0]
            minl = self.distance(qnew['np'],qcur['np'])
            for v in G['V']:
                d = self.distance(v['np'],qnew['np'])
                if d < minl:
                    minl = d
                    qcur = v
            return qcur

        def lpm(self, G, world, qcur, qnew):
            # find a path from qcur to qnew
            # avoid collision
            # use step size small enough to get the stopping config
            # here use the delta method, where delta is a parameter for tuning
            delta = 0.005
            dif = qnew['np'] - qcur['np']
            print('qcur: ' + str(qcur['np']))
            t = delta
            tmark = 1.0
            q = {}
            while t < 1:
                # find the last one that does not collide
                q['np'] = qcur['np'] + t*dif
                q['pos'] = q['np'][0:2]
                q['ori'] = q['np'][2:]
                for ob in world.obs:
                    if clsdet.convexconvex(self.getGeo(q), ob.wgeo):
                        tmark = t-delta
                if not np.isclose(tmark,1.0):
                    break
                t += delta
            print(tmark)
            if np.isclose(tmark,0):
                return G
            qnew['np'] = qcur['np'] + tmark*dif
            qnew['pos'] = qnew['np'][0:2]
            qnew['ori'] = qnew['np'][2:]
            # test if qnew repeats
            if len(list(filter(lambda v: all(np.isclose(v['np'],qnew['np'])), G['V']))):
                return G
            # insert into G
            qnew['id'] = len(G['V'])
            G['V'].append(qnew)
            G['E'] = [lst+[0] for lst in G['E']]
            G['E'].append([0]*len(G['V']))
            G['E'][qcur['id']][qnew['id']] = 1
            G['E'][qnew['id']][qcur['id']] = 1
            print('new node: ' + str(qnew['np']))

            ax.scatter(qnew['np'][0],qnew['np'][1],qnew['np'][2],c='b',marker='o')
            xs = np.array([qcur['np'][0],qnew['np'][0]])
            ys = np.array([qcur['np'][1],qnew['np'][1]])
            zs = np.array([qcur['np'][2],qnew['np'][2]])
            ax.plot(xs,ys,zs)
            return G
        def checkGoal(G, goal):
            return len(list(filter(lambda v: all(np.isclose(v['np'],goal['np'])), G['V'])))
        # plot the collision region

        # state space
        G = init(self, t)
        i = 0
        addGT = 20 #every 50 steps
        testT = 20 # every 100 steps
        termT = 40
        while True:
            if i % addGT == 0:
                # try adding goal inside
                qnew = {'pos': goal['pos'], 'ori': goal['ori'], 'np': np.append(goal['pos'], goal['ori'], axis=0)}
                print(qnew['np'])
            else:
                qnew = sample(self, world, i)

            # look for the nearest vertex in G
            qcur = vsm(self, G, qnew)
            # connect edge from qcur to qnew
            G = lpm(self, G, world, qcur, qnew)
            # check if goal is reachable
            plt.draw()
            plt.pause(1)
            #plt.clf()   clear figure
            if i % testT == 0:
                if checkGoal(G, goal):
                    print('should be end')
                    break
            i += 1
            if i > termT:
                break
        #anim = animation.FuncAnimation(fig, animate, init_func=init, frames=360, interval=20, blit=False)
        #plt.show()
        path = self.graphSearch(G, init, goal)
        if path == None:
            #no solution
            return self.naiveSearch(goal, world, t)
        # convert index path to vertex path
        path = list(map(lambda x: G['V'][x]['np'], path))
        print(path)
        # parameter of motion
        velocity = 1.0

        def preplan(t):
            print(t)
            dis = velocity * t
            i = 0
            while dis > 0.0 and i < len(path)-1:
                l = self.distance(path[i], path[i+1])
                if dis > l:
                    # delete one
                    i += 1
                    dis -= l
                else:
                    v = velocity * (path[i+1]-path[i])/l
                    vp = v[0:2]
                    vr = v[2:]
                    return {'vp': vp, 'vr': vr}
            return {'vp': 0, 'vr': 0}
        return preplan
