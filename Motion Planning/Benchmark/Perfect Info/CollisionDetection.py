import numpy as np
from scipy.optimize import linprog
class CollisionDetection:
    @classmethod
    def toclose(cls, p):
        # p: set of vertices in clockwise direction
        # make p close
        if any(p['v'][-1] != p['v'][0]):
            p['v'] = np.append(p['v'], np.array([p['v'][0]]), axis=0)
            return p
        return p

    @classmethod
    def getedges(cls, p):
        # p: set of vertices
        # return: set of edges as pair of vertices
        #         in clockwise direction
        #         with one inner vertex
        p = cls.toclose(p)
        edge = []
        for i in range(len(p['v'])-1):
            edge.append( (p['v'][i], p['v'][i+1], p['v'][(i+3)%len(p['v'])]) )
        return edge

    @classmethod
    def convexconvex(cls, p1, p2):
        # p1, p2: set of vertices in clockwise direction
        # p['v']:  vertices
        p1 = cls.toclose(p1)
        p2 = cls.toclose(p2)
        es1 = cls.getedges(p1)
        es2 = cls.getedges(p2)
        # each edge specifies one linear program
        # get the parameter for the edge
        lams = []
#        print('***********************')
        for e in es1+es2:
            # (y-y0)(x1-x0)=(y1-y0)(x-x0)
            yd = e[1][1]-e[0][1]
            xd = e[1][0]-e[0][0]
            lam = np.array([yd, -xd, -yd*e[0][0]+xd*e[0][1]])
            # ensure direction
            if np.dot(lam, np.append(e[2], 1)) > 0:
                lam = -lam
            lams.append(lam)
#            print(lam)
        # use lams to get the answer
        LARGE = 20.0
        c = np.array([1.0,1.0,1.0])
        A_ub = np.asarray(lams)
        b_ub = np.asarray([0] * len(lams))
        A_eq = np.array([[0, 0, 1]])
        b_eq = np.array([1])  # ensure x[2] = 1
        res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq)
        return res['success']

        @classmethod
        def incremental(cls, p):
            # to be implemented
            return True
