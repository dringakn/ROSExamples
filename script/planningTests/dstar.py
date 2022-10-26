"""
g(x): cost to move from start->current
h(x): estimated cost to move from current->goal
U: Open list, list of nodes to be examined, sorted by f(x)
V: Closed list, list of nodes which has been already visited, also contains parent info.
"""
from pqueue import PQueue
from ogm import OGM
from math import sqrt

class DStarNode:
    def __init__(self, x, y):
        self.g = float('inf')
        self.rhs = float('inf')
        self.x = x
        self.y = y
        self.key = hash((self.x, self.y))

    def __repr__(self):
        return f"pos: ({self.x}, {self.y}), g: {self.g}, rhs: {self.rhs}"

    def dist(self, other):
        return sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

    def __eq__(self, other):
        # return (self.x, self.y) == (other.x, other.y)
        return self.key == other.key

    def __hash__(self):
        return self.key # Create hash for tuple

class DStar:
    def __init__(self, width, height):
        self.map = OGM(width, height)
        self.start = DStarNode(0, 0)
        self.goal = DStarNode(0, 0)
        self.U = PQueue()  # Openlist
        self.V = set()  # Closedlist, Visited

    def CalculatePriority(self, ds_node):
        k = min(ds_node.g, ds_node.rhs)
        return (k + ds_node.dist(self.start), k)

    def UpdateVertex(self, u):
        if u != self.goal and self.U.contains(u):
            u.rhs = min([s.g+s.dist(u) for s in self.map.successors(u)])
        if u not in self.V:
            self.V[u] = True  # TODO: Use key instead of (x,y)
        if u.g != u.rhs:
            self.U.push((self.CalculatePriority(u), u))

    def ComputeSortestPath(self):
        while (self.start.rhs != self.start.g) | (self.U.queue[0][0] < self.CalculatePriority(self.start)):
            u = self.U.pop()
            if u.g > u.rhs:
                u.g = u.rhs
                for s in Pred(u):
                    self.UpdateVertex(s)
            else:
                u.g = float('inf')
                for s in Pred(u): # TODO: Pred(u)Union{u}
                    self.UpdateVertex(s)

    def SetStart(self, start):
        self.start = start

    def SetGoal(self, goal):
        self.goal = goal

    def DStar(self, start, goal):
        self.U.clear() # Clear the open list
        self.V.clear() # Clear the closed list
        # TODO: the g and rhs should be set to 'inf'
        self.start.rhs = 0
        self.U.push((self.CalculatePriority(self.start), self.start))
        self.ComputeSortestPath()
        while start != self.goal:
            start = min([s.g+s.dist(start) for s in Succ(start)])
            # Move the robot to start
            # Scan the graph for change in edge-costs
            if True:  # if there is change in edge cost
