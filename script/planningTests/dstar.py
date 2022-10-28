"""
g(x): cost to move from start->current
h(x): estimated cost to move from current->goal
U: Open list, list of nodes to be examined, sorted by f(x)
V: Closed list, list of nodes which has been already visited, also contains parent info.
"""
from math import sqrt

from ogm import OGM
from pqueue import PQueue, QNode


class DStar:
    def __init__(self, width, height):
        self.map = OGM(width, height)
        self.start = QNode((0, 0))
        self.goal = QNode((0, 0))
        self.U = PQueue()  # Openlist
        self.V = set()  # Closedlist, Visited
        self.BP = dict() # Store parent(BackPointer) information
        self._path_found = False
        self.km = 0;
    def calculate_key(self, node: QNode):
        k = min(node.g, node.rhs)
        return (k + node.dist(self.start) + self.km, k)

    def update_vertex(self, u: QNode):
        if (u.g != u.rhs) and (u in self.U):
            u.p = self.calculate_key(u)
            self.U.update(u)
            # u.rhs = min([s.g + s.dist(u) for s in self.map.successors(u)])
        elif (u.g != u.rhs) and (u not in self.U):
            u.p = self.calculate_key(u)
            self.U.push(u)
        elif (u.g == u.rhs) and (u in self.U):
            self.U.remove(u)

    def compute_sortest_path(self):
        while (self.U.peek() < self.start) | (self.start.rhs > self.start.g):
            u = self.U.peek()
            k_old = u.p
            k_new = self.calculate_key(u)
            if u.g > u.rhs:
                u.g = u.rhs
                for s in Pred(u):
                    self.update_vertex(s)
            else:
                u.g = float('inf')
                for s in Pred(u):  # TODO: Pred(u)Union{u}
                    self.update_vertex(s)

    def set_start(self, start: QNode):
        self.start = start

    def set_goal(self, goal: QNode):
        self.goal = goal

    def DStar(self, start: QNode, goal: QNode):
        # Swap start and goal
        self.U.clear() # Clear the open list and cache
        self.V.clear()  # Clear the visited/closed list
        self.goal.rhs = 0
        self.goal.p = (self.goal.dist(self.start), 0)
        self.U.push(self.goal)
        self.compute_sortest_path()
        while start != self.goal:
            start = min([s.g + s.dist(start) for s in Succ(start)])
            # Move the robot to start
            # Scan the graph for change in edge-costs
            if True:  # if there is change in edge cost
