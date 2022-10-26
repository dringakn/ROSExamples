"""
g(x): cost to move from start->current
h(x): estimated cost to move from current->goal
U: Open list, list of nodes to be examined, sorted by f(x)
V: Closed list, list of nodes which has been already visited, also contains parent info.
"""
from pqueue import PQueue, QNode
from ogm import OGM
from math import sqrt

class AStar:
    def __init__(self):
        self.map = OGM(0,0)
        self.start = QNode((0,0))
        self.goal = QNode((0,0))
        self.U = PQueue()  # Openlist
        self.V = set()  # Closedlist, Visited, is it really required due to keys??

    def set_start(self, start: QNode):
        self.start = start

    def set_goal(self, goal: QNode):
        self.goal = goal

    def set_map(self, map: OGM):
        self.map = map

    def _calculate_priority(self, node: QNode):
        return node.g + node.dist(self.goal)

    def search_path(self):
        self.U.clear()
        self.V.clear()
        self.start.g = 0
        self.start.p = self._calculate_priority(self.start)
        self.U.push(self.start)
        while True:
            if len(self.U) == 0:
                print(f"No path found!!!")
                break
            else:
                u = self.U.pop()
                # self.V.add(u) # Add the popped node to the visited

            if u == self.goal:
                print(f"Path found")
                break

            # Keep finding by checking neighbours of popped node
            for p, c in self.map.get_neighbours(u.pos).items():
                x = QNode(p)
                x.g = u.data.g + c  # Movement cost
                x.p = x.g + x.dist(self.goal)  # Heuristic cost
                if not self.U.contains(x):
                    self.U.push(x)  # Add the node only if not already visited
                else:
                    self.U.update(x, new_p)

