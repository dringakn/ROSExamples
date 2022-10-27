"""
g(x): cost to move from start->current
h(x): estimated cost to move from current->goal
U: Open list, list of nodes to be examined, sorted by f(x)
V: Closed list, list of nodes which has been already visited, also contains parent info.
"""
from ogm import OGM
from pqueue import PQueue, QNode


class AStar:
    def __init__(self):
        self.map = OGM(0, 0)
        self.start = QNode((0, 0))
        self.goal = QNode((0, 0))
        self.U = PQueue()  # Openlist
        self.V = set()  # Closed/Visited
        self.P = dict() # Store parent information
        self._path_found = False

    def set_start(self, start: QNode):
        self.start = start

    def set_goal(self, goal: QNode):
        self.goal = goal

    def set_map(self, map: OGM):
        self.map = map

    def _calculate_priority(self, node: QNode):
        return node.g + node.dist(self.goal)

    def search_path(self):
        self._path_found = False
        self.U.clear()
        self.V.clear()
        self.start.g = 0
        self.start.p = self.start.g + self._calculate_priority(self.start)
        self.U.push(self.start) # Add to the pqueue
        self.P[self.start.key] = (None, self.start.pos)  # Add to the parent list
        while True:
            if len(self.U) == 0: # Is the pqueue empty?
                print(f"No path found!!!")
                return False

            u = self.U.pop()  # Pop the next smallest p-val from the pqueue
            self.V.add(u.key)  # Add the popped node to the visited set
            if u == self.goal:  # Is popped node is the goal?
                self.goal = u
                self._path_found = True
                print(f"Path found.")
                return True

            # Find the free neighbours of popped node
            for pos, cost in self.map.get_neighbours(u.pos).items():

                x = QNode(pos)
                if x.key in self.V:
                    continue  # Skip if neighbour is already visited

                x.g = u.g + cost  # Add cost to move to the neighbour
                x.p = x.g + x.dist(self.goal)  # Add the goal's heuristic cost from neighnour locaiton

                # Add the neighbour only if it doesn't exist
                # Otherwise, if the neighbour already exist, replace it only if new g-val is smaller then prior
                result = self.U.push_or_update(x)
                if result != 0:
                    self.P[x.key] = (u.key, u.pos)  # If pushed or update, keep track of its parent.

    def reconstruct_path(self, node: QNode):
        path = [node.pos]
        predecessor = node.key
        while predecessor is not None:
            next = self.P[predecessor]
            if next[0] is None:
                break
            path.append(next[1])   # Append the path with the position value
            predecessor = next[0]  # Get the parent of the node

        path.reverse()
        return path

    def get_path(self):
        if self._path_found:
            return self.reconstruct_path(self.goal)
