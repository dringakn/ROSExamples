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
        self.start = QNode((0, 0), float('inf'))
        self.goal = QNode((0, 0), float('inf'))
        self.U = PQueue()  # Openlist
        self.V = dict()  # Closed/Visited also used for backtraking path
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
        while True:
            if len(self.U) == 0: # Is the pqueue empty?
                print(f"No path found!!!")
                return False

            u = self.U.pop()  # Pop the next smallest p-val from the pqueue
            self.V[u.key] = u  # Add the popped node to the visited set
            if u == self.goal:  # Is popped node is the goal?
                self.goal = u
                self._path_found = True
                print(f"Path found.")
                return True

            # Find the free neighbours of popped node
            for pos, cost in self.map.get_neighbours(u.pos).items():

                x = QNode(pos, float('inf'))
                if x.key in self.V:
                    continue  # Skip if neighbour is already visited

                x.g = u.g + cost  # Add cost to move to the neighbour
                x.p = x.g + x.dist(self.goal)  # Add the goal's heuristic cost from neighnour locaiton
                x.parent = u.key

                # Add the neighbour only if it doesn't exist
                # Otherwise, if the neighbour already exist, replace it only if new g-val is smaller then prior
                result = self.U.push_or_update(x)

    def reconstruct_path(self, node: QNode):
        path = [node.pos]
        while node.parent is not None:
            predecessor = self.V[node.parent]
            path.append(predecessor.pos)  # Append the path with the position value
            node = predecessor  # Get the next node for traversal

        path.reverse() # Reverse for start->goal instead of goal->start
        return path

    def get_path(self):
        if self._path_found:
            return self.reconstruct_path(self.goal)
        else:
            return []
