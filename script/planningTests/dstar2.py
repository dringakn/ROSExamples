"""
g(x): cost to move from start->current
h(x): estimated cost to move from current->goal
U: Open list, list of nodes to be examined, sorted by f(x)
Note: There is no closed list
"""
from ogm import OGM, SQRT2, INF, OBSTACLE, FREE, np
from math import sqrt

OBSTACLE_COST = OBSTACLE  # -1
# Free can have one of the two values
FREE_COST1 = 1  # 1 means high low traverse-ability cost
FREE_COST2 = 1000  # 1000 means high low traverse-ability cost


class Node:
    def __init__(self, pos: (int, int)):
        self.pos = pos
        self.key = hash(self.pos)
        self.p = (INF, INF)

    def __eq__(self, other):
        return self.key == other.key

    def __ne__(self, other):
        return self.key != other.key

    def __gt__(self, other):
        if self.p[0] > other.p[0]:
            return True
        elif self.p[0] < other.p[0]:
            return False
        else:
            return self.p[1] > other.p[1]

    def __lt__(self, other):
        if self.p[0] < other.p[0]:
            return True
        elif self.p[0] > other.p[0]:
            return False
        else:
            return self.p[1] < other.p[1]

    def __repr__(self):
        return f"[{self.p[0]:0.2f},{self.p[1]:0.2f}]{self.pos}"


class NodeInfo:
    def __init__(self, g, rhs, cost):
        self.g = g  # node g-value
        self.rhs = rhs  # node one-step look ahead value
        self.cost = cost  # node travers-ability cost value

    def __repr__(self):
        return f"({self.g:0.2f}, {self.rhs:0.2f}, {self.cost})"


class Path:
    def __init__(self):
        self.pos = []  # List of locations
        self.LT = {}  # lookup table
        self.path_length = 0  # Length of the path
        self.prev = None

    def __repr__(self):
        return f"[{len(self.pos)}]:{self.path_length}"

    def __contains__(self, node):
        return node.key in self.LT

    def push(self, node):
        self.pos.append(node.pos)
        self.LT[node.key] = len(self.pos)
        if self.prev is None:
            self.path_length = 0
        else:
            self.path_length += sqrt((self.prev[0]-node.pos[0])**2 + (self.prev[1]-node.pos[1])**2)
        self.prev = node.pos

    def clear(self):
        self.pos.clear()  # Empty the location list
        self.LT.clear() # Empty the lookup table
        self.path_length = 0  # Length of the path
        self.prev = None


class PQueue:
    def __init__(self):
        self.heap = list()  # List of nodes in the list
        self.heap_idx = dict()  # For quick access, index of nodes in the heap based on node hash value.
        self.size = len(self.heap)

    def __repr__(self):
        return f"{self.heap}"

    def __len__(self):
        return self.size

    def __contains__(self, n: Node):
        return n.key in self.heap_idx

    def clear(self):
        self.heap.clear()
        self.heap_idx.clear()
        self.size = 0

    def peek(self):
        result = None
        if self.size:
            result = self.heap[0]
        return result

    def push(self, node: Node):
        self.heap.append(node)  # Add the node to the end of heap
        self.size += 1  # Increment the number of elements
        idx = self.size - 1  # zero offset, last element
        self.heap_idx[node.key] = idx  # Add the node key:id to the dictionary for faster access.
        self._shift_up(idx)  # Move it to the appropriate position and update heap_idx

    def pop(self):
        result = None
        n = self.size
        if n == 1:  # very last element
            result = self.heap.pop()  # Remove the last element
            self.heap_idx.pop(result.key)  # Remove also from cached dictionary
            self.size -= 1  # Decrement size
        elif n >= 2:  # Two or more elements
            self._swap(0, self.size - 1)  # swap first and last element
            result = self.heap.pop()  # Remove the last element
            self.heap_idx.pop(result.key)  # Remove it also from cache
            self.size -= 1  # Adjust the size
            self._shift_down(0)  # move it to appropriate position and update heap_idx
        return result

    def remove(self, node: Node):
        result = False
        if node in self:
            idx = self.heap_idx[node.key]
            if idx == self.size - 1:
                self.heap.pop()
                self.heap_idx.pop(node.key)
                self.size -= 1
            else:
                self._swap(idx, self.size - 1)
                self.heap.pop()
                self.heap_idx.pop(node.key)
                self.size -= 1
                self._shift_up(idx)  # The combination of up/down make heap invariant, instead of O(n)
                self._shift_down(idx)
            result = True
        return result

    def update(self, node: Node):
        # Assuming node is present
        index = self.heap_idx[node.key]  # Quickly find the index of the item
        if node < self.heap[index]:  # if new priority is less than existing priority
            self.heap[index] = node  # Update the node
            self._shift_up(index)  # Move it to appropriate place
        elif node > self.heap[index]:  # if the new priority is greater than existing priority?
            self.heap[index] = node  # Update the node
            self._shift_down(index)  # Move it to appropriate place

    def _shift_up(self, idx):
        parent = (idx - 1) >> 1
        while (parent >= 0) and (self.heap[parent] > self.heap[idx]):
            self._swap(parent, idx)
            idx = parent
            parent = (idx - 1) >> 1

    def _shift_down(self, idx):
        n = self.size
        while True:
            m_idx = idx  # Smaller child index
            left = (idx << 1) + 1
            right = left + 1
            if left < n:  # Left child exists
                if self.heap[m_idx] > self.heap[left]:  # parent is greater than left child
                    m_idx = left  # set minimum as left

            if right < n:  # Right child exists
                if self.heap[m_idx] > self.heap[right]:  # parent or left is greater then right
                    m_idx = right  # set minimum as right

            if m_idx != idx:
                self._swap(idx, m_idx)
            else:  # No more swap required
                break

            idx = m_idx

    def _swap(self, idx1, idx2):
        self.heap[idx1], self.heap[idx2] = self.heap[idx2], self.heap[idx1]
        self.heap_idx[self.heap[idx1].key], self.heap_idx[self.heap[idx2].key] = idx1, idx2


class DStar:
    def __init__(self):

        self.km = 0  # used for detecting modifications
        self.C1 = FREE_COST1  # Default Cost for a new cell
        self.maxSteps = 1000  # Number of node expansion before we give-up
        self.path = Path()  # Path object
        self.start = None  # start location
        self.goal = None  # goal location
        self.last = None  # last start location

        self.U = PQueue()  # Open list
        self.LT = dict()  # Lookup table to store grid-cell NodeInfo: key=Node.key, value=NodeInfo
        self.width = None
        self.height = None

    def __repr__(self):
        return f"{self.U.peek()}"

    def _make_new_cell(self, key):
        if key in self.LT:
            return

        self.LT[key] = NodeInfo(INF, INF, self.C1)

    def _get_g(self, key):
        if key not in self.LT:
            return INF

        return self.LT[key].g

    def _set_g(self, key, g):
        self._make_new_cell(key)
        self.LT[key].g = g

    def _get_rhs(self, key):
        if key not in self.LT:
            return INF

        return self.LT[key].rhs

    def _set_rhs(self, key, rhs):
        self._make_new_cell(key)
        self.LT[key].rhs = rhs

    def _get_node_info(self, key):
        """
        Return the node information stored in the look-up table.
        Note: if the node doesn't exist in the table, a default value is added in the table before return
        :param node: location (x, y).
        :return: NodeInfo structure
        """
        if key in self.LT:
            return self.LT[key]
        else:
            return NodeInfo(INF, INF, self.C1)  # node default g, rhs, cost values

    def _set_node_info(self, key, info: NodeInfo):
        """
        Create/Update the node information structure in the lookup table
        :param node: Location (x, y)
        :param info: Node info structure to be stored/updated
        :return: Nothing.
        """
        self.LT[key] = info

    def _cost(self, u: (int, int), v: (int, int)):
        """
         Returns the cost of moving from state u to state v. This could be
         either the cost of moving off state u or onto state v, we went with
         the former. This is also the 8-way cost
        :param u: first location (x, y)
        :param v: second location (x, y)
        :return: float
        """
        xd = abs(u[0] - v[0])
        yd = abs(u[1] - v[1])
        scale = 1

        if xd + yd > 1:
            scale = SQRT2

        key = hash(u)
        if key not in self.LT:
            return scale * self.C1

        if self.LT[key].cost < 0:
            return INF

        return scale * self.LT[key].cost

    def _euclidean(self, u: (int, int), v: (int, int)):
        """
         Returns the Euclidean distance of moving from state u to state v
        :param u: first location (x, y)
        :param v: second location (x, y)
        :return: float
        """
        x = u[0] - v[0]
        y = u[1] - v[1]
        return sqrt((x * x) + (y * y))

    def _heuristic(self, u: (int, int), v: (int, int)):
        """
         Returns the 8-way distance between state u and state v
        :param u: first location (x, y)
        :param v: second location (x, y)
        :return: float
        """
        min = abs(u[0] - v[0])
        max = abs(u[1] - v[1])
        if min > max:
            temp = min
            min = max
            max = temp
        return ((SQRT2 - 1.0) * min + max) * self.C1

    def _calculate_priority(self, node: Node):
        tb = min(self._get_g(node.key), self._get_rhs(node.key))  # tie-beaker: min(node.g, node.rhs)
        return tb + self._heuristic(node.pos, self.start.pos) + self.km, tb

    def initialize(self, start: (int, int), goal: (int, int)):
        """
        Initialize the DStar variables to default value
        :param start: Start location as (x,y) tuple
        :param goal: Goal location as (x,y) tuple
        :return: Nothing
        """
        self.path.clear()  # Clear the path list and path length
        self.U.clear()  # Clear the open list
        self.LT.clear()  # Clear the look-up table
        self.km = 0
        self.start = Node(start)
        self._set_node_info(self.start.key, NodeInfo(INF, INF, self.C1))
        self.start.p = self._calculate_priority(self.start)  # calculate priority before assignment
        self.last = self.start

        self.goal = Node(goal)
        self._set_node_info(self.goal.key, NodeInfo(INF, 0, self.C1))  # set the initial value of goal (g, rhs)
        self.goal.p = self._calculate_priority(self.goal)  # calculate priority before assignment
        self.U.push(self.goal)

    def _occupied(self, key):
        """
        Check if the pos is traversable/free or non-traversable/obstacle.
        Note: New/unknown locations are considered unoccupied. Occupied cells are marked with cost of -1
        :param pos: location with (x,y)
        :return: True if pos is non-traversable/obstacle, False otherwise
        """
        if key in self.LT:
            # Non-traversable/obstacle location has cost value of -1, traversable/free has 1 or 1000
            return self.LT[key].cost < 0
        else:
            return False  # unseen location is considered traversable

    def _is_consistent(self, node: Node):
        """
        Check if the specified node is consistent i.e. g == rhs
        :param node: Node with key information.
        :return: True if consistent, False otherwise
        """
        return self._get_g(node.key) == self._get_rhs(node.key)

    def get_path(self):
        """
        Retrieve the pre-computed path object.
        :return: Path object.
        """
        return self.path

    def update_map_node(self, pos: (int, int), cost: int):
        """
        Add a node to the lookup table if it's not the start or goal location.
        If the node already exists with same grid cell state(cost) then skip updating.
        Otherwise, process the node (add/update/remove from the open list).
        Node priority and rhs values shall be automatically calculated within process_node
        :param pos: location (x,y)
        :param cost: ogm.FREE or ogm.OBSTACLE
        :return: Nothing
        """
        node = Node(pos)
        if (node == self.start) or (node == self.goal):
            return

        if node.key in self.LT:
            if self.LT[node.key].cost == cost:
                return

        self._make_new_cell(node.key)
        self.LT[node.key].cost = cost
        self._process_node(node)

    def update_map(self, map: OGM):
        self.width, self.height = map.width, map.height
        for x in range(self.width):
            for y in range(self.height):
                self.update_map_node((x, y), map.map[(x, y)])

    def get_map(self):
        map = np.ones((self.width, self.height), dtype=np.int8) * FREE
        for x in range(self.width):
            for y in range(self.height):
                pos = (x,y)
                key = hash(pos)
                if key in self.LT:
                    map[pos] = self.LT[key].cost

        return map

    def get_G(self):
        map = np.ones((self.width, self.height), dtype=np.float) * INF
        for x in range(self.width):
            for y in range(self.height):
                pos = (x,y)
                map[pos] = self.LT[hash(pos)].g
        return map

    def get_RHS(self):
        map = np.ones((self.width, self.height), dtype=np.float) * INF
        for x in range(self.width):
            for y in range(self.height):
                pos = (x,y)
                map[pos] = self.LT[hash(pos)].rhs
        return map

    def _process_node(self, u: Node):
        if u != self.goal:  # if the current node isn't the goal
            rhs = self._min_successors(u)  # find minimum rhs value of current node's successors
            self._set_rhs(u.key, rhs)

        if (self._get_g(u.key) != self._get_rhs(u.key)) and (u in self.U):  # in-consistent node already present
            u.p = self._calculate_priority(u)
            self.U.update(u)
        elif (self._get_g(u.key) != self._get_rhs(u.key)) and (u not in self.U):  # in-consistent node not present
            u.p = self._calculate_priority(u)
            self.U.push(u)
        elif (self._get_g(u.key) == self._get_rhs(u.key)) and (u in self.U):  # consistent node already present
            self.U.remove(u)

    def _start_priority(self):
        self.start.p = self._calculate_priority(self.start)
        return self.start.p

    def _compute_path(self):
        """
        Main DStar function to process open list nodes.
        :return: 0 = Open list exhausted during processing.
                 1 = Empty open list,
                 2 = Max. # of iterations reached.
                 3 = Start node became consistent.
                 4 = top most node priority became higher than start node priority.
        """
        if len(self.U) == 0:
            return 1  # There is no node present in the priority queue

        k = 0
        # if open queue is not empty
        while (len(self.U) > 0) and (self.U.peek().p < self._start_priority()) or (not self._is_consistent(self.start)):
            k += 1
            if k > self.maxSteps:
                print(f"Maximum number of iterations reached.")
                return -1

            u = self.U.peek()  # Get the smallest priority node from the open list
            p_old = u.p  # Store previous priority
            p_new = self._calculate_priority(u)  # Calculate new priority

            if p_old < p_new:  # u is out of date
                u.p = p_new  # update new priority
                self.U.update(u)  # update node

            elif self._get_g(u.key) > self._get_rhs(u.key):  # needs update (better node, but over-consistent)
                self._set_g(u.key, self._get_rhs(u.key))
                self.U.pop()  # Remove peeked node
                self._process_predecessors(u)  # Explore neighbors predecessors

            else:  # g <= rhs, state has got worse
                self._set_g(u.key, INF)
                self._process_predecessors(u)
                self._process_node(u)  # predecessors(u) UNION u

        return 0

    def re_plan(self):

        self.path.clear()
        res = self._compute_path()
        if res < 0:
            print(f"Compute Path Result: {res}")
            self.path.path_length = 0
            return False

        if self._get_g(self.start.key) == INF:
            print(f"No path to the goal!!")
            self.path.path_length = 0
            return False

        curr = self.start
        while curr != self.goal:
            self.path.push(curr)
            successors = self._successors(curr)
            if len(successors) == 0:
                print(f"No path to the goal!!")
                self.path.path_length = 0
                return False

            c_min = INF
            t_min = INF
            s_min = None
            for s in successors:
                if self._occupied(s.key):
                    continue

                if s in self.path:
                    continue

                val = min(self._get_g(s.key), self._get_rhs(s.key)) + self._cost(curr.pos, s.pos)
                # val = self._get_g(s.key) + self._cost(curr, s_pos)
                val2 = self._euclidean(s.pos, self.goal.pos) + self._euclidean(s.pos, self.start.pos)
                if val != INF and val == c_min:
                    # Tiebreak, if current neighbour is equal to current best
                    # choose the neighbour that has the smallest t_min value
                    if val2 < t_min:
                        t_min = val2
                        c_min = val
                        s_min = s
                elif val < c_min:
                    # if next neighbour s is strictly lower cost than the
                    # current best, then set it to be the current best
                    t_min = val2
                    c_min = val
                    s_min = s

            if s_min is None:
                print(f"Replanning required, loop in the path at {curr.pos}: {self.path.pos}")
                print(self.get_RHS())
                return False

            curr = s_min

        self.path.push(self.goal)
        return True

    def update_start(self, new_pos: (int, int)):
        """
        Update the position of the robot, this does not force re-planning.
        :param new_pos:
        :return:
        """
        self.start = Node(new_pos)
        self.km += self._heuristic(self.last.pos, self.start.pos)  # Update km before calculate_priority
        self.start.p = self._calculate_priority(self.start)  # Update start node priority before copying to last
        self.last = self.start

    def _successors(self, node):
        """
        List of outgoing edges from a vertex.
        NOTE: If the vertex is an obstacle the movement cost to each of its neighbours is  an empty list
        :param pos: vertex position (x,y)
        :return: list of successor vertices
        """
        result = []
        if self._occupied(node.pos):
            return result

        (x, y) = node.pos
        x += 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            result.append(Node((x, y)))
        y += 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            result.append(Node((x, y)))
        x -= 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            result.append(Node((x, y)))
        x -= 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            result.append(Node((x, y)))
        y -= 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            result.append(Node((x, y)))
        y -= 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            result.append(Node((x, y)))
        x += 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            result.append(Node((x, y)))
        x += 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            result.append(Node((x, y)))

        return result

    def _predecessors(self, node):
        """
        List of incoming edges to a vertex.
        NOTE: Obstacle vertices are not added to the result
        :param pos: vertex position (x,y)
        :return: List with predecessors vertices
        """
        result = []
        (x, y) = node.pos
        x += 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            n = Node((x, y))
            if not self._occupied(n.key):
                result.append(n)
        y += 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            n = Node((x, y))
            if not self._occupied(n.key):
                result.append(n)
        x -= 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            n = Node((x, y))
            if not self._occupied(n.key):
                result.append(n)
        x -= 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            n = Node((x, y))
            if not self._occupied(n.key):
                result.append(n)
        y -= 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            n = Node((x, y))
            if not self._occupied(n.key):
                result.append(n)
        y -= 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            n = Node((x, y))
            if not self._occupied(n.key):
                result.append(n)
        x += 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            n = Node((x, y))
            if not self._occupied(n.key):
                result.append(n)
        x += 1
        if (0 <= x < self.width) and (0 <= y < self.height):
            n = Node((x, y))
            if not self._occupied(n.key):
                result.append(n)

        return result

    def _min_successors(self, node):
        """
        Find the minimum value (g(s_pos) + c(pos, s_pos)) for the location successors
        :param pos: location (x, y)
        :return: float
        """
        min_s = INF
        for s in self._successors(node):
            temp = self._get_g(s.key) + self._cost(node.pos, s.pos)
            if temp < min_s:
                min_s = temp

        return min_s

    def _process_predecessors(self, node):
        for p in self._predecessors(node):
            self._process_node(p)
