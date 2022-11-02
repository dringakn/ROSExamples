"""
g(x): cost to move from start->current
h(x): estimated cost to move from current->goal
U: Open list, list of nodes to be examined, sorted by f(x)
Note: There is no closed list
TODO: What about BP?
"""
import ogm
from ogm import OGM
from pqueue import PQueue, QNode, sqrt


class DStar:
    def __init__(self):
        self.map = OGM(0, 0)
        self.start = QNode((0, 0), (ogm.INF, ogm.INF))
        self.goal = QNode((0, 0), (ogm.INF, ogm.INF))
        self.U = PQueue()  # Open list
        self.LT = dict()  # Lookup table to store (g, rhs)
        self.path = []
        self.km = 0  # used for detecting modifications

    def __repr__(self):
        return f"{self.U.peek()}"

    def set_start(self, start: QNode):
        self.start = start

    def set_goal(self, goal: QNode):
        self.goal = goal

    def set_map(self, _map: OGM):
        self.map = _map
        self.G = ogm.np.ones((_map.width, _map.height), dtype=ogm.np.uint8)*ogm.np.inf  # TODO: delete
        self.RHS = self.G.copy()  # TODO: delete

    def _get_g_rhs(self, key):
        if key in self.LT:
            return self.LT[key]
        else:
            return ogm.INF, ogm.INF # g, rhs

    def _set_g_rhs(self, node: QNode, g, rhs):
        self.LT[node.key] = (g, rhs)
        self.G[node.pos] = g  # TODO: delete
        self.RHS[node.pos] = rhs  # TODO: delete

    def _calculate_priority(self, node: QNode):
        g, rhs, = self._get_g_rhs(node.key)
        tb = min(g, rhs)  # tie-beaker: min(node.g, node.rhs)
        return self.map.move_cost(node.pos, self.start.pos) + tb + self.km, tb

    def initialize(self):
        self.path = []
        self.U.clear()  # Clear the open list
        self.LT.clear()  # Clear the look-up table
        self.km = 0
        self.LT[self.goal.key] = (ogm.INF, 0)  # set the initial value of goal (g, rhs)
        self.goal.p = self._calculate_priority(self.goal)  # [goal.dist(start), 0]
        self.U.push(self.goal)
        self.RHS[self.goal.pos] = 0 # TODO: delete

    def _process_node(self, u: QNode):
        g, rhs = self._get_g_rhs(u.key)
        if (g != rhs) and (u in self.U):
            u.p = self._calculate_priority(u)
            self.U.update(u)
        elif (g != rhs) and (u not in self.U):
            u.p = self._calculate_priority(u)
            self.U.push(u)
        elif (g == rhs) and (u in self.U):  # consistent node already present
            self.U.remove(u)  # Remove or pop?

    def _process_successors(self, node: QNode):
        n_g, n_rhs = self._get_g_rhs(node.key)
        for pos, cost in self.map.get_successors(node.pos).items():
            s = QNode(pos, (ogm.INF, ogm.INF))
            if s != self.goal:
                s_g, s_rhs = self._get_g_rhs(s.key)  # Get the g and rhs value
                s_rhs = min(s_rhs, cost + n_g)
                self._set_g_rhs(s, s_g, s_rhs)
            self._process_node(s)

    def _process_predecessors(self, node: QNode):
        n_g, n_rhs = self._get_g_rhs(node.key)
        for pos, cost in self.map.get_predecessors(node.pos).items():
            s = QNode(pos, (ogm.INF, ogm.INF))
            if s != self.goal:
                s_g, s_rhs = self._get_g_rhs(s.key)  # Get the g and rhs value
                s_rhs = min(s_rhs, cost + n_g)
                self._set_g_rhs(s, s_g, s_rhs)
            self._process_node(s)

    def _min_successors(self, node: QNode):
        min_s = ogm.INF
        for pos, cost in self.map.get_successors(node.pos).items():
            s = QNode(pos, (ogm.INF, ogm.INF))
            s_g, s_rhs = self._get_g_rhs(s.key)  # Get the g and rhs value
            temp = cost + s_g
            if temp < min_s:
                min_s = temp

        return min_s

    def _compute_path(self):
        result = False
        while len(self.U):  # if open queue is not empty
            u = self.U.peek()  # Get the smallest priority node from the open list
            p_start = self._calculate_priority(self.start)  # Calculate new priority based on new values
            start_g, start_rhs = self._get_g_rhs(self.start.key)  # Get the g and rhs values for start node
            if not ((u.p < p_start) or (start_rhs > start_g)):  # top node priority is less than start or starts is inconsistent
                result = True
                break

            p_old = u.p  # Store previous priority
            p_new = self._calculate_priority(u)  # Calculate new priority
            u_g, u_rhs = self._get_g_rhs(u.key)  # Get the g and rhs values
            if p_old < p_new:  # u is out of date
                u.p = p_new  # update new priority
                self.U.update(u)  # update node

            elif u_g > u_rhs:  # needs update (better node, but over-consistent)
                u_g = u_rhs
                self._set_g_rhs(u, u_g, u_rhs)  # update the g-value to rhs
                self.U.pop()  # self.U.remove(u)  or self.U.pop()
                self._process_successors(u)  # Explore neighbors predecessors or successors

            else:  # g <= rhs, state has got worse
                g_old = u_g  # store previous g-value
                u_g = ogm.INF  # set new value
                self._set_g_rhs(u, u_g, u_rhs)  # update g-value
                pred = self.map.get_successors(u.pos)  # Find neighbours, predecessors or successors
                pred[u.pos] = self.map.move_cost(u.pos, u.pos)  # Append current node: pred(u) UNION u

                for pos, cost in pred.items():  # Explore neighbours
                    s = QNode(pos, (ogm.INF, ogm.INF))
                    s_g, s_rhs = self._get_g_rhs(s.key)  # Get the g and rhs value
                    if s_rhs == g_old + cost:  # ???
                        if s != self.goal:  # if the current node is not a goal
                            s_rhs = self._min_successors(s)  # find the minimum rhs value from s's neighbours
                            self._set_g_rhs(s, s_g, s_rhs)  # update the g and rhs values of the current node

                    self._process_node(u)  # process the current node

        return result

    def update_map(self, vertices):
        # For all directed edges (u, v) with changed edge costs
        for pos, pred in vertices.items():  # Iterate over all changed cells
            v = QNode(pos, (ogm.INF, ogm.INF))
            v_g, v_rhs = self._get_g_rhs(v.key)
            for pos_, old_cost in pred.items():  # Explore neighbours of changed cell
                u = QNode(pos_, (ogm.INF, ogm.INF))  # Create as node
                u_g, u_rhs = self._get_g_rhs(u.key)  # Get g and rhs values from lookup table
                new_cost = self.map.move_cost(pos, pos_)

                if old_cost > new_cost:  # if dist(v,u) < old_dist(v,u), obstacle cell cleared !
                    if u != self.goal:  # if neighbour is not the goal cell
                        u_rhs = min(u_rhs, v_g + new_cost)  # minimum of current rhs or new rhs
                        self._set_g_rhs(u, u_g, u_rhs)  # update u's g and rhs lookup table values

                elif u_rhs == v_g + old_cost:  # if current rhs is equal to old, obstacle will remain obstacle!!!
                    if u != self.goal:
                        u_rhs = self._min_successors(u) # minimum of u's neighbors
                        self._set_g_rhs(u, u_g, u_rhs)  # update u's g and rhs lookup table values

                self._process_node(u)

    def re_plan(self, new_start_pos, changed_cells = None):

        self.path.clear()  # Clear previous contents if any.
        self.path.append(new_start_pos)  # Add start location to the list
        new_pos = QNode(new_start_pos, (ogm.INF, ogm.INF))  # Create a node
        new_pos.p = self._calculate_priority(new_pos)  # Calculate priority
        self.start = new_pos  # update start location
        self.last = self.start # Save start location to be used for calculating km
        if not self._compute_path():  # Run to update pqueue and costs containers (g, rhs).
            print(f"There is no path!!!")
            return self.path

        while self.start != self.goal: # Start iterating unless we reached goal location
            s_g, s_rhs = self._get_g_rhs(self.start.key)  # Get the g and rhs values of the current location
            if s_rhs == ogm.INF:  # is it still infinity after search?
                print(f"There is no path!!!")
                return self.path

            succ = self.map.get_successors(self.start.pos)  # Explore neighbours of current location
            if self.count_valid_successors(succ) == 0:
                print(f"There is no path!!!")
                return self.path

            if len(self.path) > (self.map.width * self.map.height):
                print("G:\n", self.G)
                print("RHS:\n", self.RHS)
                print('Error')
                print("Path:", self.path)
                break

            self.start = self.find_nearest_neighbour(succ)  # Find next location
            self.path.append(self.start.pos)  # Add to path list

            if changed_cells is not None:
                self.km = 0 # TODO: delete
                # self.km += heuristic(self.s_last, self.s_start)
                self.last = self.start
                self.update_map(changed_cells)
                changed_cells = None  # Clear to stop incorporating again

            self._compute_path()  # should it be indented inside if???

        return self.path

    def find_nearest_neighbour(self, succ):
        c_min = ogm.INF  # minimum cost
        s_min = None  # neighbour with minimum value
        for pos, cost in succ.items():  # Explore neighbours
            s = QNode(pos, (ogm.INF, ogm.INF))
            s_g, s_rhs = self._get_g_rhs(s.key)
            temp = s_g + cost
            if temp < c_min:
                c_min = temp
                s_min = s
        return s_min

    def count_valid_successors(self, succ):
        non_inf_values = 0
        for val in succ.values():
            if val != ogm.INF:  # are there are neighbours?
                non_inf_values += 1
        return non_inf_values

