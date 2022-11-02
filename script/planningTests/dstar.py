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
        self.start = QNode((0, 0), (float('inf'), float('inf')))
        self.goal = QNode((0, 0), (float('inf'), float('inf')))
        self.U = PQueue()  # Open list
        self.LT = dict()  # Lookup table to store (g, rhs)
        self.path = []
        self.km = 0  # used for detecting modifications

    def __repr__(self):
        return f"{self.U.peek()}"

    def set_start(self, start: QNode):
        self.start = start

    def update_start(self, new_pos):
        new_pos = QNode(new_pos, (float('inf'), float('inf')))  # Create a node
        g, rhs = self._get_g_rhs(new_pos.key)  # Get the g and rhs values at new location
        new_pos.p = self._calculate_priority(g, rhs, new_pos.pos)  # Calculate new priority
        self.km = self._heuristic(self.start.pos, new_pos.pos)  # offset for heap re-ordring wrt last position
        self.start = new_pos  # Should we update the node in pqueue???

    def set_goal(self, goal: QNode):
        self.goal = goal

    def update_goal(self, new_pos):
        """
        This is somewhat of a hack, to change the position of the goal we
        first save all of the non-empty on the map, clear the map, move the
        goal, and re-add all of non-empty cells. Since most of these cells
        are not between the start and goal this does not seem to hurt
        performance too much. Also it free's up a good deal of memory we
        likely no longer use.

        :param new_pos:
        :return:
        """
        pass

    def set_map(self, _map: OGM):
        self.map = _map
        self.G = ogm.np.ones((_map.width, _map.height), dtype=ogm.np.uint8)*ogm.np.inf
        self.RHS = self.G.copy()

    def cost_uv(self, u, v):
        if self.map.is_obstacle(u) or self.map.is_obstacle(v):
            return float('inf')
        else:
            return self._heuristic(u, v)

    def update_map(self, vertices):
        # For all directed edges (u, v) with changed edge costs
        for pos, succ in vertices.items():  # Iterate over all changed cells
            v = QNode(pos, (float('inf'), float('inf')))
            v_g, v_rhs = self._get_g_rhs(v.key)
            for pos_, old_cost in succ.items():  # Explore neighbours of changed cell
                u = QNode(pos_, (float('inf'), float('inf')))  # Create as node
                u_g, u_rhs = self._get_g_rhs(u.key)  # Get g and rhs values from lookup table
                new_cost = self.cost_uv(pos, pos_)

                if old_cost > new_cost:  # if dist(v,u) < old_dist(v,u), cleared cell!!!
                    if u != self.goal:  # if neighbour is not the goal cell
                        u_rhs = min(u_rhs, v_g + new_cost)  # minimum of current rhs or new rhs
                        self._set_g_rhs(u, u_g, u_rhs)  # update u's g and rhs lookup table values

                elif u_rhs == v_g + old_cost:  # if current rhs is equal to old!!!
                    if u != self.goal:
                        u_rhs = self._min_successors(u) # minimum of u's neighbors
                        self._set_g_rhs(u, u_g, u_rhs)  # update u's g and rhs lookup table values

                self._process_node(u, u_g, u_rhs)

    def _heuristic(self, u, v):
        return sqrt((u[0] - v[0]) ** 2 + (u[1] - v[1]) ** 2)

    def _get_g_rhs(self, key):
        if key in self.LT:
            return self.LT[key]
        else:
            return float('inf'), float('inf') # g, rhs

    def _set_g_rhs(self, node: QNode, g, rhs):
        self.LT[node.key] = (g, rhs)
        self.G[node.pos] = g
        self.RHS[node.pos] = rhs

    def _calculate_priority(self, g, rhs, pos):
        tb = min(g, rhs)  # tie-beaker: min(node.g, node.rhs)
        return self._heuristic(pos, self.start.pos) + tb + self.km, tb

    def initialize(self):
        self.path = []
        self.U.clear()  # Clear the open list
        self.LT.clear()  # Clear the look-up table
        self.km = 0
        self.LT[self.goal.key] = (float('inf'), 0)  # set the initial value of goal (g, rhs)
        self.goal.p = self._calculate_priority(float('inf'), 0, self.goal.pos)  # [goal.dist(start), 0]
        self.U.push(self.goal)
        self.RHS[self.goal.pos] = 0

    def _process_node(self, u: QNode, g, rhs):
        if (g != rhs) and (u in self.U):
            u.p = self._calculate_priority(g, rhs, u.pos)
            self.U.update(u)
        elif (g != rhs) and (u not in self.U):
            u.p = self._calculate_priority(g, rhs, u.pos)
            self.U.push(u)
        elif (g == rhs) and (u in self.U):  # consistent node already present
            self.U.remove(u)  # Remove or pop?

    def _process_successors(self, node: QNode, n_g):
        for pos, cost in self.map.get_neighbours(node.pos).items():
            s = QNode(pos, (float('inf'), float('inf')))
            s_g, s_rhs = self._get_g_rhs(s.key)  # Get the g and rhs value
            if s != self.goal:
                s_rhs = min(s_rhs, cost + n_g)
                self._set_g_rhs(s, s_g, s_rhs)
            self._process_node(s, s_g, s_rhs)

    def _min_successors(self, node: QNode):
        min_s = float('inf')
        for pos, cost in self.map.get_neighbours(node.pos).items():
            s = QNode(pos, (float('inf'), float('inf')))
            s_g, s_rhs = self._get_g_rhs(s.key)  # Get the g and rhs value
            temp = cost + s_g
            if temp < min_s:
                min_s = temp

        return min_s

    def _compute_path(self):
        result = False
        while len(self.U):  # if open queue is not empty
            start_g, start_rhs = self._get_g_rhs(self.start.key)  # Get the g and rhs values for start node
            p_start = self._calculate_priority(start_g, start_rhs, self.start.pos)  # Calculate new priority based on new values
            u = self.U.peek()  # Get the smallest priority node from the open list
            if not ((u.p < p_start) or (start_rhs > start_g)):  # top node priority is less than start or starts is inconsistent
                result = True
                break

            g, rhs = self._get_g_rhs(u.key)  # Get the g and rhs values
            p_old = u.p  # Store previous priority
            p_new = self._calculate_priority(g, rhs, u.pos)  # Calculate new priority
            if p_old < p_new:  # u is out of date
                u.p = p_new  # update new priority
                self.U.update(u)  # update node

            elif rhs < g:  # needs update (better node, but over-consistent)
                g = rhs
                self._set_g_rhs(u, g, rhs)  # update the g-value to rhs
                self.U.pop()  # self.U.remove(u)  or self.U.pop()
                self._process_successors(u, g)  # Explore neighbors

            else:  # g <= rhs, state has got worse
                g_old = g  # store previous g-value
                g = float('inf')  # set new value
                self._set_g_rhs(u, g, rhs)  # update g-value
                pred = self.map.get_neighbours(u.pos)  # Find neighbours
                pred[u.pos] = 0  # Append current node: pred(u) UNION u

                for pos, cost in pred.items():  # Explore neighbours
                    s = QNode(pos, (float('inf'), float('inf')))
                    s_g, s_rhs = self._get_g_rhs(s.key)  # Get the g and rhs value
                    if s_rhs == g_old + cost:  # ???
                        if s != self.goal:  # if the current node is not a goal
                            s_rhs = self._min_successors(s)  # find the minimum rhs value from s's neighbours
                            self._set_g_rhs(s, s_g, s_rhs)  # update the g and rhs values of the current node

                    self._process_node(s, s_g, s_rhs)  # process the current node

        return result

    def re_plan(self, new_start_pos, changed_cells = None):
        self.update_start(new_start_pos)
        if changed_cells is not None:
            self.update_map(changed_cells)
        print(f"compute path: {self._compute_path()}")

    def get_path(self):

        self.path.clear()  # Clear previous contents
        if not self._compute_path():  # is path found?
            print(f"There is no path!!!")
            return self.path

        curr = self.start  # Start from start location as current
        s_g, s_rhs = self._get_g_rhs(curr.key)  # Get the g and rhs values of the current location
        if s_rhs == float('inf'):  # is it still infinity after search?
            print(f"There is no path!!!")
            return self.path

        while curr != self.goal:  # Start iterating unless we reached goal location
            self.path.append(curr.pos)  # Append current location to the path
            succ = self.map.get_neighbours(curr.pos)  # Explore neighbours of current node

            non_inf_values = 0
            for val in succ.values():
                if val != float('inf'):  # are there are neighbours?
                    non_inf_values += 1
            if non_inf_values == 0:
                print(f"There is no path!!!")
                return self.path
            if len(self.path) > (self.map.width * self.map.height):
                print(self.path)
                print(self.G)
                print(self.RHS)
                print('Error')
                break

            c_min = float('inf')  # minimum cost
            t_min = c_min  # used for straight line path
            s_min = None  # neighbour with minimum value
            for pos, cost in self.map.get_neighbours(curr.pos).items():  # Explore neighbours
                s = QNode(pos, (float('inf'), float('inf')))
                s_g, s_rhs = self._get_g_rhs(s.key)
                temp = s_g + cost
                temp2 = s.dist(self.start) + s.dist(self.goal)  # straight line path
                if temp < c_min:
                    c_min = temp
                    t_min = temp2
                    s_min = s
                elif temp == c_min:
                    if temp2 < t_min:
                        c_min = temp
                        t_min = temp2
                        s_min = s

            curr = s_min

        self.path.append(self.goal.pos)  # Append the remaining one
        return self.path
