import numpy as np
import math

np.set_printoptions(precision=2)

INF = float('inf')
SQRT2 = 1.414213562

class OGM:
    def __init__(self, width, height):
        self.OBSTACLE = 255
        self.FREE = 0
        self.width = width
        self.height = height
        self.map = np.zeros((self.width, self.height), dtype=np.uint8)

    def __repr__(self):
        return f"{self.map.astype(int)}"

    def within_map(self, pos: tuple):
        (x, y) = (round(pos[0]), round(pos[1]))
        return (0 <= x < self.width) and (0 <= y < self.height)

    def set_map(self, map: np.array):
        self.map = map
        self.width = self.map.shape[0]
        self.height = self.map.shape[1]

    def get_map(self):
        return self.map

    def set_obstacle(self, pos: tuple):
        """
        Set the map grid cell as obstacle
        :param pos: (x,y)
        :return: True if pos is within map, False otherwise
        """
        if self.within_map(pos):
            self.map[round(pos[0]), round(pos[1])] = self.OBSTACLE
            return True
        else:
            return False

    def set_free(self, pos: tuple):
        """
        Set the map grid cell as free
        :param pos: (x,y)
        :return: True if pos is within map, False otherwise
        """
        if self.within_map(pos):
            self.map[round(pos[0]), round(pos[1])] = self.FREE
            return True
        else:
            return False

    def is_obstacle(self, pos: tuple):
        """
        Check if the specified grid cell is occupied.
        :param pos: (x,y)
        :return: True if Occupied/Obstacle and within map, False otherwise
        """
        if self.within_map(pos):
            return self.map[round(pos[0]), round(pos[1])] == self.OBSTACLE
        else:
            return False

    def is_free(self, pos: tuple):
        """
        Check if the specified grid cell is free.
        :param pos: (x,y)
        :return: True if UnOccupied/Free and within map, False otherwise
        """
        if self.within_map(pos):
            return self.map[round(pos[0]), round(pos[1])] == self.FREE
        else:
            return False

    def move_cost(self, u: tuple, v: tuple):
        if self.is_obstacle(u) or self.is_obstacle(v):
            return INF
        else:
            return math.sqrt((u[0] - v[0]) ** 2 + (u[1] - v[1]) ** 2)

    def get_neighbours(self, pos: tuple):
        """
        Get neighbouring vertices of a location, which are within map bounds.
        :param pos: (x,y) locaiton
        :return: List of valid vertices along with the movement cost.
        """
        (x, y) = (round(pos[0]), round(pos[1]))
        neighbours = {(x + 1, y): 1, (x + 1, y + 1): SQRT2, (x, y + 1): 1, (x - 1, y + 1): SQRT2,
                      (x - 1, y): 1, (x - 1, y - 1): SQRT2, (x, y - 1): 1, (x + 1, y - 1): SQRT2}
        return {k: v for k, v in neighbours.items() if self.within_map(k)}


    def get_successors(self, pos: tuple):
        """
        List of outgoing edges from a vertex.
        NOTE: If the vertex is an obstacle the movement cost to each of its neighbours is INFINITY.
        The movement cost from a free vertex to an neighbouring obstacle vertex is also INFINITY.
        :param pos: vertex
        :return: Dictionary with list of successor vertices and movement cost
        """
        (x, y) = (round(pos[0]), round(pos[1]))
        if self.is_obstacle((x,y)):
            # return {}
            neighbours = {(x + 1, y): INF, (x + 1, y + 1): INF, (x, y + 1): INF, (x - 1, y + 1): INF,
                          (x - 1, y): INF, (x - 1, y - 1): INF, (x, y - 1): INF, (x + 1, y - 1): INF}
        else:
            neighbours = {(x + 1, y): 1, (x + 1, y + 1): SQRT2, (x, y + 1): 1, (x - 1, y + 1): SQRT2,
                          (x - 1, y): 1, (x - 1, y - 1): SQRT2, (x, y - 1): 1, (x + 1, y - 1): SQRT2}

        return {k: (INF if (self.map[k] == self.OBSTACLE) else v) for k, v in neighbours.items() if self.within_map(k)}
        # return {k: v for k, v in neighbours.items() if self.is_free(k)}

    def get_predecessors(self, pos: tuple):
        """
        List of incoming edges to a vertex.
        NOTE: The movement cost from an obstacle vertex to pos is INFINITY.
        :param pos: vertex
        :return: Dictionary with list of predecessors vertices and movement cost
        """
        (x, y) = (round(pos[0]), round(pos[1]))
        if self.is_obstacle((x,y)):
            # return {}
            neighbours = {(x + 1, y): INF, (x + 1, y + 1): INF, (x, y + 1): INF, (x - 1, y + 1): INF,
                          (x - 1, y): INF, (x - 1, y - 1): INF, (x, y - 1): INF, (x + 1, y - 1): INF}
        else:
            neighbours = {(x + 1, y): 1, (x + 1, y + 1): SQRT2, (x, y + 1): 1, (x - 1, y + 1): SQRT2,
                          (x - 1, y): 1, (x - 1, y - 1): SQRT2, (x, y - 1): 1, (x + 1, y - 1): SQRT2}

        return {k: (INF if (self.map[k] == self.OBSTACLE) else v) for k, v in neighbours.items() if self.within_map(k)}
        # return {k: v for k, v in neighbours.items() if self.is_free(k)}
