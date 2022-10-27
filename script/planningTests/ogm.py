import numpy as np


class OGM:
    def __init__(self, width, height):
        self.OBSTACLE = 1
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

    def get_neighbours(self, pos: tuple, free_only=True):
        (x, y) = (round(pos[0]), round(pos[1]))
        neighbours = {(x + 1, y): 1, (x + 1, y + 1): 1.414, (x, y + 1): 1, (x - 1, y + 1): 1.414,
                      (x - 1, y): 1, (x - 1, y - 1): 1.414, (x, y - 1): 1, (x + 1, y - 1): 1.414}
        return {k: v for k, v in neighbours.items() if self.is_free(k)}

    def successors(self, pos: tuple):
        return self.get_neighbours(pos)
