from unittest import TestCase
from astar import AStar, QNode, OGM

class TestAStar(TestCase):
    def test_search_path(self):
        m = OGM(100, 100)
        m.set_obstacle((5, 5))
        astar = AStar()
        astar.set_map(m)
        astar.set_start(QNode((0, 0)))
        astar.set_goal(QNode((99, 99)))
        if astar.search_path():
            path = astar.get_path()
            print(f"Path length: {len(path)}")
            self.assertEqual(len(path), 101)
        else:
            self.fail()
