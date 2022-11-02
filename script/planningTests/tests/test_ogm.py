from unittest import TestCase

import numpy as np

from ogm import OGM


class TestOGM(TestCase):

    def test_within_map(self):
        map = OGM(10, 20)
        self.assertTrue(map.within_map((0, 0)))

    def test_set_map(self):
        map = OGM(1, 1)
        map.set_map(np.zeros((100, 101), dtype=np.uint8))
        self.assertTrue((map.width == 100) and (map.height == 101))

    def test_get_map(self):
        map = OGM(10, 20)
        self.assertTupleEqual(map.get_map().shape, (10, 20))

    def test_set_obstacle(self):
        map = OGM(10, 20)
        map.set_obstacle((1, 1))
        self.assertTrue(map.is_obstacle((1, 1)))

    def test_set_free(self):
        map = OGM(10, 20)
        map.set_free((2, 2))
        self.assertTrue(map.is_free((2, 2)))

    def test_is_obstacle(self):
        self.test_set_obstacle()

    def test_is_free(self):
        self.test_set_free()

    def test_get_neighbours(self):
        map = OGM(10, 20)
        result1 = map.get_neighbours((5, 5))
        expected1 = {(6, 5): 1, (6, 6): 1.414, (5, 6): 1, (4, 6): 1.414, (4, 5): 1, (4, 4): 1.414, (5, 4): 1,
                     (6, 4): 1.414}
        self.assertDictEqual(result1, expected1)
        map.set_obstacle((6, 5))
        map.set_obstacle((4, 4))
        result2 = map.get_neighbours((5, 5))
        expected2 = {(6, 5): float('inf'), (6, 6): 1.414, (5, 6): 1, (4, 6): 1.414, (4, 5): 1, (4, 4): float('inf'), (5, 4): 1, (6, 4): 1.414}
        # expected2 = {(6, 6): 1.414, (5, 6): 1, (4, 6): 1.414, (4, 5): 1, (5, 4): 1, (6, 4): 1.414}
        self.assertDictEqual(result2, expected2)
        map.set_free((6, 5))
        map.set_free((4, 4))
        result3 = map.get_neighbours((5, 5))
        self.assertDictEqual(result3, expected1)
