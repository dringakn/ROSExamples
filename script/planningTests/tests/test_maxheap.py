from random import randint, uniform
from unittest import TestCase

from maxheap import MaxHeap


class TestMaxHeap(TestCase):
    def test_push(self):
        mh = MaxHeap()
        for i in range(100000):
            item = (randint(-10000, 10000), randint(-10000, 10000))
            mh.push(item)
            self.assertEqual(mh.heap[mh.heap_idx[item]], item)

    def test_pop(self):
        mh = MaxHeap()
        for i in range(10000):
            mh.push((uniform(-10000, 10000), uniform(-10000, 10000)))

        last_val = (float('inf'), float('inf'))
        for i in range(len(mh.heap)):
            x = mh.pop()
            self.assertGreater(last_val, x)

    def test_peek(self):
        mh = MaxHeap()
        for i in range(10):
            mh.push(randint(-10000, 10000))

        self.assertGreaterEqual(mh.heap[0], mh.heap[1])
        self.assertGreaterEqual(mh.heap[0], mh.heap[2])

    def test_remove(self):
        mh = MaxHeap()
        for i in range(1000000):
            mh.push(randint(-100000, 100000))

        for i in range(100):
            mh.remove(randint(-100, 100))

        self.assertGreaterEqual(mh.heap[0], mh.heap[1])
        self.assertGreaterEqual(mh.heap[0], mh.heap[2])

        last_val = float('inf')
        print(f"Items: {len(mh)}")
        for i in range(len(mh)):
            item = mh.peek()
            self.assertEqual(mh.heap[mh.heap_idx[item]], item)
            x = mh.pop()
            self.assertGreater(last_val, x)

    def test_update(self):
        mh = MaxHeap()
        for i in range(1000000):
            mh.push(randint(-100000, 100000))

        for i in range(10000):
            mh.update(randint(-10000, 10000), randint(-10000, 10000))

        self.assertGreaterEqual(mh.heap[0], mh.heap[1])
        self.assertGreaterEqual(mh.heap[0], mh.heap[2])

        last_val = float('inf')
        print(f"Items: {len(mh)}")
        for i in range(len(mh)):
            item = mh.peek()
            self.assertEqual(mh.heap[mh.heap_idx[item]], item)
            x = mh.pop()
            self.assertGreater(last_val, x)
