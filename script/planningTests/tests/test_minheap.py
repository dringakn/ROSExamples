from random import randint, uniform, choice
from unittest import TestCase

from minheap import MinHeap

class TestMinHeap(TestCase):

    def test_push(self):
        mh = MinHeap()
        while len(mh) < 100000:
            n = (randint(-10000, 10000), randint(-10000, 10000))
            mh.push(n)
            self.assertEqual(mh.heap[mh.heap_idx[n]], n)  # verify indices

        self.assertTrue(len(set(mh.heap_idx.values())) == len(mh.heap_idx)) # No duplicated index
        self.assertTrue(mh._is_heap())  # check if it's a min-heap

        print(f"test_push: {len(mh)}")

    def test_pop(self):
        mh = MinHeap()
        while len(mh) < 100000:
            n = uniform(-10000, 10000)
            mh.push(n)

        self.assertTrue(len(set(mh.heap_idx.values())) == len(mh.heap_idx)) # No duplicated index
        self.assertTrue(mh._is_heap())

        print(f"test_pop: {len(mh)}")
        last_val = float('-inf')
        for i in range(len(mh.heap)):
            x = mh.pop()
            self.assertLessEqual(last_val, x)
            last_val = x

    def test_peek(self):
        mh = MinHeap()
        for i in range(100):
            mh.push(randint(-10000, 10000))

        self.assertTrue(len(set(mh.heap_idx.values())) == len(mh.heap_idx)) # No duplicated index

        last_val = float('-inf')
        for i in range(len(mh.heap)):
            x = mh.pop()
            self.assertLessEqual(last_val, x)
            last_val = x

    def test_remove(self):
        mh = MinHeap()
        while len(mh) < 200000:
            mh.push(randint(-100000, 100000))

        print(f"test_remove: {len(mh)}")
        for i in range(10000):
            x = choice(mh.heap)
            mh.remove(x)
        print(f"test_remove: {len(mh)}")

        self.assertTrue(mh._is_heap())

        last_val = float('-inf')
        for i in range(len(mh)):
            item = mh.peek()
            self.assertEqual(mh.heap[mh.heap_idx[item]], item)
            x = mh.pop()
            self.assertLessEqual(last_val, x)
            last_val = x
            # print(f"{i}: {x}")

    def test_update(self):
        mh = MinHeap()
        while len(mh) < 100:
            mh.push(randint(-100000, 100000))

        for i in range(10):
            x = choice(mh.heap)
            mh.update(x, x - 9999999)

        print(f"test_update: {len(mh)}")
        self.assertTrue(mh._is_heap())

        self.assertTrue(mh._is_heap())

        last_val = float('-inf')
        for i in range(len(mh)):
            item = mh.peek()
            self.assertEqual(mh.heap[mh.heap_idx[item]], item)
            x = mh.pop()
            self.assertLessEqual(last_val, x)
            last_val = x
