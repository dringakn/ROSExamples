from unittest import TestCase
from pqueue import QNode, PQueue
from random import randint

class TestQNode(TestCase):
    def test_dist(self):
        n1 = QNode((1, 1), 100, 100)
        n2 = QNode((2, 2), 200, 200)
        n3 = QNode((3, 3), 300, 300)
        n4 = QNode((4, 4), 400, 400)
        n5 = QNode((1, 1), 999, 999)
        self.assertIsInstance(n1, QNode)
        self.assertEqual(n1.key, n5.key)
        self.assertNotEqual(n1, n2)
        self.assertTupleEqual(n3.pos, (3, 3))
        self.assertAlmostEqual(n5.g, 999)
        self.assertAlmostEqual(n2.p, 200)
        self.assertLess(n2, n1)  # Note for the inverted logic to compare priority
        self.assertAlmostEqual(n1.dist(n2), 1.4142135623730951)
        self.assertAlmostEqual(n2.dist(n1), 1.4142135623730951)
        self.assertAlmostEqual(n4.dist(n1), 4.242640687119285)
        s = {n1, n2, n3, n4, n5}  # Create a set to test hashing: n1=n5
        self.assertEqual(len(s), 4)
        self.assertIn(n1, s)


class TestPQueue(TestCase):

    def setUp(self) -> None:
        self.q = PQueue()
        self.n1 = QNode((1, 1), 100, 100)
        self.n2 = QNode((2, 2), 200, 200)
        self.n3 = QNode((3, 3), 300, 300)
        self.n4 = QNode((4, 4), 400, 400)
        self.n5 = QNode((1, 1), 999, 999)
        self.q.push(self.n4)
        self.q.push(self.n3)
        self.q.push(self.n2)
        self.q.push(self.n1)
        self.assertIsInstance(self.q, PQueue)
        self.assertIsInstance(self.n1, QNode)
        self.assertEqual(len(self.q), 4)
        for k,v in self.q.heap_ids.items():
            self.assertEqual(self.q.heap[v].key, k)

    def test_clear(self):
        q = PQueue()
        q.push(self.n4)
        q.push(self.n3)
        q.push(self.n2)
        q.push(self.n1)
        self.assertEqual(len(q), 4)
        q.clear()
        self.assertEqual(len(q), 0)

    def test_peek(self):
        self.assertEqual(self.n1, self.q.peek())  # n1 should be at the top

    def test_contains(self):
        self.assertTrue(self.q.contains(self.n2))

    def test_push(self):
        q = PQueue()
        for i in range(100000):
            n = QNode((randint(1,100), randint(1,100)), randint(1,100))
            q.push(n)

        last_val = QNode((0,0), float('-inf'))
        for i in range(len(q)):
            x = q.pop()
            self.assertLessEqual(last_val.p, x.p)
            last_val = x


    def test_pop(self):
        q = PQueue()
        q.push(self.n4)
        q.push(self.n3)
        q.push(self.n2)
        q.push(self.n1)
        self.assertEqual(self.n1, q.pop())

    def test_remove(self):
        q = PQueue()
        for i in range(100):
            n = QNode((randint(1,10), randint(1,10)), randint(1,100))
            if q.contains(n):
                if n.p > 50:
                    q.remove(n)
                    # pass
                else:
                    q.update(n)
            else:
                q.push(n)
        print(f"Size: {len(q)}")
        last_val = QNode((0,0), float('-inf'))
        for i in range(len(q)):
            try:
                x = q.pop()
                self.assertLessEqual(last_val.p, x.p)
                last_val = x
                print(x)
            except Exception as ex:
                pass

    def test_update(self):
        self.q.update(QNode((1,1), 101, 101))
        self.assertEqual(self.n1, self.q.peek())
        self.q.update(QNode((1,1), 201, 201))
        self.assertEqual(self.n2, self.q.peek())
        self.q.update(QNode((1,1), 101, 101))
        self.assertEqual(self.n1, self.q.peek())
        self.q.update(QNode((1,1), 901, 901))
        self.q.update(QNode((2,2), 902, 902))
        self.q.update(QNode((3,3), 903, 903))
        self.q.update(QNode((4,4), 904, 904))
        self.assertEqual(self.n1, self.q.peek())

        for i in range(len(self.q)):
            print(self.q.pop())
