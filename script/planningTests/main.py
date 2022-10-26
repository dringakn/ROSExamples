def list_dict_lookup_performance():
    import time
    ll = [i for i in range(100_000_000)]
    dd = {i: i for i in range(100_000_000)}

    start = time.time()
    print(99_999_999 in dd)
    print(f"DD Exec. time: {time.time() - start}")

    start = time.time()
    print(99_999_999 in ll)
    print(f"LL Exec. time: {time.time() - start}")


def test_pqueue():
    from pqueue import PQueue, QNode
    q = PQueue()
    n1 = QNode((1, 1), 100)
    n2 = QNode((2, 2), 200)
    n3 = QNode((3, 3), 300)
    n4 = QNode((4, 4), 400)
    q.push(n4)
    q.push(n3)
    q.push(n2)
    q.push(n1)
    print(q.contains(n1))
    print(q.contains(n2))
    print(q.contains(n3))
    print(q.contains(n4))
    n = q.pop()
    n = q.pop()
    n = q.pop()
    n = q.pop()
    print(q.contains(n1))
    print(q.contains(n2))
    print(q.contains(n3))
    print(q.contains(n4))
    q.push(n1)
    q.push(n2)
    q.push(n3)
    q.push(n4)
    q.remove(n2)
    q.remove(n2)
    q.remove(n4)
    q.remove(n4)
    q.remove(n3)
    q.remove(n1)
    q.contains(n4)


if __name__ == '__main__':
    # list_dict_lookup_performance()
    test_pqueue()
    # m = OGM(1000, 500)
    # m.set_obstacle((2, 2))
    # # m.get_neighbours((1,2))
    # astar = AStar()
    # astar.set_map(m)
    # astar.set_start(QNode(0, 0))
    # astar.set_goal(QNode(9, 4))
    # astar.search_path()
