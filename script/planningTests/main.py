from astar import AStar, QNode, OGM
import time

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

if __name__ == '__main__':
    # list_dict_lookup_performance()
    m = OGM(100, 1000)
    m.set_obstacle((5, 5))
    # m.get_neighbours((1,2))
    astar = AStar()
    astar.set_map(m)
    astar.set_start(QNode((0, 0)))
    astar.set_goal(QNode((99, 999)))
    start = time.time()
    if astar.search_path():
        print(f"Exec. time: {time.time() - start}")
        print(f"Path length: {len(astar.get_path())}")
    else:
        print(f"Exec. time: {time.time() - start}")
