from astar import AStar, QNode, OGM
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
    m = OGM(1000, 500)
    m.set_obstacle((5, 5))
    # m.get_neighbours((1,2))
    astar = AStar()
    astar.set_map(m)
    astar.set_start(QNode((0, 0)))
    astar.set_goal(QNode((9, 9)))
    astar.search_path()
