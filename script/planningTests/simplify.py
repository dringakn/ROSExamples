import matplotlib.pyplot as plt
import copy
import math

path = [
    [0, 0, 0],

    [9, 0, 0],
    [10, 0, 0],
    [10, 1, 0],

    [10, 2, 0],
    [10, 3, 0],
    [9, 3, 0],

    [1, 3, 0],
    [0, 3, 0],
    [0, 4, 0],

    [0, 5, 0],
    [0, 6, 0],
    [1, 6, 0],

    [9, 6, 0],
    [10, 6, 0],
    [10, 7, 0],

    [10, 8, 0],
    [10, 9, 0],
    [9, 9, 0],

    [1, 9, 0],
    [0, 9, 0],
    [0, 10, 0],

    [0, 11, 0],
    [0, 12, 0],
]

def norm(u):
    return math.sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2])


def cross(u, v):
    return [u[1]*v[2] - u[2]*v[1], -(u[0]*v[2] - u[2]*v[0]), u[0]*v[1] - u[1]*v[0]]


def dist_to_line(pt, start, end):
    # works when start and end are not collinear
    u = [0, 0, 0] # direction vector
    u[0] = end[0] - start[0]
    u[1] = end[1] - start[1]
    u[2] = end[2] - start[2]
    v = [0, 0, 0] # Point vector
    v[0] = pt[0] - start[0]
    v[1] = pt[1] - start[1]
    v[2] = pt[2] - start[2]
    return norm(cross(v, u)) / norm(u)


def simplify(path, tol=1):
    new_path = []
    if len(path)>2:
        new_path.append(path[0])
        for idx in range(2, len(path)):
            dist = dist_to_line(path[idx-1], path[idx-2], path[idx])
            if dist > tol:
                new_path.append(path[idx-1])

    return new_path

new_path = simplify(path, 0.5)
print(new_path)
# x = [p[0] for p in new_path]
# y = [p[1] for p in new_path]
# for p in path:
#     print(p)

# plt.plot(x, y, '-o')
# plt.grid(axis='both')
# plt.show()
