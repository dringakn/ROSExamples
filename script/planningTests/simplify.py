import matplotlib.pyplot as plt
import copy

path = [
    [0, 0],

    [9, 0],
    [10, 0],
    [10, 1],

    [10, 2],
    [10, 3],
    [9, 3],

    [1, 3],
    [0, 3],
    [0, 4],

    [0, 5],
    [0, 6],
    [1, 6],

    [9, 6],
    [10, 6],
    [10, 7],

    [10, 8],
    [10, 9],
    [9, 9],

    [1, 9],
    [0, 9],
    [0, 10],

    [0, 11],
    [0, 12],
]

def iepf(path, tol=0.0000001):
    new_path = copy.deepcopy(path)

    change = tol
    while change >= tol:
        change = 0
        for i in range(1, len(path)-1):
            for j in range(len(path[i])):
                old = new_path[i][j]
                new_path[i][j] += (alpha * (path[i][j] - new_path[i][j]))
                new_path[i][j] += (beta * (new_path[i-1][j] + new_path[i+1][j] - (2*new_path[i][j])))
                change += abs(old - new_path[i][j])

    return new_path

new_path = iepf(path, 0.00001)
x = [p[0] for p in new_path]
y = [p[1] for p in new_path]
# for p in path:
#     print(p)

plt.plot(x, y, '-o')
plt.grid(axis='both')
plt.show()
