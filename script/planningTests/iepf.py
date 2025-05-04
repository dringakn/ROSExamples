#!/usr/bin/env python3
"""
Author:    Dr. Ing. Ahmad Kamal Nasir
Email:     dringakn@gmail.com

Description:
    This script implements the Improved Euler Path Filter (IEPF) algorithm
    to smooth a 2D polyline.  Starting from an initial zig‑zag “path” of
    integer grid points, it iteratively adjusts each interior vertex to
    balance fidelity to the original data (controlled by α) against overall
    smoothness (controlled by β), until the total adjustment falls below
    a user‑specified tolerance.

Features:
  • iepf(path, alpha, beta, tol):
      – path: list of [x, y] coordinates (must have at least 3 points)
      – alpha (0 ≤ α ≤ 1): data‑fidelity weight (higher → stays closer to input)
      – beta  (0 ≤ β ≤ 1): smoothness weight (higher → smoother result)
      – tol: convergence tolerance on total coordinate change
      – returns a new list of [x, y] coordinates (same length as input)
  • Example usage and matplotlib plot at the bottom.

Usage:
    ./smooth_path.py
    (adjust α, β, tol in the call to iepf() as desired)

Dependencies:
    • Python 3.x
    • matplotlib
"""
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
