"""
File: pqueue.py
Created: 22 Oct 2022
Modified: 22 Oct 2022
Author: Dr. -Ing. Ahmad Kamal Nasir [dringakn@gmail.com]
Description:
    Implement Priority Queue using list.
Note:
    For descending order sorting invert the logic of __lt__ operator of class 'Node'.

"""
from math import sqrt


class QNode:
    def __init__(self, pos, p):
        self.g = float('inf')  # Not used for DStar, instead lookup table is used
        self.p = p # Tuple for DStar, override comparision for tuple
        self.pos = pos
        self.key = hash(self.pos)
        self.parent = None # Not used for DStar

    def __lt__(self, other):
        # return self.p < other.p # AStar
        return (self.p[0] < other.p[0]) or ((self.p[0] == other.p[0]) and (self.p[1] < other.p[1])) # DStar

    def __le__(self, other):
        # return self.p <= other.p # AStar
        return (self.p[0] < other.p[0]) or ((self.p[0] == other.p[0]) and (self.p[1] <= other.p[1])) # DStar

    def __gt__(self, other):
        # return self.p > other.p  # AStar
        return (self.p[0] > other.p[0]) or ((self.p[0] == other.p[0]) and (self.p[1] > other.p[1])) # DStar

    def __ge__(self, other):
        # return self.p >= other.p  # AStar
        return (self.p[0] > other.p[0]) or ((self.p[0] == other.p[0]) and (self.p[1] >= other.p[1])) # DStar

    def __repr__(self):
        # return f"{self.p:0.2f}(pos:{self.pos}[{self.key}])"  # For AStar
        return f"({self.p[0]:0.2f}, {self.p[1]:0.2f})pos:{self.pos}"  # For DStar

    def dist(self, other):
        return sqrt((self.pos[0] - other.pos[0]) ** 2 + (self.pos[1] - other.pos[1]) ** 2)

    def __eq__(self, other):
        # return (self.x, self.y) == (other.x, other.y)
        return self.key == other.key

    def __hash__(self):
        return self.key  # Hash value for the position tuple


class PQueue:
    def __init__(self):
        self.heap = list()  # List of nodes in the list
        self.heap_idx = dict()  # For quick access, index of nodes in the heap based on node hash value.
        self.size = len(self.heap)

    def __repr__(self):
        return f"{self.heap}"

    def __len__(self):
        return self.size

    def __contains__(self, n: QNode):
        return n.key in self.heap_idx

    def clear(self):
        self.heap.clear()
        self.heap_idx.clear()
        self.size = 0

    def peek(self):
        result = None
        if self.size:
            result = self.heap[0]
        return result

    def push(self, node: QNode):
        self.heap.append(node)  # Add the node to the end of heap
        self.size += 1  # Increment the number of elements
        idx = self.size - 1  # zero offset, last element
        self.heap_idx[node.key] = idx  # Add the node key:id to the dictionary for faster access.
        self._shift_up(idx)  # Move it to the appropriate position and update heap_idx

    def pop(self):
        result = None
        n = self.size
        if n == 1:  # very last element
            result = self.heap.pop()  # Remove the last element
            self.heap_idx.pop(result.key)  # Remove also from cached dictionary
            self.size -= 1  # Decrement size
        elif n >= 2:  # Two or more elements
            self._swap(0, self.size - 1)  # swap first and last element
            result = self.heap.pop()  # Remove the last element
            self.heap_idx.pop(result.key)  # Remove it also from cache
            self.size -= 1 # Adjust the size
            self._shift_down(0)  # move it to appropriate position and update heap_idx
        return result

    def remove(self, node: QNode):
        result = False
        if node in self:
            idx = self.heap_idx[node.key]
            if idx == self.size-1:
                self.heap.pop()
                self.heap_idx.pop(node.key)
                self.size -= 1
            else:
                self._swap(idx, self.size - 1)
                self.heap.pop()
                self.heap_idx.pop(node.key)
                self.size -= 1
                self._shift_up(idx)  # The combination of up/down make heap invariant, instead of O(n)
                self._shift_down(idx)
            result = True
        return result

    def update(self, node: QNode):
        # Assuming node is present
        index = self.heap_idx[node.key]  # Quickly find the index of the item
        if node < self.heap[index]:  # if new priority is less than existing priority
            self.heap[index] = node  # Update the node
            self._shift_up(index)  # Move it to appropriate place
        elif node > self.heap[index]:  # if the new priority is greater than existing priority?
            self.heap[index] = node  # Update the node
            self._shift_down(index)  # Move it to appropriate place

    def push_or_update(self, node: QNode):
        result = 0
        if node not in self:
            self.push(node)
            result = 1
        else:
            index = self.heap_idx[node.key]  # Quickly find the index of the item
            if node.g < self.heap[index].g:
                self.update(node)
                result = 2
        return result


    def _shift_up(self, idx):
        parent = (idx - 1) >> 1
        while (parent >= 0) and (self.heap[parent] > self.heap[idx]):
            self._swap(parent, idx)
            idx = parent
            parent = (idx - 1) >> 1

    def _shift_down(self, idx):
        n = self.size
        while True:
            m_idx = idx  # Smaller child index
            left = (idx << 1) + 1
            right = left + 1
            if left < n: # Left child exists
                if self.heap[m_idx] > self.heap[left]: # parent is greater than left child
                    m_idx = left # set minimum as left

            if right < n: # Right child exists
                if self.heap[m_idx] > self.heap[right]: # parent or left is greater then right
                    m_idx = right # set minimum as right

            if m_idx != idx:
                self._swap(idx, m_idx)
            else:  # No more swap required
                break

            idx = m_idx

    def _swap(self, idx1, idx2):
        self.heap[idx1], self.heap[idx2] = self.heap[idx2], self.heap[idx1]
        self.heap_idx[self.heap[idx1].key], self.heap_idx[self.heap[idx2].key] = idx1, idx2

    def _is_min_heap(self, idx, n):

        if idx > n:  # complete binary tree: out of valid index range
            return True

        left = (idx << 1) + 1  # left node index
        if left < n:  # check if left node exists
            if self.heap[left] < self.heap[idx]:  # check if left node value is smaller then parent
                print(f"L[{left}]P[{idx}]: {self.heap[left]} < {self.heap[idx]}")
                return False

        right = left + 1  # right node index
        if right < n:  # check if right node exists
            if self.heap[right] < self.heap[idx]:  # check if right node value is smaller then parent
                print(f"R[{right}]P[{idx}]: {self.heap[right]} < {self.heap[idx]}")
                return False

        # check for left and right subtrees
        return self._is_min_heap(left, n) and self._is_min_heap(right, n)

    def _is_heap(self):
        return self._is_min_heap(0, self.size)

    def _build_heap(self):
        """Transform list into a heap, in-place, in O(len(x)) time."""
        n = len(self.heap)
        # Transform bottom-up.  The largest index there's any point to looking at
        # is the largest with a child index in-range, so must have 2*i + 1 < n,
        # or i < (n-1)/2.  If n is even = 2*j, this is (2*j-1)/2 = j-1/2 so
        # j-1 is the largest, which is n//2 - 1.  If n is odd = 2*j+1, this is
        # (2*j+1-1)/2 = j so j-1 is the largest, and that's again n//2 - 1.
        for i in reversed(range(n // 2)):
            self._shift_up(i)
