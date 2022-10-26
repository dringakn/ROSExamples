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
    def __init__(self, pos, p=float('inf'), g=float('inf')):
        self.p = p
        self.g = g
        self.pos = pos
        self.key = hash(self.pos)

    def __lt__(self, other):
        return self.p > other.p  # NOTE: Invert the logic for descending order

    def __gt__(self, other):
        return self.p < other.p  # NOTE: Invert the logic for descending order

    def __repr__(self):
        return f"{self.p:0.2f}(pos:{self.pos}[{self.key}])"

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
        self.heap_ids = dict()  # For quick access, index of nodes in the heap based on node hash value.

    def __repr__(self):
        return f"{self.heap}"

    def __len__(self):
        return len(self.heap_ids)

    def clear(self):
        self.heap.clear()
        self.heap_ids.clear()

    def peek(self):
        if len(self.heap) == 0:
            return None
        else:
            return self.heap[0]

    def contains(self, node: QNode):
        return node.key in self.heap_ids  # Fast access due to dict()

    def push(self, node: QNode):
        result = False
        if node.key not in self.heap_ids:
            self.heap.append(node)  # Add the node to the end of heap
            pos = len(self.heap) - 1  # idx of last position
            self.heap_ids[node.key] = pos  # Add the node key:id to the dictionary for faster access.
            self._shift_up(pos)  # Move it to the appropriate position and update heap_idx
            result = True

        if len(self.heap) != len(self.heap_ids):
            raise AssertionError("Something wrong!!!")
        return result

    def pop(self):
        result = None
        if len(self.heap) == 1:
            result = self.heap.pop()  # Remove the last element
            self.heap_ids.pop(result.key)  # Remove also from cached dictionary
            if len(self.heap) != len(self.heap_ids):
                raise AssertionError("Something wrong!!!")
        elif len(self.heap) >= 2:
            result = self.heap[0]  # Retrieve the first element
            last_elem = self.heap.pop()  # Remove the last element
            self.heap_ids.pop(last_elem.key)  # Remove it also from cache
            self.heap[0] = last_elem  # Replace the first element with last
            self.heap_ids[last_elem.key] = 0  # Change also it's heap_idx
            self._shift_down(0)  # move it to appropriate position and update heap_idx
            if len(self.heap) != len(self.heap_ids):
                raise AssertionError("Something wrong!!!")

        return result

    def remove(self, node: QNode):
        result = False
        if node.key in self.heap_ids:
            node.p = float('inf')
            self.update(node)
            self.heap.pop()
            self.heap_ids.pop(node.key)
            # index = self.heap_ids[node.key] # Quickly find the index of the node
            # last_elem = self.heap.pop() # Remove the element at index with last element on heap.
            # self.heap_ids.pop(last_elem.key) # Remove it also from the cache
            # self.heap[index] = last_elem # Copy last element at the removed index
            # self.heap_ids[last_elem.key] = index  # update the heap_idx accordingly
            # self._shift_down(index)  # Update it's location accordingly
            # # Will the heap_ids are still valid???
            result = True

        if len(self.heap) != len(self.heap_ids):
            raise AssertionError("Something wrong!!!")

        return result

    def update(self, node: QNode):
        result = False
        try:
            if node.key in self.heap_ids:
                index = self.heap_ids[node.key]  # Quickly find the index of the item
                if node.p < self.heap[index].p:  # if new priority is less than existing priority
                    self.heap[index] = node  # Update the node
                    self._shift_up(index)  # Move it to appropriate place
                    result = True
                elif node.p > self.heap[index].p:  # if the new priority is greater than existing priority?
                    self.heap[index] = node  # Update the node
                    self._shift_down(index)  # Move it to appropriate place
                    result = True
        except:
            pass

        if len(self.heap) != len(self.heap_ids):
            raise AssertionError("Something wrong!!!")

        return result

    def _shift_up(self, pos):
        n = len(self.heap)

        if (n < 2) or (pos <= 0):
            return  # return because, no or one element in the heap or idx<=0

        parent = (pos - 1) >> 1

        if not (self.heap[pos] > self.heap[parent]):
            return  # node is less than it's parent (Note inverted logic in overload!)

        val = self.heap[pos]  # Get the new item

        while val > self.heap[parent]:
            self.heap[pos] = self.heap[parent]
            self.heap_ids[self.heap[pos].key] = pos
            pos = parent
            if pos <= 0:
                break
            parent = (parent - 1) >> 1

        self.heap[pos] = val  # Save the new-item
        self.heap_ids[val.key] = pos  # Cache new-item's heap_idx

    def _shift_down(self, pos):
        n = len(self.heap)
        child = (pos << 1) + 1
        if (n < 2) or (child >= n):
            return  # return because, no or one element in the heap or child index is outside heap

        if (child + 1) < n:
            if self.heap[child + 1] > self.heap[child]:
                child = child + 1  # Switch child

        if not (self.heap[child] > self.heap[pos]):
            return  # node is less than it's parent (Note inverted logic in overload!)

        val = self.heap[pos]  # Get the new item

        while self.heap[child] > val:
            self.heap[pos] = self.heap[child]
            self.heap_ids[self.heap[pos].key] = pos
            pos = child
            child = (child << 1) + 1
            if child >= n:
                break

            if (child + 1) < n:
                if self.heap[child + 1] > self.heap[child]:
                    child = child + 1  # Switch child

        self.heap[pos] = val  # Save the new-item
        self.heap_ids[val.key] = pos  # Cache new-item's heap_idx

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
