"""
Operations:
Push:         O(log n) -> Add value to the end of list and shift it upward.
Pop(RemMax):  O(log n) ->
Peek(GetMax): O(1)
Remove:       O(log n)
Update:       O(log n)

Easy to implement using a list. All nodes below a specific node must be lower than it.
 ArrayIndex     | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 10 | 11 |
 Data           | 25 | 16 | 24 | 05 | 11 | 19 | 01 | 02 | 03 | 05 | 12 |
Properties:
 parent[idx]    = (idx-1)/2, e.g. the parent of value @idx=4[05] is @idx=2[16]
 LeftChild[idx] = (idx*2)+1, e.g. the left child of value @idx=4[05] is @idx=8[02]
 RightChild[idx] = (idx*2)+2, e.g. the right child of value @idx=4[05] is @idx=9[03]

Push Example:
    Let's say we want to push [12] to the MaxHeap. We insert it the end at location 11.
    Now shift it up by swapping to if it's greater than it's parent.
              25                               01
       16         24                     02           03
  05      11    19  01                04     05     06    07
02  03  05  *12                     08  09 10  11

Pop Example: Swap the first[max] element to the last element. Remove the last and now
             shift the first element dow to it's proper location. To pop 25, swap it with 05.
             Remove 25 and now bubble down 11. check value with it's right child. if it's less
             than it's right child 11 < 24 so swap 11 and 24. Then check 11 with right[1] if false check with left[19]..
"""
import heapq


class MinHeap:
    def __init__(self):
        super().__init__()
        self.heap = []
        self.heap_idx = {}
        self.size = len(self.heap)

    def __contains__(self, item):
        return item in self.heap_idx

    def __len__(self):
        return self.size

    def push(self, item):
        result = False
        if item not in self:
            self.heap.append(item)
            self.size += 1
            idx = self.size - 1  # zero offset, last element
            self.heap_idx[item] = idx
            self.__shift_up(idx)
            result = True
        return result

    def pop(self):
        result = None
        n = self.size
        if n == 1:  # very last element
            result = self.heap.pop()
            self.heap_idx.pop(result)
            self.size -= 1
        elif n >= 2: # Two or more elements
            self.__swap(0, self.size - 1) # swap first and last element
            result = self.heap.pop() # remove last element
            self.heap_idx.pop(result) # remove also from heap
            self.size -= 1
            self.__shift_down(0) # re-arrange
        return result

    def peek(self):
        result = None
        if self.size:
            result = self.heap[0]
        return result

    def update(self, item, nitem):
        result = False
        if (item in self.heap_idx) and (nitem not in self.heap_idx):
            idx = self.heap_idx[item]
            # update structure with p-value
            self.heap[idx] = nitem
            self.heap_idx.pop(item)    # Remove old key
            self.heap_idx[nitem] = idx # Add new key
            if nitem < item:
                # No need to change left and right subtrees, only check parent
                self.__shift_up(idx)
            elif nitem > item:
                # No need to check parent, only left or right subtree need to be adjusted
                self.__shift_down(idx) # Adjust from current's parent
            result = True
        return result

    def remove(self, item):
        result = False
        if item in self.heap_idx:
            idx = self.heap_idx[item]
            self.__swap(idx, self.size - 1)
            self.heap.pop()
            self.heap_idx.pop(item)
            self.size -= 1
            self.__shift_up(idx)  # The combination of up/down make heap invariant, instead of O(n)
            self.__shift_down(idx)
            result = True
        return result

    def __shift_up(self, idx):
        parent = (idx - 1) >> 1
        while (parent >= 0) and (self.heap[parent] > self.heap[idx]):
            self.__swap(parent, idx)
            idx = parent
            parent = (idx - 1) >> 1

    def __shift_down(self, idx):
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
                self.__swap(idx, m_idx)
            else:  # No more swap required
                break

            idx = m_idx


    def __swap(self, idx1, idx2):
        self.heap[idx1], self.heap[idx2] = self.heap[idx2], self.heap[idx1]
        self.heap_idx[self.heap[idx1]], self.heap_idx[self.heap[idx2]] = idx1, idx2

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

        # Else check left and right subtree

    def _is_heap(self):
        return self._is_min_heap(0, self.size)
