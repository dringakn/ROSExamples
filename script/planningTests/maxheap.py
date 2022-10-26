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
 parent[idx]    = idx/2, e.g. the parent of value @idx=4[05] is @idx=2[16]
 LeftChild[idx] = idx*2, e.g. the left child of value @idx=4[05] is @idx=8[02]
 RightChild[idx] = (idx*2)+1, e.g. the right child of value @idx=4[05] is @idx=9[03]

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


class MaxHeap:
    def __init__(self):
        super().__init__()
        self.heap = []
        self.heap_idx = {}

    def __contains__(self, item):
        return item in self.heap_idx

    def __len__(self):
        return len(self.heap)

    def push(self, item):
        if item not in self:
            n = len(self.heap)
            self.heap.append(item)
            self.heap_idx[item] = n
            self.__shift_up(n)

    def pop(self):
        result = None
        n = len(self.heap)
        if n == 1:
            result = self.heap.pop()
            self.heap_idx.pop(result)
        elif n >= 2:
            self.__swap(0, len(self.heap) - 1)
            result = self.heap.pop()
            self.heap_idx.pop(result)
            self.__shift_down(0)
        return result

    def peek(self):
        result = None
        if len(self.heap):
            result = self.heap[0]
        return result

    def update(self, item, nitem):
        if (item in self.heap_idx) and (nitem not in self.heap_idx):
            idx = self.heap_idx[item]
            # update structure with p-value
            self.heap[idx] = nitem
            self.heap_idx.pop(item)
            self.heap_idx[nitem] = idx
            if nitem > item:
                self.__shift_up(idx)
            elif nitem < item:
                self.__shift_down(idx)

    def remove(self, item):
        if item in self.heap_idx:
            idx = self.heap_idx[item]
            self.__swap(idx, len(self.heap) - 1)
            result = self.heap.pop()
            self.heap_idx.pop(result)
            self.__shift_down(idx)

    def __shift_up(self, idx):
        while idx > 0:
            parent = idx >> 1
            if self.heap[idx] > self.heap[parent]:
                self.__swap(idx, parent)
            idx = parent

    def __shift_down(self, idx):
        n = len(self.heap)
        while True:
            m_idx = idx
            left = idx << 1
            right = left + 1
            if n > left and self.heap[m_idx] < self.heap[left]:
                m_idx = left
            if n > right and self.heap[m_idx] < self.heap[right]:
                m_idx = right
            if m_idx != idx:
                self.__swap(idx, m_idx)
            else:  # No more swap required
                break
            idx = m_idx

    def __swap(self, idx1, idx2):
        self.heap[idx1], self.heap[idx2] = self.heap[idx2], self.heap[idx1]
        self.heap_idx[self.heap[idx1]], self.heap_idx[self.heap[idx2]] = idx1, idx2
