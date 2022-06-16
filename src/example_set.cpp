#include <bits/stdc++.h>
#include <set>
#include <map>

/*
Properties:
    The set stores the elements in sorted order.
    All the elements in a set have unique values.
    The value of the element cannot be modified once it is added to the set, though it is possible to remove and then add the modified value of that element.
    Thus, the values are immutable.
    Sets follow the Binary search tree implementation.
    The values in a set are unindexed
*/

using namespace std;

template <typename Key, typename Priority>
class MyNode
{
public:
    Key key;
    Priority priority;
    MyNode(Key k, Priority p) : key(k), priority(p) {}

    friend bool operator<(const MyNode &lhs, const MyNode &rhs)
    {
        // return lhs.priority < rhs.priority;
        return lhs.priority > rhs.priority; // Reverse
    }

    friend bool operator>(const MyNode &lhs, const MyNode &rhs)
    {
        // return lhs.priority > rhs.priority;
        return lhs.priority < rhs.priority; // Reverse
    }

    friend std::ostream &operator<<(std::ostream &os, const MyNode &n)
    {
        os << n.key << ": " << n.priority;
        return os;
    }
};

template <typename Key, typename Priority>
class MyQueue
{
protected:
    std::map<Key, Priority> id;              // Id of the value
    std::vector<MyNode<Key, Priority>> heap; // Sorted node in the list

public:
    MyQueue() {}                                                      // Constructor
    bool empty() const { return heap.empty(); }                       // Check if the queue is empty?
    std::size_t size() const { return heap.size(); }                  // Get the number of elements in the queue
    const MyNode<Key, Priority> &top() const { return heap.front(); } // First value in the queue

    /* Returns true if the key was not inside and was added, otherwise does nothing and returns false
     * If the key was remembered and only_if_unknown is true, does nothing and returns false
     */
    bool push(const Key &key, const Priority &priority)
    {

        size_t n = heap.size();           // Current elements in the queue
        id[key] = n;                      // Element is stored at the end (Vector index)
        heap.emplace_back(key, priority); // Place the element at the end of the vector
        shiftUp(n);                       // Sort the elements, move the largest to the front.
        return true;
    }

    MyNode<Key, Priority> pop()
    {
        if (size() == 0)
            return MyNode<Key, Priority>(-1, Priority());

        MyNode<Key, Priority> ret = std::move(*heap.begin()); // Get the front value
        id.erase(ret.key);                                    // remove the corresponding id
        if (size() > 1)                                       // If there are more than one node in the list
            *heap.begin() = std::move(*(heap.end() - 1));     // Move the last entry at front

        heap.pop_back();
        shiftDown(0);
        return ret;
    }

    MyNode<Key, Priority> get_priority(const Key &key)
    {

        if (id.find(key) == id.end()) // Key is not available
            return MyNode<Key, Priority>(0, 0);
        else
            return heap[id[key]];
    }

    /** Returns true if the key was already inside and was updated, otherwise does nothing and returns false */
    bool set_priority(const Key &key, const Priority &new_priority)
    {
        bool result = false;
        if (id.find(key) == id.end()) // Key is not available
            return result;

        size_t heappos = id[key];
        Priority &priority = heap[heappos].priority; // Get the priority of the key
        if (new_priority > priority)                 // If new priority is higher
        {
            priority = new_priority; // Updte pirority
            shiftUp(heappos);
            result = true;
        }
        else if (new_priority < priority)
        {
            priority = new_priority;
            shiftDown(heappos);
            result = true;
        }
        return result;
    }

private:
    void shiftUp(size_t heappos) // Move the value up
    {
        size_t len = heap.size();    // Size of the queue
        if (len < 2 || heappos <= 0) // Return if queue position is less than or equal to 0 or only 1 element in the queue
            return;

        size_t parent = (heappos - 1) >> 1;  // Middle of the queue: e.g. (4-1)/2=1
        if (!(heap[heappos] > heap[parent])) // Return if queue value is not greater than it's parent
            return;

        MyNode<Key, Priority> val = std::move(heap[heappos]); // Convert a value to an rvalue.
        do
        {
            heap[heappos] = std::move(heap[parent]); // move the parent to heappos
            id[heap[heappos].key] = heappos;         // change the id of the node
            heappos = parent;                        // Update the parent
            if (heappos <= 0)                        // If the id is less than or equal to 0
                break;                               // If parent id becomes or equal to 0
            parent = (parent - 1) >> 1;              // Middle of the queue
        } while (val > heap[parent]);                // Until the value is greater than it's parent

        heap[heappos] = std::move(val);  // Move the last value
        id[heap[heappos].key] = heappos; // ?
    }

    void shiftDown(size_t heappos)
    {
        size_t len = heap.size(); // Size of the queue
        size_t child = (heappos << 1) + 1;
        if (len < 2 || child >= len)
            return;

        if (child + 1 < len && heap[child + 1] > heap[child])
            ++child; // Check whether second child is higher

        if (!(heap[child] > heap[heappos]))
            return; // Already in heap order

        MyNode<Key, Priority> val = std::move(heap[heappos]);
        do
        {
            heap[heappos] = std::move(heap[child]);
            id[heap[heappos].key] = heappos;
            heappos = child;
            child = (child << 1) + 1;
            if (child >= len)
                break;

            if (child + 1 < len && heap[child + 1] > heap[child])
                ++child;

        } while (heap[child] > val);

        heap[heappos] = std::move(val);
        id[heap[heappos].key] = heappos;
    }
};

int main(int argc, char const *argv[])
{
    set<int> sa;                 // defining an empty set
    set<int> sb = {6, 10, 5, 1}; // defining a set with values
    set<int, less<int>> sc;      // empty set container, Ascending order
    sc.insert(40);               // insert elements in random order
    sc.insert(30);
    sc.insert(60);
    sc.insert(20);
    sc.insert(50);
    sc.insert(50); // Duplicated value, will not be added
    sc.insert(50); // Duplicated value, will not be added
    sc.insert(50); // Duplicated value, will not be added
    sc.insert(50); // Duplicated value, will not be added

    // Iterating over set
    for (auto itr = sc.begin(); itr != sc.end(); itr++)
        cout << *itr << " ";
    cout << endl;

    // assigning the elements from sc to sd

    set<int> sd(sc.begin(), sc.end());

    // remove all elements up to 40 in sd
    sd.erase(sd.begin(), sd.find(40));

    int n;
    // remove element with value 50 in sd
    n = sd.erase(50);
    cout << "Number of removed elements: " << n << endl;

    // check if an element exist in the set
    bool isfound = sd.find(50) != sd.end();
    cout << "Does 50 found in the set?: " << isfound << endl;
    // cout << "Does 50 contain in the set?: " << sd.contains(50) << endl; // c++20

    // remove element which doesn't exist in sd
    n = sd.erase(100);
    cout << "Number of removed elements: " << n << endl;

    // lower bound and upper bound for set sd
    cout << "sd.lower_bound(49) : " << *sd.lower_bound(49) << endl; // Pointer to value <= 49 or end()
    cout << "sd.upper_bound(61) : " << *sd.upper_bound(61) << endl; // Pointer to value <= 61 or end()

    for (auto itr = sd.begin(); itr != sd.end(); itr++)
        cout << *itr << " ";
    cout << endl;

    // Testing custom nodes as set
    cout << "Custom Node Example" << endl;
    std::set<MyNode<int, int>> s;
    s.insert(MyNode<int, int>(1, 10));
    s.insert(MyNode<int, int>(2, 10));
    s.insert(MyNode<int, int>(3, 10));
    s.insert(MyNode<int, int>(3, 10));
    s.insert(MyNode<int, int>(3, 10));
    s.insert(MyNode<int, int>(3, 9));
    s.insert(MyNode<int, int>(3, 8));
    s.insert(MyNode<int, int>(3, 11));
    s.insert(MyNode<int, int>(3, 12));
    // Iterating over set
    for (auto itr = s.begin(); itr != s.end(); itr++)
        cout << *itr << endl;
    cout << endl;
    isfound = s.find(MyNode(1, 9)) != s.end();
    cout << "Does (1, 9) found in the set?: " << isfound << endl;

    // Testing custom pirority queue
    cout << "MyQueue" << endl;
    MyQueue<unsigned long, double> pq;
    std::random_device device();                                   // Random number device
    std::mt19937 rng(0);                                           // RNG, 0, time(0), device
    std::uniform_real_distribution<double> distUni(-10000, 10000); // Distributation
    auto t0 = std::chrono::steady_clock::now();
    for (signed long i = 1; i <= 100; i++)
        pq.push(i, distUni(rng)); // Add a node with key and priority
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count() << std::endl;

    t0 = std::chrono::steady_clock::now();
    while (!pq.empty()) // Print the queue
        std::cout << pq.pop() << endl;
    // pq.pop();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count() << std::endl;

    return 0;
}
