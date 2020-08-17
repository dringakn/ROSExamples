#include <bits/stdc++.h>  // Standard C++ related functionality
// #include <ros/ros.h>      // ROS related functionality, comment for GDB

const int K = 2;       // Number of dimenssions for the KD-Tree
typedef int datatype;  // To ease in chaning datatype
using namespace std;   // Standard namespace

/**
 * @brief Data to maintained by the KD-Tree
 *
 */
struct Point {
  datatype x, y;
  Point() : x(0), y(0) {}  // Default constructor
  Point(datatype a, datatype b) : x(a), y(b) {}
  // Subscript operator overloading
  datatype& operator[](int index) { return (index) ? y : x; }
  // << operator overloading
  friend ostream& operator<<(ostream& os, const Point& obj) {
    return os << obj.x << '\t' << obj.y << endl;
  }
  // Comparision operator
  bool operator==(const Point& rhs) const {
    return (x == rhs.x && y == rhs.y) ? true : false;
  }
  // Not equal operator
  bool operator!=(const Point& rhs) const { return !(*this == rhs); }
  // Addition operator
  Point operator+(const Point& rhs) const {
    return Point(this->x + rhs.x, this->y + rhs.y);
  }
  // Subtraction operator
  Point operator-(const Point& rhs) const {
    return Point(this->x - rhs.x, this->y - rhs.y);
  }
  // Second norm (Euclidean distance) of the point
  double norm2() { return sqrt(pow(x, 2) + pow(y, 2)); }
};

/**
 * @brief Node structure for KD-Tree
 *
 */
struct Node {
  Point data;
  Node *left, *right;
  Node() : left(nullptr), right(nullptr) {}
  Node(Point& pt) : left(nullptr), right(nullptr), data(pt) {}
  ~Node() {  // Default destructor
    if (left) {
      delete left;
      left = nullptr;
    }
    if (right) {
      delete right;
      right = nullptr;
    }
  }
};

/**
 * @brief Create a new node in the KD-Tree at appopriate location
 *
 * @param n Root Node
 * @param pt Point to be inserted
 * @param depth Depth of the Root node, ignore it!
 * @return Node* Pointer to the root node
 */
Node* insert(Node* n, Point& pt, int depth = 0) {
  if (n == nullptr) {
    return new Node(pt);
  }
  int cd = depth % K;  // Get the current dimension
  if (pt[cd] < n->data[cd]) {
    n->left = insert(n->left, pt, depth + 1);
  } else {
    n->right = insert(n->right, pt, depth + 1);
  }
}

/**
 * @brief Search the specified point in the KD-Tree
 *
 * @param n Root Node
 * @param key Point to be searched
 * @param depth Depth of the Root node, ignore it!
 * @return true If the point is found
 * @return false If the key point is not found
 */
bool search(Node* n, Point& key, int depth = 0) {
  if (n == nullptr) {
    return false;
  } else if (n->data == key) {
    return true;
  }
  int cd = depth % K;
  if (key[cd] < n->data[cd]) {
    return search(n->left, key, depth + 1);
  } else {
    return search(n->right, key, depth + 1);
  }
}

/**
 * @brief Find the nearest neighbouring point in the KDTree.
 *
 * @param n The root node.
 * @param key The point to be searched.
 * @param dist The distance threshold for searching.
 * @param res Ignore it! For internal book-keeping, same as return value.
 * @param depth Ignore it!.
 * @return Node* The nearest node to the specified key.
 */
Node* searchNN(Node* n, Point& key, double& dist, Node* res = nullptr,
               int depth = 0) {
  if (n == nullptr) {
    return res;
  } else if (n->data == key) {
    res = n;
    return res;
  } else {
    double norm = (n->data - key).norm2();
    if (norm < dist) {
      dist = norm;
      res = n;
    }
    int cd = depth % K;
    if (key[cd] < n->data[cd]) {
      return searchNN(n->left, key, dist, res, depth + 1);
    } else {
      return searchNN(n->right, key, dist, res, depth + 1);
    }
  }
}

/**
 * @brief Height of the KD-Tree
 *
 * @param n Root node
 * @return datatype -1 if the tree is emplty otherwise the depth
 */
datatype treeHeight(Node* n) {
  if (n == nullptr) {
    return -1;
  }
  return max(treeHeight(n->left), treeHeight(n->right)) + 1;
}

/**
 * @brief Find the minimum along the specified dimension
 *
 * @param n Root Node
 * @param d Dimension along which minimum is to be find.
 * @param depth Ignore it!
 * @return datatype MAX if the tree is empth otherwise minimum.
 */
datatype findMin(Node* n, int d, int depth = 0) {
  if (n == nullptr) {
    return INT_MAX;
  }
  int cd = depth % K;
  // Compare point with root with respect to cd (Current dimension)
  if (cd == d) {
    if (n->left == nullptr) {
      return n->data[cd];
    } else {
      return min(n->data[cd], findMin(n->left, d, depth + 1));
    }
  }
  // If cd is different, then minimum can be anywhere in this subtree
  return min({n->data[d], findMin(n->left, d, depth + 1),
              findMin(n->right, d, depth + 1)});
}

/**
 * @brief Find minimums for all dimensions
 *
 * @param n The root node
 * @return Point The minimum along each KDTree dimension.
 */
Point findMins(Node* n) {
  Point result;
  result.x = findMin(n, 0);
  result.y = findMin(n, 1);
  return result;
}

/**
 * @brief Find the maximum along the specified dimension
 *
 * @param n The root node.
 * @param d Dimension along which the maximum to be searched.
 * @param depth Ignore it!
 * @return datatype Returns MIN if tree is empty otherwise the maximum.
 */
datatype findMax(Node* n, int d, int depth = 0) {
  if (n == nullptr) {
    return INT_MIN;
  }
  int cd = depth % K;
  // Compare point with root with respect to cd (Current dimension)
  if (cd == d) {
    if (n->right == nullptr) {
      return n->data[cd];
    } else {
      return max(n->data[cd], findMax(n->right, d, depth + 1));
    }
  }
  // If cd is different then maximum can be anywhere in this subtree
  return max({n->data[d], findMax(n->left, d, depth + 1),
              findMax(n->right, d, depth + 1)});
}

/**
 * @brief Find the maximums along all the dimensions of the KDTree.
 *
 * @param n The root node.
 * @return Point MIN if the tree is empty, otherwise maximums along each
 * dimension.
 */
Point findMaxs(Node* n) {
  Point result;
  result.x = findMax(n, 0);
  result.y = findMax(n, 1);
  return result;
}

/**
 * @brief Linearly search (Brute Force) the specified Key from a given vector or
 * points.
 *
 * @param L vector of points
 * @param key Point to be searched
 * @return true If the point is found within the vector.
 * @return false otherwise.
 */
bool linearSearch(vector<Point>& L, Point& key) {
  bool result = false;
  for (auto&& l : L) {
    if (l.x == key.x && l.y == key.y) {
      result = true;
    }
  }
  return result;
}

/**
 * @brief Find the nearest neighbour of the specified point.
 *
 * @param L Vector of points.
 * @param key Point to be searched.
 * @return Point Closeset point in term of Euclidean distance.
 */
Point linearSearchNN(vector<Point>& L, Point& key) {
  Point res;
  double dist, min = DBL_MAX;
  for (auto&& l : L) {
    dist = (l - key).norm2();
    if (dist < min) {
      res = l;
      min = dist;
    }
  }
  return res;
}

/**
 * @brief Linearly search the minimum and maximum along all dimensions of the
 * vector list.
 *
 * @param L The vector to be searched
 * @param min Reference point containing the minimum along each dimension.
 * @param max Referenced point containing maximum along each dimension.
 */
void linearMinsMaxs(vector<Point>& L, Point& min, Point& max) {
  min.x = min.y = INT_MAX;
  max.x = max.y = INT_MIN;
  for (auto&& l : L) {
    if (l.x < min.x) min.x = l.x;
    if (l.y < min.y) min.y = l.y;
    if (l.x > max.x) max.x = l.x;
    if (l.y > max.y) max.y = l.y;
  }
}

/**
 * @brief The example program.
 *
 * @param argc The number of command line arguments.
 * @param argv The list of arguments.
 * @return int 0 if sucessful.
 */
int main(int argc, char* argv[]) {
  //   ros::init(argc, argv, "example_kdtree_search");
  //   ros::NodeHandle nh;
  std::mt19937 rng(0);
  std::uniform_int_distribution<int> distUni((argc > 2) ? atoi(argv[2]) : -10,
                                             (argc > 3) ? atoi(argv[3]) : 10);
  auto start = chrono::high_resolution_clock::now();
  auto stop = chrono::high_resolution_clock::now();
  Point a(3, 4), b(0, 0), c(-1, -1), d(3, 4), e(9, 4), f(6, 7), g(0, 0);
  vector<Point> data((argc > 1) ? atoi(argv[1]) : 10);
  Node* tree = nullptr;
  start = chrono::high_resolution_clock::now();
  for (auto&& d : data) {
    d.x = distUni(rng);
    d.y = distUni(rng);
    tree = insert(tree, d);
    cout << d;
  }
  stop = chrono::high_resolution_clock::now();
  cout << "Creation Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  cout << "a :" << a << "b :" << b;
  cout << "Indexing a:" << a[0] << '\t' << a[1] << endl;
  cout << "a == b :" << (a == b) << endl;
  cout << "a != b :" << (a != b) << endl;
  cout << "a + b :" << (a + b);
  cout << "a - b :" << (a - b);
  cout << "||a - b|| :" << (a - b).norm2() << endl;

  Point key = data.back();
  start = chrono::high_resolution_clock::now();
  cout << "Search: " << key << search(tree, key) << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  cout << "LinearSearch: " << key << linearSearch(data, key) << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  cout << "Tree Height: " << treeHeight(tree) << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  cout << "Find Mins: " << findMins(tree) << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  cout << "Find Maxs: " << findMaxs(tree) << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  Point min, max;
  linearMinsMaxs(data, min, max);
  cout << "Linear Find Mins Maxs: " << endl << min << max;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  key.x = distUni(rng);
  key.y = distUni(rng);
  Point res = linearSearchNN(data, key);
  cout << "LinearSearchNN: " << key << res << (res - key).norm2() << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  double dist = DBL_MAX;
  res = searchNN(tree, key, dist)->data;
  cout << "SearchNN: " << res << (res - key).norm2() << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  return 0;
}