#include <bits/stdc++.h>  // Standard C++ related functionality
// #include <ros/ros.h>      // ROS related functionality, comment for GDB

const int K = 2;       // Number of dimenssions for the KD-Tree
typedef int datatype;  // To ease in chaning datatype
using namespace std;   // Standard namespace

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
};

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

datatype treeHeight(Node* n) {
  if (n == nullptr) {
    return -1;
  }
  return max(treeHeight(n->left), treeHeight(n->right)) + 1;
}

// TODO: Need modifications
Node* findMin(Node* n) {
  if (n == nullptr) {
    return nullptr;
  } else if (n->left == nullptr) {
    return n;
  } else {
    return findMin(n->left);
  }
}

// TODO: Need modifications
Node* findMax(Node* n) {
  if (n == nullptr) {
    return nullptr;
  } else if (n->right == nullptr) {
    return n;
  } else {
    return findMax(n->right);
  }
}

bool linearSearch(vector<Point>& L, Point& key) {
  bool result = false;
  for (auto&& l : L) {
    if (l.x == key.x && l.y == key.y) {
      result = true;
    }
  }
  return result;
}

int main(int argc, char* argv[]) {
  //   ros::init(argc, argv, "example_kdtree_search");
  //   ros::NodeHandle nh;
  std::mt19937 rng(0);
  std::uniform_int_distribution<int> distUni(-1e1, 1e1);
  auto start = chrono::high_resolution_clock::now();
  auto stop = chrono::high_resolution_clock::now();
  Point a(2, 4), b(-10, 100), c(-1, -1), d(3, 4), e(9, 4), f(6, 7);
  vector<Point> data(1e1);
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

  //   cout << a << b;
  //   cout << a[0] << '\t' << a[1] << endl;
  //   cout << "a == b :" << (a == b) << endl;
  //   cout << "a != b :" << (a != b) << endl;

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
  cout << "Find Min: " << findMin(tree)->data << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  cout << "Find Max: " << findMax(tree)->data << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  return 0;
}