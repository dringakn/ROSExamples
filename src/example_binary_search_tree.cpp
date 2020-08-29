/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
// #include <ros/ros.h>

#include <bits/stdc++.h>  // C++ Headers

#include <chrono>  // Time profiling
#include <random>  // Random Number Generator (RNG)

typedef int datatype;
using namespace std;

struct Node {
  datatype data;
  Node *left, *right;
  Node() : left(nullptr), right(nullptr), data(0) {}
};

Node* createNode(datatype& data) {
  Node* n = new Node;            // Create node
  n->data = data;                // Update node data
  n->left = n->right = nullptr;  // Update node childrens
  return n;                      // Return node
}

Node* insert(Node* n, datatype& data) {
  if (n == nullptr) {
    n = createNode(data);
  } else if (data < n->data) {
    n->left = insert(n->left, data);
  } else {
    n->right = insert(n->right, data);
  }
}

bool search(Node* n, datatype& key) {
  if (n == nullptr) {
    return false;
  } else if (n->data == key) {
    return true;
  } else if (key < n->data) {
    return search(n->left, key);
  } else {
    return search(n->right, key);
  }
}

Node* searchNN(Node* n, datatype& key, double& dist, Node* res = nullptr) {
  if (n == nullptr) {
    return res;
  } else if (n->data == key) {
    res = n;
    return res;
  }
  double norm = abs(n->data - key);
  if (norm < dist) {
    dist = norm;
    res = n;
  }
  if (key < n->data) {
    return searchNN(n->left, key, dist, res);
  } else {
    return searchNN(n->right, key, dist, res);
  }
}

Node* findMin(Node* n) {
  if (n == nullptr) {
    return nullptr;
  } else if (n->left == nullptr) {
    return n;
  } else {
    return findMin(n->left);
  }
}

Node* findMax(Node* n) {
  if (n == nullptr) {
    return nullptr;
  } else if (n->right == nullptr) {
    return n;
  } else {
    return findMax(n->right);
  }
}

datatype treeHeight(Node* n) {
  if (n == nullptr) {
    return -1;
  }
  return max(treeHeight(n->left), treeHeight(n->right)) + 1;
}

void preOrder(Node* n) {
  if (n == nullptr) {
    return;
  }
  cout << n->data << " ";  // Parent
  preOrder(n->left);       // Left
  preOrder(n->right);      // Right
}

void inOrder(Node* n) {
  if (n == nullptr) {
    return;
  }
  inOrder(n->left);        // Left
  cout << n->data << " ";  // Parent
  inOrder(n->right);       // Right
}

void postOrder(Node* n) {
  if (n == nullptr) {
    return;
  }
  postOrder(n->left);      // Left
  postOrder(n->right);     // Right
  cout << n->data << " ";  // Parent
}

datatype linearSearchNN(vector<datatype>& L, datatype& key) {
  datatype tempMax = INT_MAX, dist, result = tempMax;
  for (int i = 0; i < L.size(); i++) {
    dist = abs(L[i] - key);
    if (dist < tempMax) {
      tempMax = dist;
      result = L[i];
    }
  }
  return result;
}

int main(int argc, char* argv[]) {
  //   ros::init(argc, argv, "example_binary_search_tree");
  //   ros::NodeHandle nh;

  std::random_device device();  // Random number device
  std::mt19937 rng(0);          // RNG, 0, time(0), device
  std::uniform_int_distribution<int> distUni(-1e6, 1e6);  // Distributation
  vector<datatype> data(1e6);                             // Data
  Node* tree = nullptr;                                   // Binary Tree
  auto start = chrono::high_resolution_clock::now();  // Time profiling variable
  auto stop = chrono::high_resolution_clock::now();   // Time profiling variable

  // Populate tree
  cout << "Create Tree: ";
  start = chrono::high_resolution_clock::now();
  for (int i = 0; i < data.size(); i++) {
    data[i] = distUni(rng);
    tree = insert(tree, data[i]);
  }
  stop = chrono::high_resolution_clock::now();
  cout << endl;
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  cout << "Sort List: ";
  start = chrono::high_resolution_clock::now();
  sort(data.begin(), data.end());
  stop = chrono::high_resolution_clock::now();
  //   for (auto d : data) std::cout << d << ' ';
  //   cout << endl;
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Pre-order traversal (DFS)
  cout << "Pre-order: ";
  start = chrono::high_resolution_clock::now();
  //   preOrder(tree);
  stop = chrono::high_resolution_clock::now();
  cout << endl;
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // In-order traversal (DFS)
  cout << "In-order: ";
  start = chrono::high_resolution_clock::now();
  //   inOrder(tree);
  stop = chrono::high_resolution_clock::now();
  cout << endl;
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Post-order traversal (DFS)
  cout << "Post-order: ";
  start = chrono::high_resolution_clock::now();
  //   postOrder(tree);
  stop = chrono::high_resolution_clock::now();
  cout << endl;
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Tree height
  start = chrono::high_resolution_clock::now();
  cout << "Tree Height: " << treeHeight(tree) << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Search for the key
  datatype key = (argc > 1) ? atoi(argv[1]) : 50;
  start = chrono::high_resolution_clock::now();
  cout << "Search: " << key << ", result=" << search(tree, key) << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Search for the min key
  start = chrono::high_resolution_clock::now();
  cout << "Min: " << findMin(tree)->data << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Search for the max key
  start = chrono::high_resolution_clock::now();
  cout << "Max: " << findMax(tree)->data << endl;
  stop = chrono::high_resolution_clock::now();
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Search for Nearest key
  start = chrono::high_resolution_clock::now();
  double dist = INT_MAX;  // minimum distance
  Node* res = searchNN(tree, key, dist);
  stop = chrono::high_resolution_clock::now();
  cout << "SearchNN: " << key << "\t" << res->data << endl;
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Linear search for Nearest key
  start = chrono::high_resolution_clock::now();
  datatype res1 = linearSearchNN(data, key);
  stop = chrono::high_resolution_clock::now();
  cout << "LinearSearchNN: " << key << "\t" << res1 << endl;
  cout << "Elapsed time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  return 0;
}