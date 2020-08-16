#include <ros/ros.h>

#include <chrono>  // Time profiling
#include <random>  // Random Number Generator (RNG)

typedef int datatype;
using namespace std;

struct Node {
  datatype data;
  Node *left, *right;
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

void searchNN(Node* n, datatype& key, double& norm, Node* res) {
  if (n == nullptr) {
    return;
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

int main(int argc, char* argv[]) {
  //   ros::init(argc, argv, "example_binary_search_tree");
  //   ros::NodeHandle nh;

  std::random_device device();  // Random number device
  std::mt19937 rng(0);          // RNG, 0, time(0), device
  std::uniform_int_distribution<int> dist(-1000, 1000);     // Distributation
  vector<datatype> data = {11, 13, -10, -99, 99, 50, -52};  // Data
  Node* tree = nullptr;                                     // Binary Tree
  auto start = chrono::high_resolution_clock::now();  // Time profiling variable
  auto stop = chrono::high_resolution_clock::now();   // Time profiling variable
  auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);

  // Populate tree
  cout << "List: ";
  start = chrono::high_resolution_clock::now();
  data.resize(20);
  for (int i = 0; i < data.size(); i++) {
    data[i] = dist(rng);
    tree = insert(tree, data[i]);
    cout << data[i] << " ";
  }
  stop = chrono::high_resolution_clock::now();
  cout << endl;
  chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Elapsed time: " << duration.count() << " uSec" << endl;

  // Pre-order traversal (DFS)
  cout << "Pre-order: ";
  start = chrono::high_resolution_clock::now();
  preOrder(tree);
  stop = chrono::high_resolution_clock::now();
  cout << endl;
  chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Elapsed time: " << duration.count() << " uSec" << endl;

  // In-order traversal (DFS)
  cout << "In-order: ";
  start = chrono::high_resolution_clock::now();
  inOrder(tree);
  stop = chrono::high_resolution_clock::now();
  cout << endl;
  chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Elapsed time: " << duration.count() << " uSec" << endl;

  // Post-order traversal (DFS)
  cout << "Post-order: ";
  start = chrono::high_resolution_clock::now();
  postOrder(tree);
  stop = chrono::high_resolution_clock::now();
  cout << endl;
  chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Elapsed time: " << duration.count() << " uSec" << endl;

  // Tree height
  start = chrono::high_resolution_clock::now();
  cout << "Tree Height: " << treeHeight(tree) << endl;
  stop = chrono::high_resolution_clock::now();
  chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Elapsed time: " << duration.count() << " uSec" << endl;

  // Search for the key
  datatype key = (argc > 1) ? atoi(argv[1]) : 10;
  start = chrono::high_resolution_clock::now();
  cout << "Search: " << key << ", result=" << search(tree, key) << endl;
  stop = chrono::high_resolution_clock::now();
  chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Elapsed time: " << duration.count() << " uSec" << endl;

  // Search for the min key
  start = chrono::high_resolution_clock::now();
  cout << "Min: " << findMin(tree)->data << endl;
  stop = chrono::high_resolution_clock::now();
  chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Elapsed time: " << duration.count() << " uSec" << endl;

  // Search for the max key
  start = chrono::high_resolution_clock::now();
  cout << "Max: " << findMax(tree)->data << endl;
  stop = chrono::high_resolution_clock::now();
  chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Elapsed time: " << duration.count() << " uSec" << endl;

  return 0;
}