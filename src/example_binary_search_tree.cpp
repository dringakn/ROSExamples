#include <ros/ros.h>

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
  vector<datatype> data = {11, 13, -10, -99, 99, 50, -52};  // Data
  Node* tree = nullptr;                                     // Binary Tree

  // Populate tree
  cout << "List: ";
  for (int i = 0; i < data.size(); i++) {
    tree = insert(tree, data[i]);
    cout << data[i] << " ";
  }
  cout << endl;

  // Pre-order traversal (DFS)
  cout << "Pre-order: ";
  preOrder(tree);
  cout << endl;

  // In-order traversal (DFS)
  cout << "In-order: ";
  inOrder(tree);
  cout << endl;

  // Post-order traversal (DFS)
  cout << "Post-order: ";
  postOrder(tree);
  cout << endl;

  // Tree height
  cout << "Tree Height: " << treeHeight(tree) << endl;

  // Search for the key
  datatype key = (argc > 1) ? atoi(argv[1]) : 10;
  cout << "Search: " << key << ", result=" << search(tree, key) << endl;

  // Search for the min key
  cout << "Min: " << findMin(tree)->data << endl;

  // Search for the max key
  cout << "Max: " << findMax(tree)->data << endl;

  return 0;
}