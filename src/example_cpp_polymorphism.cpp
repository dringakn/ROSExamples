#include <fstream>   // file operations
#include <iostream>  // cout, etc.
#include <memory>    // make_shared, etc.

/*
  C++ polymorphism refers to the ability of objects of different classes to be
  treated as objects of a common base class. It allows you to write code that
  can work with objects of different types, as long as they share a common
  interface or base class.

  There are two main types of polymorphism in C++: compile-time polymorphism
  (achieved through function overloading and templates) and runtime
  polymorphism (achieved through virtual functions and inheritance). Here,
  we'll focus on runtime polymorphism using virtual functions and inheritance.

  In C++, you can define a base class that contains virtual functions. A
  virtual function is a member function that is declared in the base class and
  can be overridden in derived classes. The base class typically provides a
  common interface, while the derived classes can provide their own
  implementations of the virtual functions.

  To enable polymorphic behavior, you usually use pointers or references to
  the base class type. When you call a virtual function on a pointer or
  reference to a base class, the actual function implementation of the derived
  class is invoked based on the dynamic type of the object being pointed to or
  referenced.
*/

class Shape {
 public:
  virtual void draw() {
    // Default implementation for base class
    std::cout << "Drawing a shape." << std::endl;
  }
};

class Circle : public Shape {
 public:
  void draw() override { std::cout << "Drawing a circle." << std::endl; }
};

class Square : public Shape {
 public:
  void draw() override { std::cout << "Drawing a square." << std::endl; }
};

int main() {
  /*
    In this example, we define a base class Shape with a virtual function
    draw(). We also have two derived classes, Circle and Square, which override
    the draw() function with their own implementations.

    In the main() function, we create two pointers of type Shape* that are
    assigned objects of derived classes (Circle and Square). When we call the
    draw() function on these pointers, the appropriate derived class
    implementation is invoked, demonstrating polymorphism.
  */
  Shape* shape1 = new Circle();
  Shape* shape2 = new Square();

  shape1->draw();  // Outputs "Drawing a circle."
  shape2->draw();  // Outputs "Drawing a square."

  delete shape1;
  delete shape2;

  return 0;
}
