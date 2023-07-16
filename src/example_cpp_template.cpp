#include <fstream>   // file operations
#include <iostream>  // cout, etc.
#include <memory>    // make_shared, etc.

/*
An abstract class is a class that is designed to be used as a base class for
other classes, but cannot be instantiated on its own. It serves as a blueprint
for derived classes, providing a common interface and defining certain behaviors
that derived classes must implement.

An abstract class is declared by using the virtual keyword in the class
declaration and including at least one pure virtual function. A pure virtual
function is declared by appending = 0 to its declaration, indicating that it has
no implementation in the abstract class.

Since an abstract class cannot be instantiated, it is considered an incomplete
type. You cannot create objects of the abstract class type, but you can use
pointers and references to the abstract class.

Any class that derives from an abstract class must provide definitions for all
the pure virtual functions declared in the abstract class. If a derived class
fails to implement any pure virtual function, it will also become an abstract
class.

Abstract classes provide a way to achieve polymorphic behavior in C++. You can
use pointers or references of the abstract class type to invoke functions and
perform operations specific to the derived classes. This allows for treating
different derived classes as instances of the abstract base class, enabling code
reuse and flexibility.
*/
class AbstractClass {
 public:
  /*
    In the example above, pure_virutal_fun() is a pure virtual function, denoted
    by the = 0 at the end of the function declaration. This indicates that any
    class deriving from AbstractClass must provide its own implementation for
    pure_virutal_fun().
 */
  virtual void pure_virutal_fun() = 0;  // pure virtual function

  virtual void virtual_fun()  // virtual function
  {
    /*
    A virtual function in C++ is a member function of a class that can be
    overridden in its derived classes. When a virtual function is called on a
    base class pointer or reference, the appropriate function in the derived
    class is invoked, based on the actual type of the object pointed to or
    referred to.

    The virtual keyword is used to enable dynamic polymorphism in C++. When a
    function is declared as virtual, it allows the function to be overridden in
    derived classes. This means that when you have a pointer or reference to a
    base class object that points to or refers to a derived class object, the
    appropriate function implementation will be called based on the actual type
    of the object.
    */
    std::cout << "virtual_fun: Base/Abstract Class" << std::endl;
  }

  /*
    Implementation of a non-virtual function.
    Used by all derived classes.
  */
  void non_virtual_fun() {
    std::cout << "non_virtual_fun: AbstractClass" << std::endl;
  }
};

// Derived (Concrete) Class
class ConcreteClass1 : public AbstractClass {
 public:
  void pure_virutal_fun() override {
    std::cout << "pure_virutal_fun: Derived/Concrete Class1" << std::endl;
  }
};

// Derived (Concrete) Class
class ConcreteClass2 : public AbstractClass {
 public:
  void pure_virutal_fun() override {
    std::cout << "pure_virutal_fun: Derived/Concrete Class2" << std::endl;
  }

  void virtual_fun() override {
    std::cout << "virtual_fun: Derived/Concrete Class2" << std::endl;
  }
};

// Derived (Incomplete) Class: No pure virtual function implementation
class IncompleteClass1 : public AbstractClass {
 public:
  void virtual_fun() override {
    std::cout << "virtual_fun: Derived/Incomplete Class1" << std::endl;
  }
};
void performOperation(AbstractClass* obj) {
  // Polymorphic behavior based on the actual object type
  obj->non_virtual_fun();   // call base-class function
  obj->pure_virutal_fun();  // derived class specific
  obj->virtual_fun();       // override able funtion
}

int main(int argc, char const* argv[]) {
  //   AbstractClass ac; // Error: Can not instinate.
  //   IncompleteClass1 obj3;  // Error: can't instinate.
  ConcreteClass1 obj1;
  ConcreteClass2 obj2;

  // instance of the ConcreteClass1 as AbstractClass pointer
  performOperation(&obj1);
  performOperation(&obj2);

  // Explicit declerations of the pointers object.
  AbstractClass* ptr1 = &obj1;
  ptr1->virtual_fun();
  AbstractClass* ptr2 = new ConcreteClass1();
  ptr2->virtual_fun();
  AbstractClass* ptr3 = new ConcreteClass2();
  ptr3->virtual_fun();

  // Dynamically create object, referenced by multiple pointers
  std::cout << "Shared Memory:" << std::endl;
  std::shared_ptr<AbstractClass> ptr4 = std::make_shared<ConcreteClass2>();
  // share object between ptr4 and ptr5
  std::shared_ptr<AbstractClass> ptr5 = ptr4;
  ptr4->virtual_fun();
  ptr5->virtual_fun();

  return 0;
}
