#include <fstream>   // file operations
#include <iostream>  // cout, etc.
#include <memory>    // make_shared, etc.
#include <vector>

/*
A shared pointer is a smart pointer that provides automatic memory management by
keeping track of the number of references to an object. It allows multiple
shared pointers to share ownership of the same dynamically allocated object.

Shared pointers are part of the C++ Standard Library and can be used to manage
the lifetime of objects, especially when dealing with dynamically allocated
memory. They help prevent memory leaks by automatically releasing the memory
when there are no more shared pointers referring to it.
*/

class BaseClass {
 public:
  BaseClass() { std::cout << "BaseClass created." << std::endl; }

  ~BaseClass() { std::cout << "BaseClass destroyed." << std::endl; }

  virtual void someFunction() { std::cout << "BaseClass" << std::endl; }
};

class DerivedClass1 : public BaseClass {
 public:
  DerivedClass1() { std::cout << "DerivedClass1 created." << std::endl; }

  ~DerivedClass1() { std::cout << "DerivedClass1 destroyed." << std::endl; }

  void someFunction() override { std::cout << "DerivedClass1" << std::endl; }
  void someFunction2() {
    std::cout << "DerivedClass1: someFunction2" << std::endl;
  }
};

class DerivedClass2 : public BaseClass {
 public:
  DerivedClass2() { std::cout << "DerivedClass2 created." << std::endl; }

  ~DerivedClass2() { std::cout << "DerivedClass2 destroyed." << std::endl; }

  void someFunction() override { std::cout << "DerivedClass2" << std::endl; }
  void someFunction2() {
    std::cout << "DerivedClass2: someFunction2" << std::endl;
  }
};

int main() {
  /*
  In this example, we create a class BaseClass that simply prints messages when
  it is created and destroyed, and has a member function someFunction(). In the
  main() function, we create two shared pointers ptr1 and ptr2, both pointing to
  a dynamically allocated BaseClass object created using make_shared. Since both
  shared pointers point to the same object, they share ownership.

  We can use the shared pointers just like regular pointers, calling member
  functions and accessing data members of the object. When the last shared
  pointer goes out of scope or is reset, the object is automatically destroyed,
  and its destructor is called.

  Shared pointers provide a convenient and safe way to manage dynamically
  allocated objects in C++, reducing the risk of memory leaks and dangling
  pointers.
  */

  // Dynamically create the object and share it's memory location.
  std::shared_ptr<BaseClass> ptr1 = std::make_shared<BaseClass>();

  // Create a new pointer, pointing to the same memory.
  // Both pointers now point to the same object
  std::shared_ptr<BaseClass> ptr2 = ptr1;

  ptr1->someFunction();
  ptr2->someFunction();

  // The object is automatically destroyed when the last shared_ptr goes out of
  // scope

  /*
  std::dynamic_pointer_cast is a powerful feature in C++ that allows you to
  perform runtime type checking and type conversion on std::shared_ptr objects.
  It is primarily used in scenarios where you have a base class pointer (or
  std::shared_ptr) and you need to check if it points to a derived class object
  and perform operations specific to the derived class.

  Here are some common use cases for std::dynamic_pointer_cast:

      Polymorphic behavior: dynamic_pointer_cast is often used in situations
  where you have a polymorphic hierarchy of classes and you want to call
  overridden functions specific to the derived class through a base class
  pointer or std::shared_ptr. By using dynamic_pointer_cast, you can safely
  downcast the base class pointer to a derived class pointer and call the
  derived class's member functions.

      Accessing derived class functionality: When you have a base class pointer
  or std::shared_ptr and you know that it actually points to a derived class
  object, you can use dynamic_pointer_cast to convert it to a std::shared_ptr of
  the derived class type. This allows you to access and utilize the additional
  member functions and data specific to the derived class.

      Event handling and callbacks: In event-driven systems or callback
  mechanisms, you may receive a base class pointer or std::shared_ptr to a
  generic event object, but you need to handle it differently based on the
  specific event type. dynamic_pointer_cast can be used to perform runtime type
  checking and safely downcast the event object to the appropriate derived
  class, allowing you to handle the event specific to its type.

      Visitor pattern: The Visitor pattern involves performing operations on
  different types of objects in a hierarchy without modifying the classes
  themselves. By combining std::dynamic_pointer_cast with the Visitor pattern,
  you can dispatch the appropriate visit operation based on the actual type of
  the object pointed to by the base class pointer or std::shared_ptr.

  Object serialization and deserialization: When serializing and deserializing
  objects, std::dynamic_pointer_cast can be utilized to properly cast pointers
  to derived classes during the deserialization process. This is especially
  useful when dealing with inheritance hierarchies where objects of different
  derived classes are stored in a container, such as a vector or a map, and need
  to be correctly restored to their original types.

  It's important to note that the use of std::dynamic_pointer_cast assumes that
  you have a valid and compatible hierarchy of classes. If the base class is not
  polymorphic (lacks a virtual function), or the classes in the hierarchy do not
  follow the necessary inheritance relationships, dynamic_pointer_cast may lead
  to unexpected behavior or runtime errors.

  In summary, std::dynamic_pointer_cast is a useful tool for performing runtime
  type checking and type conversion on std::shared_ptr objects in situations
  where you need to work with derived class functionality or perform operations
  specific to derived classes. It enables safe downcasting and facilitates
  polymorphic behavior in C++ programs.
  */

  // Accessing the member function of DerivedClass through base class pointer.
  std::shared_ptr<BaseClass> ptr = std::make_shared<DerivedClass1>();
  ptr->someFunction();
  std::shared_ptr<DerivedClass1> derivedPtr =
      std::dynamic_pointer_cast<DerivedClass1>(ptr);
  if (derivedPtr) {
    derivedPtr->someFunction2();
  }

  std::vector<std::shared_ptr<BaseClass>> objects;
  objects.push_back(std::make_shared<DerivedClass1>());
  objects.push_back(std::make_shared<DerivedClass2>());

  // Serialize objects to a file

  // Deserialize objects from the file
  // and restore them to their original types

  for (const auto& obj : objects) {
    // Use dynamic_pointer_cast to downcast
    // and invoke derived class-specific functions
    auto derivedObj = std::dynamic_pointer_cast<DerivedClass1>(obj);
    if (derivedObj) {
      derivedObj->someFunction();  // Output: "DerivedClass1"
    }
  }

  /*
    Apart from std::dynamic_pointer_cast, there are other useful casts
  available in C++ for handling polymorphism and type conversions. Let's explore
  a few of them:

    static_cast: static_cast is a compile-time cast used for implicit and
  explicit type conversions between compatible types. It is commonly used to
  convert between fundamental data types, perform upcasting (converting a
  pointer/reference to a base class to a pointer/reference to a derived class),
  and perform downcasting (converting a pointer/reference to a derived class to
  a pointer/reference to a base class). However, unlike dynamic_cast,
  static_cast does not perform runtime type checking and is not safe for all
  downcasting scenarios.

    reinterpret_cast: reinterpret_cast is used to convert one pointer type to
  another pointer type, even if they are unrelated. It is typically used for
  low-level type punning and converting between pointers and integral types.
  However, reinterpret_cast is a dangerous cast and should be used with caution,
  as it bypasses type safety and can lead to undefined behavior if misused.

    const_cast: const_cast is used to add or remove const, volatile, or const
  volatile qualifiers from a pointer or reference. It is primarily used to
  remove const-ness and volatile-ness when necessary, allowing modifications to
  a variable that is initially declared as const or volatile.

    dynamic_cast: dynamic_cast is primarily used for safe downcasting in
  polymorphic class hierarchies. It performs runtime type checking to ensure
  that the conversion is valid. If the conversion is not possible, dynamic_cast
  returns a null pointer (when casting pointers) or throws a std::bad_cast
  exception (when casting references). dynamic_cast is useful when you need to
  check the compatibility of types during runtime, such as downcasting from a
  base class to a derived class.

  Each cast has its own specific use cases and considerations. It's important to
  choose the appropriate cast based on the requirements of your code and to
  understand the potential implications and limitations of each cast.

  Remember to use casts judiciously and ensure that they are used in situations
  where they are well-defined and necessary. Improper or unnecessary use of
  casts can introduce bugs and lead to code that is difficult to maintain.
  */
  return 0;
}