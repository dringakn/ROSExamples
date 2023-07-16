#include <fstream>   // file operations
#include <iostream>  // cout, etc.
#include <memory>    // make_shared, etc.

class BaseClass {
 public:
  int varBaseClass;
  BaseClass() {}
  BaseClass(int var) : varBaseClass(var) {}
  virtual void foo() = 0;
};

class MyClass : public BaseClass {
 public:
  // Default constructor
  MyClass() {}

  // Parameterized constructor
  MyClass(int i, float f, const std::string& s, const int arr[])
      : myInt(i), myFloat(f), myString(s) {
    for (int j = 0; j < 5; j++) {
      myArray[j] = arr[j];
    }
  }

  void print() {
    std::cout << "myInt: " << myInt << "\n";
    std::cout << "myFloat: " << myFloat << "\n";
    std::cout << "myString: " << myString << "\n";
    std::cout << "myArray: ";
    for (int i = 0; i < 5; i++) {
      std::cout << myArray[i] << " ";
    }
    std::cout << "\n";
  }
  void foo() override {}

  // Serialize object to a file
  void serializeObject(const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);

    if (outFile) {
      std::cout << "Searilized Object size: " << sizeof(*this) << std::endl;
      outFile.write(reinterpret_cast<const char*>(this), sizeof(*this));
      outFile.close();
      std::cout << "Object serialized successfully.\n";
    } else {
      std::cerr << "Failed to open file for writing.\n";
    }
  }

  // Deserialize object from a file
  void deserializeObject(const std::string& filename) {
    std::ifstream inFile(filename, std::ios::binary);

    if (inFile) {
      inFile.read(reinterpret_cast<char*>(this), sizeof(*this));
      inFile.close();
      std::cout << "Object deserialized successfully.\n";
    } else {
      std::cerr << "Failed to open file for reading.\n";
    }
  }

 private:
  int myInt;
  float myFloat;
  std::string myString;
  int myArray[5];
};

int main(int argc, char const* argv[]) {
  // Create an object of MyClass
  int numbers[5] = {1, 2, 3, 4, 5};
  MyClass obj1(42, 3.14f, "Hello, World!", numbers);

  // Serialize the object to a file
  obj1.serializeObject("serialized_object.bin");

  // Create a new object
  MyClass obj2;

  // Deserialize the object from the file
  obj2.deserializeObject("serialized_object.bin");

  // Display the deserialized object's values
  std::cout << "Deserialized Object:\n";
  obj2.print();

  std::shared_ptr<BaseClass> ptr =
      std::make_shared<MyClass>(99, 6.28f, "Bello!", numbers);
  // Accessing the member function of DerivedClass through base class pointer.
  std::shared_ptr<MyClass> derivedPtr = std::dynamic_pointer_cast<MyClass>(ptr);
  derivedPtr->print();
  if (derivedPtr) {
    derivedPtr->serializeObject("serialized_object.bin");
    derivedPtr->deserializeObject("serialized_object.bin");
    derivedPtr->print();
  }
  return 0;
}
