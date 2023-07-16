#include <fstream>
#include <iostream>
#include <memory>

template <typename T>
void saveSharedPtrToFile(const std::string& filename,
                         const std::shared_ptr<T>& ptr) {
  std::ofstream file(filename, std::ios::binary);
  if (file) {
    // Write a flag indicating whether the pointer is null or not
    bool isNull = (ptr == nullptr);
    file.write(reinterpret_cast<const char*>(&isNull), sizeof(isNull));

    if (!isNull) {
      // Write the object pointed to by the shared_ptr
      file.write(reinterpret_cast<const char*>(ptr.get()), sizeof(T));
    }

    std::cout << "shared_ptr saved to file: " << filename << std::endl;
  } else {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
  }
}

template <typename T>
std::shared_ptr<T> loadSharedPtrFromFile(const std::string& filename) {
  std::ifstream file(filename, std::ios::binary);
  if (file) {
    // Read the flag indicating whether the pointer is null or not
    bool isNull;
    file.read(reinterpret_cast<char*>(&isNull), sizeof(isNull));

    if (!isNull) {
      // Read the object from the file
      auto ptr = std::make_shared<T>();
      file.read(reinterpret_cast<char*>(ptr.get()), sizeof(T));

      std::cout << "shared_ptr loaded from file: " << filename << std::endl;
      return ptr;
    } else {
      std::cout << "Null shared_ptr loaded from file: " << filename
                << std::endl;
      return nullptr;
    }
  } else {
    std::cerr << "Failed to open file for reading: " << filename << std::endl;
    return nullptr;
  }
}

// Example struct for serialization
class MyData {
 public:
  int value1;
  float value2;
};

int main() {
  // Create a shared_ptr object
  std::shared_ptr<MyData> myDataPtr = std::make_shared<MyData>();
  myDataPtr->value1 = 42;
  myDataPtr->value2 = 3.14f;

  // Save the shared_ptr to a file
  saveSharedPtrToFile("mydata_data.bin", myDataPtr);

  // Load the shared_ptr from the file
  std::shared_ptr<MyData> loadedDataPtr =
      loadSharedPtrFromFile<MyData>("mydata_data.bin");

  // Check if the loaded shared_ptr is valid
  if (loadedDataPtr) {
    std::cout << "Loaded Data: value1=" << loadedDataPtr->value1
              << ", value2=" << loadedDataPtr->value2 << std::endl;
  } else {
    std::cout << "Failed to load shared_ptr" << std::endl;
  }

  return 0;
}
