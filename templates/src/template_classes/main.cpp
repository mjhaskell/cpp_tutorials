#include <iostream>
#include "template_classes/my_vector.hpp"

int main()
{
  MyVector<int> vec{5};
  std::cout << "size: " << vec.size() << std::endl;
  std::cout << "capacity: " << vec.capacity() << std::endl;

  for (int i{0}; i < 5; ++i)
  {
    vec.push_back(i*i);
    std::cout << "Inserted " << vec[i] << std::endl;
  }

  std::cout << "size: " << vec.size() << std::endl;
  std::cout << "capacity: " << vec.capacity() << std::endl;

  return 0;
}
