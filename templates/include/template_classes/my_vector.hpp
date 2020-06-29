#ifndef MY_VECTOR_HPP
#define MY_VECTOR_HPP

template <typename T>
class MyVector
{
public:
  MyVector(const int capacity = 50);
  virtual ~MyVector();
  int size() const;
  int capacity() const;
  void push_back(T value);
  T operator[](int i) const;

private:
  T *data_;
  int capacity_;
  int size_;
};

#include "my_vector.tpp"

#endif // MY_VECTOR_HPP
