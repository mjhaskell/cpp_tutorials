// Source file for template class MyVector
#ifndef MY_VECTOR_HPP
#include "my_vector.hpp" // For IDE purposes only, include never happens
#endif

template <typename T>
MyVector<T>::MyVector(const int capacity) :
  data_{new T[capacity]},
  capacity_{capacity},
  size_{0}
{
}

template <typename T>
MyVector<T>::~MyVector()
{
  delete[] data_;
}

template <typename T>
int MyVector<T>::size() const
{
  return size_;
}

template <typename T>
int MyVector<T>::capacity() const
{
  return capacity_;
}

template <typename T>
void MyVector<T>::push_back(T value)
{
  data_[size_++] = value;
  if (size_ >= capacity_)
  {
    T *copy = new T[capacity_ + 50];
    for (int i{0}; i < capacity_; ++i)
      copy[i] = data_[i];
    delete[] data_;
    data_ = copy;
    capacity_ += 50;
  }
}

template <typename T>
T MyVector<T>::operator[](int i) const
{
  return data_[i];
}
