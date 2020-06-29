#include <iostream>
using namespace std;


template <typename T>
T maxof1(const T &lhs, const T &rhs)
{
  return (lhs > rhs) ? lhs : rhs;
}

template <typename TL, typename TR>
auto maxof2(const TL &lhs, const TR &rhs)
{
  return (lhs > rhs) ? lhs : rhs;
}


int main()
{
  double d1{1.2}, d2{0.5};
  float f1{3.9}, f2{1.1};

  // version 1
  cout << "Single template parameter:" << endl;
  cout << maxof1(d1, d2) << endl;
  cout << maxof1(f1, f2) << endl;
  cout << maxof1<double>(d1, f2) << endl;
  cout << maxof1<int>(f1, d2) << endl;
  // cout << maxof1(d1, f1) << endl; // doesn't compile
  // cout << maxof1(5, 3.14f) << endl; // doesn't compile

  // version 2
  cout << "\nMultiple template parameters:" << endl;
  cout << maxof2(d1, d2) << endl;
  cout << maxof2(f1, f2) << endl;
  cout << maxof2(d1, f2) << endl;
  cout << maxof2(f1, d2) << endl;
  cout << maxof2(d1, f1) << endl;
  cout << maxof2(5, 3.14f) << endl;

  return 0;
}
