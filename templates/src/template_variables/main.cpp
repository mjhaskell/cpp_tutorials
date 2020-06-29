#include <iostream>
using namespace std;


template <typename T>
constexpr T pi = T(3.1415926535897932385L);

template <typename T>
T calcAreaOfCircle(const T &radius)
{
  return pi<T>*radius*radius;
}


int main()
{
  cout.precision(20);
  cout << "float:\n" << pi<float> << endl;
  cout << calcAreaOfCircle<float>(2) << endl << endl;

  cout << "double:\n" << pi<double> << endl;
  cout << calcAreaOfCircle<double>(2) << endl << endl;

  cout << "long double:\n" << pi<long double> << endl;
  cout << calcAreaOfCircle<long double>(2) << endl;

  return 0;
}
