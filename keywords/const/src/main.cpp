#include <iostream>

int main()
{
  int a{0}, b{1};
  const int *ci_ptr = &a;
  int *const i_cptr = &a;
  const int *const ci_cptr = &a;

  std::cout << "a=" << a << ", b=" << b << std::endl;
  std::cout << *ci_ptr << " " << *i_cptr << " " << *ci_cptr << std::endl;

  ci_ptr = &b;
  // *ci_cptr = 7; //will not compile

  std::cout << "a=" << a << ", b=" << b << std::endl;
  std::cout << *ci_ptr << " " << *i_cptr << " " << *ci_cptr << std::endl;

  *i_cptr = 7;
  // i_cptr = &b; // will not compile

  std::cout << "a=" << a << ", b=" << b << std::endl;
  std::cout << *ci_ptr << " " << *i_cptr << " " << *ci_cptr << std::endl;

  b = *ci_cptr + 2;

  std::cout << "a=" << a << ", b=" << b << std::endl;
  std::cout << *ci_ptr << " " << *i_cptr << " " << *ci_cptr << std::endl;

  return 0;
}
