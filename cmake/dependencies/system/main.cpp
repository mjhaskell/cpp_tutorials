#include <iostream>
#include <Eigen/Core>


int main()
{
  Eigen::Matrix2d mat;
  mat << 1,2, 3,4;
  std::cout << mat << std::endl;

  return 0;
}
