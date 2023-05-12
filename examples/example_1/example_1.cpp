#include <Eigen/Core>

#include "drift/math/lie_group.h"

using namespace math;

int main(int argc, char** argv) {
  Eigen::Vector3d v;
  v << 3, 2, 3;
  std::cout << "v is: \n" << v << std::endl;
  auto skew_v = lie_group::skew(v);
  std::cout << "skew v is: \n" << skew_v << std::endl;
}