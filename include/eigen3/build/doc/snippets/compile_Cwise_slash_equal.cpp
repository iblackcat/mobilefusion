#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

int main(int, char**)
{
  cout.precision(3);
  Array3d v(3,2,4), w(5,4,2);
v /= w;
cout << v << endl;

  return 0;
}
