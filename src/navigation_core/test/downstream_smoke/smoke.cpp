#include "navigation_core/path_utils.hpp"

int main()
{
  navigation::navigation_core::Path2D path;
  return navigation::navigation_core::pathLengthM(path) == 0.0 ? 0 : 1;
}
