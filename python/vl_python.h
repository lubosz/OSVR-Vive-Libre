#include "vl_driver.h"
#include <string>
#include <vector>
#include <map>

class ViveLibre {
 public:
  ViveLibre();
  ~ViveLibre();
  void connect();
  std::map<int, std::vector<float>> pollAngles();
};
