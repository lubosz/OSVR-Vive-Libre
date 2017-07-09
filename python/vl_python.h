#include "vl_driver.h"
#include <string>
#include <vector>

class ViveLibre {
  ohmd_context* ctx;
  ohmd_device* hmd;
  int num_devices;
  
 public:
  ViveLibre();
  ~ViveLibre();

  std::vector<float> rotation;
  std::vector<float> leftprojectionmatrix;
  std::vector<float> leftviewmatrix;
  std::vector<float> rightprojectionmatrix;
  std::vector<float> rightviewmatrix;

  void inputLoop();
  void printDeviceInfo();
  void poll();
  void printSensors();
  void reset();

 private:
  void print(std::string name, int len, ohmd_float_value val);
  void sleep(double seconds);
  void connect();
};
