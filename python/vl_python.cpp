#include <time.h>
#include <sys/time.h>
#include <stdio.h>

#include "vl_python.h"

ViveLibre::ViveLibre() {
  rotation = std::vector<float>(4);
  leftprojectionmatrix = std::vector<float>(16);
  rightprojectionmatrix = std::vector<float>(16);
  leftviewmatrix = std::vector<float>(16);
  rightviewmatrix = std::vector<float>(16);
  //connect();
}

ViveLibre::~ViveLibre() {
}

void ViveLibre::connect() {
}

void ViveLibre::printDeviceInfo() {

}

void ViveLibre::inputLoop() {

}

void ViveLibre::printSensors() {
}

void ViveLibre::poll() {
}

void ViveLibre::sleep(double seconds) {
}


// gets float values from the device and prints them
void ViveLibre::print(std::string name, int len, ohmd_float_value val) {
}

void ViveLibre::reset(){
}
