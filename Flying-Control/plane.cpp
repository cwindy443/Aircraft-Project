#include <iostream>

class Plane {
public:
  
  void deploy() {
    std::cout << "Plane initialized." << std::endl;
  }
  
};

int main() {
  Plane aircraft;
  aircraft.deploy();
  return 0;
}