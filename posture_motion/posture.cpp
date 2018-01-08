#include <iostream>
#include "posture.h"

void quaternion::setquat(std::vector<float> *vec){
  if(vec->size() == 4){
    quarter = vec;
  }
  else{
    std::cout << "setquat error: invalid size of vector...:" << vec->size() << '\n';
  }
}

std::vector<float> quaternion::getquat() {
  return *quarter;
}

// std::vector<float> quaternion::odequat(std::vector<float> *ome){
//
//   return *ome;
// }

void quaternion::showquat() {
  std::cout << "show quaternion..." << '\n';
  for(float x: *quarter){
    std::cout << x << '\n';
  }
}


void vecs::disp(std::vector<int> *vec) {
  for(int x : *vec){
    std::cout << x << '\n';
  }
}

void vecs::disp(std::vector<float> *vec) {
  for(float x : *vec){
    std::cout << x << '\n';
  }
}
