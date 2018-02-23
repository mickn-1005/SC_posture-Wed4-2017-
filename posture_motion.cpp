// simulator of S/C's posture controll

#include <iostream>
#include <cassert>
#include <cmath>

#include "posture.h"
#include "posture.cpp"

int main(void) {
  Eigen::Vector4d q,p;
  Eigen::Vector3d o;
  Eigen::VectorXd stat(7);
  q = q0;
  o = omega0;
  PostureEOM eom;

  Eigen::VectorXd result[times];

  eom.state_initialize(0, 0, 0);
  if(eom.setstate(&q, &o)){
    for(int itr = 0; itr < times; itr++){
      eom.torque();
      result[itr] = eom.nextstate();
      if(itr %1000 ==0){
        std::cout << "----" << '\n';
        std::cout << eom.getstate() << '\n';
        std::cout << "----" << '\n';
      }
    }
    FILE *quats=nullptr;
    char name[30] = "quat_eom.csv";

    csvw(quats, name, result);

    std::cout << "----" << '\n';
  }
  return 0;
}
