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
  p = {0,0,0,0};
  q = q0;
  o = omega0;
  PostureEOM eom;

  vecs vecs;
  vecs.disp(&p);

  eom.state_initialize();
  if(eom.setstate(&q, &o)){
    // eom.nextstate();
    for(int itr = 0; itr < times; itr++){
      eom.nextstate();
      if(itr %1000 ==0){
        std::cout << eom.getstate() << '\n';
        std::cout << "----" << '\n';
      }
    }
  }

  std::cout << "----" << '\n';
  // std::cout << p << '\n';
  // std::cout << "----" << '\n';

  // vecs.disp(&p);

  return 0;
}
