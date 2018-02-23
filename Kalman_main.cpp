//simulator of Kalman Filter

#include <iostream>
#include <cmath>
#include <random>
#include <fstream>

#include "posture.h"
#include "posture.cpp"

int main(void) {
  Eigen::Vector4d q, qest;  // True value, observed value, estimated value
  Eigen::Vector3d o, oest, DCMtrue;

  q = q0;
  qest = {initial(iniest),initial(iniest),initial(iniest),initial(iniest)}; //初期推定値を生成
  qest += q0;
  o = omega0;
  oest = {initial(iniest),initial(iniest),initial(iniest)};
  oest += omega0;
  PostureEOM eom;   // True motion
  PostureEOM est;   // estimated motion

  Eigen::VectorXd result[times];
  Eigen::VectorXd estimate[times];

  eom.state_initialize(0, wnoisy, 0);
  est.state_initialize(Kalman, 0, vnoisy);

  if(eom.setstate(&q, &o) && est.setstate(&qest,&oest)){
    for(int itr = 0; itr < times; itr++){
      eom.torque();
      est.torque();
      result[itr] = eom.nextstate();
      if(itr%(int)(1/delt)==0){
        DCMtrue = eom.getDCM().col(DCMind(indgen));
        est.nextstate();
        estimate[itr] = est.Kalman_observe(DCMtrue);
      }
      else{
        estimate[itr] = est.nextstate();
      }

      if(itr %1000 ==0){
        std::cout << "----" << '\n';
        std::cout << "itr No..." << itr << '\n';
        std::cout << result[itr] << '\n';
        std::cout << "----" << '\n';
        std::cout << estimate[itr] << '\n';
        std::cout << "----" << '\n';
        std::cout << est.getqua().norm() << '\n';
        std::cout << "----" << '\n';
      }
    }

    FILE *Kalman_true=nullptr;
    FILE *Kalman_estm=nullptr;
    char Kalt[30] = "Kalman_true.csv";
    char Kale[30] = "Kalman_estm.csv";

    csvw(Kalman_true, Kalt, result);
    csvw(Kalman_estm, Kale, estimate);

    std::cout << "----" << '\n';
  }

  return 0;
}
