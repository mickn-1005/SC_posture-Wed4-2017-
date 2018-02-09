#include <iostream>
#include "Eigen/Core"
#include "posture.h"

void PostureEOM::state_initialize(){
  state.resize(7);
}

int PostureEOM::setstate(Eigen::Vector4d *qua, Eigen::Vector3d *ome){
  Eigen::VectorXd sta(7);
  if(qua->size() == 4){
    quata = *qua;
    sta(0) = quata(0);
    sta(1) = quata(1);
    sta(2) = quata(2);
    sta(3) = quata(3);
  }
  else{
    std::cout << "setstate error: invalid size of quat...:" << qua->size() << '\n';
    return 0;   //fail
  }
  if(ome->size() == 3){
    omega = *ome;
    sta(4) = omega(0);
    sta(5) = omega(1);
    sta(6) = omega(2);
  }
  else{
    std::cout << "setstate error: invalid size of omeg...:" << ome->size() << '\n';
    return 0;   //fail
  }
  state = sta;
  std::cout << "finish setting state" << '\n';
  return 1;
}

Eigen::Vector4d PostureEOM::getqua() {
  return quata;
}

Eigen::Vector3d PostureEOM::getome(){
  return omega;
}

Eigen::VectorXd PostureEOM::getstate(){
  return state;
}

Eigen::VectorXd PostureEOM::calcdelstate(){
  Eigen::VectorXd sta(7);
  Eigen::VectorXd delsta(7);
  sta = state;

  delsta(0) = 0.5*(-sta(1)*sta(4) - sta(2)*sta(5) - sta(3)*sta(6));
  delsta(1) = 0.5*(+sta(0)*sta(4) - sta(3)*sta(5) + sta(2)*sta(6));
  delsta(2) = 0.5*(+sta(3)*sta(4) + sta(0)*sta(5) + sta(1)*sta(6));
  delsta(3) = 0.5*(-sta(2)*sta(4) + sta(1)*sta(5) + sta(0)*sta(6));

  delsta(4) = (TCx + TNx - (Iz-Iy) * sta(5)*sta(6)) / Ix;
  delsta(5) = (TCy + TNy - (Ix-Iz) * sta(6)*sta(4)) / Iy;
  delsta(6) = (TCz + TNz - (Iy-Ix) * sta(5)*sta(4)) / Iz;

  delstate = delsta;
  return delstate;
}

Eigen::VectorXd PostureEOM::nextstate(){
  // std::cout << "calc nextstate..." << '\n';
  Eigen::VectorXd k0(7);
  Eigen::VectorXd k1(7);
  Eigen::VectorXd k2(7);
  Eigen::VectorXd k3(7);
  Eigen::VectorXd g1(7);
  Eigen::VectorXd g2(7);
  Eigen::VectorXd g3(7);
  Eigen::VectorXd sta0(7);

  sta0 = state;

  k0 = calcdelstate() * delt;
  state = state + 0.5 * k0;
  g1 = k0;

  k1 = calcdelstate() * delt;
  state = state + (1 - 0.5 * sqrt(2)) * (k1-g1);
  g2 = (-2 + 3/(sqrt(2))) * g1 + (2-sqrt(2))*k1;

  k2 = calcdelstate() * delt;
  state = state + (1 + 1/(sqrt(2)))*(k2-g2);
  g3 = -(2+3/(sqrt(2)))*g2 + (2+sqrt(2))*k2;

  k3 = calcdelstate() * delt;

  state = sta0 + 1/6.0 * (k0 + (2-sqrt(2))*k1 + (2+sqrt(2))*k2 + k3);

  return state;
}

void PostureEOM::showqua() {
  std::cout << "show quaternion..." << '\n';
  std::cout << quata << '\n';
}


void vecs::disp(Eigen::Vector4d *vec) {
  std::cout << *vec << '\n';
}
