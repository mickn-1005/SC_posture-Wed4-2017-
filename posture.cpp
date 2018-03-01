#include <iostream>
#include "Eigen/Core"
#include "Eigen/LU"
#include "posture.h"

void PostureEOM::state_initialize(int kaltf, int noisetf, int obs_noitf){
  isKalman = kaltf;
  noisy = noisetf;
  obsnoise = obs_noitf;
  state.resize(7);
  if(isKalman==1){
    P.resize(7,7);
    Q.resize(4,4);
    R.resize(3,3);

    P = Eigen::MatrixXd::Identity(7,7) * conv_noise;

    if(noisy){
      Q = Eigen::MatrixXd::Identity(3,3) * conv_noise*conv_noise;
    }
    else{
      Q = Eigen::MatrixXd::Zero(3,3);
      // Q = Eigen::MatrixXd::Identity(3,3) * conv_noise*conv_noise;
    }
    if (obsnoise) {
      R = Eigen::MatrixXd::Identity(3,3) * conv_noise*conv_noise;
    }
    else{
      R = Eigen::MatrixXd::Zero(3,3);
    }
  }
}

int PostureEOM::setstate(Eigen::Vector4d *qua, Eigen::Vector3d *ome){
  Eigen::VectorXd sta(7);
  if(qua->size() == 4){
    Eigen::Vector4d quata = *qua;
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
    Eigen::Vector3d omega = *ome;
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

int PostureEOM::torque(){
  if (noisy) {
    TNx = dist(wnoise);
    TNy = dist(wnoise);
    TNz = dist(wnoise);
  }
  else{
    TCx = 0.0;
    TCy = 0.0;
    TCz = 0.0;
    TNx = 0.0;
    TNy = 0.0;
    TNz = 0.0;
  }
  return 0;
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
  if(isKalman==1){
    Eigen::MatrixXd A = EOSMatA();
    Eigen::MatrixXd B = EOSMatB();
    P = A * (P * A.transpose()) + B * (Q * B.transpose());
  }
  // Qnormal();
  return state;
}

Eigen::MatrixXd PostureEOM::EOSMatA(){
  Eigen::MatrixXd A(7,7);
  A << 0.0, -0.5*state(4), -0.5*state(5), -0.5*state(6), -0.5*state(1), -0.5*state(2), -0.5*state(3),
       0.5*state(4), 0.0, 0.5*state(6), -0.5*state(5), 0.5*state(0), -0.5*state(3), -0.5*state(2),
       0.5*state(5), -0.5*state(6), 0.0, 0.5*state(4), 0.5*state(3), 0.5*state(0), -0.5*state(1),
       0.5*state(6), 0.5*state(5), -0.5*state(4), 0.0, -0.5*state(2), 0.5*state(1), 0.5*state(0),
       0.0,0.0,0.0,0.0, 0.0, (Iy-Iz)*state(6)/Ix, (Iy-Iz)*state(5)/Ix,
       0.0,0.0,0.0,0.0, (Iz-Ix)*state(6)/Iy, 0.0, (Iz-Ix)*state(4)/Iy,
       0.0,0.0,0.0,0.0, (Ix-Iy)*state(5)/Iz, (Ix-Iy)*state(4)/Iz, 0.0;
  return A*delt + Eigen::MatrixXd::Identity(7,7);
}
Eigen::MatrixXd PostureEOM::EOSMatB(){
  Eigen::MatrixXd B(7,3);
  B << 0.0,0.0,0.0,
       0.0,0.0,0.0,
       0.0,0.0,0.0,
       0.0,0.0,0.0,
       1/Ix,0.0,0.0,
       0.0,1/Iy,0.0,
       0.0,0.0,1/Iz;
  return B*delt;
}
Eigen::MatrixXd PostureEOM::EOOMatH(int index){
  Eigen::MatrixXd H(3,7);
  if(index==0){
    H << 2.0*state(0),2.0*state(1),-2.0*state(2),-2.0*state(3),0.0,0.0,0.0,
         2.0*state(3),2.0*state(2),2.0*state(1),2.0*state(0),0.0,0.0,0.0,
         -2.0*state(2),2.0*state(3),-2.0*state(0),2.0*state(1),0.0,0.0,0.0;
  }
  else if(index==1){
    H << -2.0*state(3),2.0*state(2),2.0*state(1),-2.0*state(0),0.0,0.0,0.0,
         2.0*state(0),-2.0*state(1),2.0*state(2),-2.0*state(3),0.0,0.0,0.0,
         2.0*state(1),2.0*state(0),2.0*state(3),2.0*state(2),0.0,0.0,0.0;
  }
  else if(index==2){
    H << 2.0*state(2),2.0*state(3),2.0*state(0),2.0*state(1),0.0,0.0,0.0,
         -2.0*state(1),-2.0*state(0),2.0*state(3),2.0*state(2),0.0,0.0,0.0,
         2.0*state(0),-2.0*state(1),-2.0*state(2),2.0*state(3),0.0,0.0,0.0;
  }
  else{
    std::cout << "invalid index..." << index << '\n';
    return Eigen::MatrixXd::Zero(1,1);
  }
  return H;
}

Eigen::VectorXd PostureEOM::Kalman_observe(Eigen::Vector3d DCMvec){    //input: DCM of True Motion
  if(obsnoise){       // add noise of observation
    for(int i=0; i<DCMvec.rows(); i++){
      DCMvec(i) += dist(vnoise);
    }
  }
  Eigen::MatrixXd DCMest(3,3);
  DCMest = QtoDCM(state);

  Eigen::Vector3d norms;
  Eigen::Vector3d::Index minind;
  for(int i=0; i<3; i++){
    norms(i) = (DCMvec-DCMest.col(i)).norm();
  }
  norms.minCoeff(&minind);

  Eigen::Vector3d obs_vec = DCMvec - DCMest.col(minind);    //observed vector z
  Eigen::MatrixXd H = EOOMatH(minind);
  //観測行列の更新
  P = P - P*H.transpose()*(H*P*H.transpose()+R).inverse()*H*P;
  Eigen::MatrixXd K = P *(H.transpose()*R.inverse());
  state = state + K*obs_vec;        //propagation
  Qnormal();

  return state;
}

Eigen::MatrixXd PostureEOM::QtoDCM(Eigen::VectorXd Q){
  Eigen::MatrixXd DCM(3,3);
  DCM << std::pow(Q(0), 2.0)+std::pow(Q(1), 2.0)-std::pow(Q(2), 2.0)-std::pow(Q(3), 2.0),
         2.0*(Q(1)*Q(2)-Q(3)*Q(0)), 2.0*(Q(1)*Q(3)+Q(2)*Q(0)),

         2.0*(Q(1)*Q(2)+Q(3)*Q(0)),
         std::pow(Q(0), 2.0)-std::pow(Q(1), 2.0)+std::pow(Q(2), 2.0)-std::pow(Q(3), 2.0),
         2.0*(Q(2)*Q(3)-Q(1)*Q(0)),

         2.0*(Q(1)*Q(3)-Q(2)*Q(0)), 2.0*(Q(2)*Q(3)+Q(1)*Q(0)),
         std::pow(Q(0), 2.0)-std::pow(Q(1), 2.0)-std::pow(Q(2), 2.0)+std::pow(Q(3), 2.0);
  return DCM;
}

int PostureEOM::Qnormal(){
  Eigen::Vector4d quaqua;
  for(int i=0; i<4; i++){
    quaqua(i) = state(i);
  }
  quaqua.normalize();
  for(int i=0; i<4; i++){
    state(i) = quaqua(i);
  }
  return 0;
}

Eigen::Vector4d PostureEOM::getqua() {
  Eigen::Vector4d ret;
  ret << state(0), state(1), state(2), state(3);
  return ret;
}
Eigen::Vector3d PostureEOM::getome(){
  return state.block(0,4,1,3);
}
Eigen::VectorXd PostureEOM::getstate(){
  return state;
}
Eigen::MatrixXd PostureEOM::getPmat(){
  return P;
}
Eigen::MatrixXd PostureEOM::getQmat(){
  return Q;
}
Eigen::MatrixXd PostureEOM::getRmat(){
  return R;
}
Eigen::MatrixXd PostureEOM::getDCM(){
  return QtoDCM(state);
}


int csvw(FILE *ffile, char *fname, Eigen::VectorXd *factor){
  if((ffile = fopen(fname, "w")) == NULL){
    std::cout << "can't open file:::" << fname << '\n';
  }
  else{
    for(int j=0; j<times; j++){
      for(int i=0; i<6; i++){
        fprintf(ffile, "%f,", factor[j][i]);
      }
      fprintf(ffile, "%f\n", factor[j][6]);
    }
  }
  fclose(ffile);
  return 0;
}
