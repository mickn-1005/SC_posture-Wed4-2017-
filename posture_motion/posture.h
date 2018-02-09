#ifndef  _POSTURE
#define  _POSTURE

#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace Eigen;

/*-----------------
define constants...
*note*
No Gravity Gradient and other Disturbent torque
No gravity effection
No energy dissipation
-----------------*/
const float Ix = 1.9;   // [kgm^2]
const float Iy = 1.6;
const float Iz = 2.0;
const float omega_s = 17.0; //[rpm] nominal angular verocity torque (around y axis)
const float delt = 0.01;   // delta time
const int times = 10000;   // time steps

float TCx = 0.0;    // controll torque
float TCy = 0.0;
float TCz = 0.0;
float TNx = 0.0;    // noisy torque
float TNy = 0.0;
float TNz = 0.0;

Eigen::Vector4d q0 = {1.0,0.0,0.0,0.0};  //initial quaternion  (t = 0)
Eigen::Vector3d omega0 = {0.1, omega_s+0.1, 0};

/*---------------
define classes...
---------------*/

class PostureEOM{
  private:
    Eigen::Vector4d quata;
    Eigen::Vector3d omega;
    Eigen::VectorXd state;
    Eigen::VectorXd delstate;
  public:
    void state_initialize();
    int setstate(Eigen::Vector4d *qua, Eigen::Vector3d *ome);  // define quaternion
    Eigen::VectorXd calcdelstate();
    Eigen::VectorXd nextstate();    //Runge-Kutta-Gill

    Eigen::Vector4d getqua();  // get quarter in quaternion
    Eigen::Vector3d getome();
    Eigen::VectorXd getstate();
    void showqua();  // if quarter.size() ~= 4 -> segmentation fault!!!!!!!!!
};

class vecs {  //vector type showing
  public:
    void disp (Eigen::Vector4d *vec);
};

#endif  // posture
