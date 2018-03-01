#ifndef  _POSTURE
#define  _POSTURE

#include <random>
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
const float omega_s = 17.0*2*3.1415926535/60.0; //[rpm] nominal angular verocity torque (around y axis)
const float delt = 0.01;   // delta time
const int times = 10000;   // time steps

const int wnoisy = 1;    // 1:random noise to Torque 0: No noise
const int vnoisy = 1;
const int ranval = 1;   //
const int isdebug = 1;
const int Kalman = 1;

const double mean_noise = 0.0;
const double conv_noise = 0.01;

Eigen::Vector4d q0 = {1.0,0.0,0.0,0.0};  //initial quaternion  (t = 0)
Eigen::Vector3d omega0 = {0.1, omega_s+0.1, 0};

std::random_device seed_gen;
std::normal_distribution<> dist(mean_noise, conv_noise);    //create white noise
// std::uniform_real_distribution<double> initial(-1.0, 1.0);
std::normal_distribution<> initial(mean_noise, conv_noise);    //create white noise
std::uniform_int_distribution<int> DCMind(0, 2);

std::mt19937 vnoise(seed_gen());
std::mt19937 wnoise(seed_gen());
std::mt19937 indgen(seed_gen());
std::mt19937 iniest(1);    //再現性のためメルセンヌツイスター法の初期シードは固定

/*---------------
define classes...
---------------*/

class PostureEOM{
  private:
    Eigen::VectorXd state;
    Eigen::VectorXd delstate;

    float TCx;    // controll torque
    float TCy;
    float TCz;
    double TNx;    // noisy torque
    double TNy;
    double TNz;

    int isKalman;
    int noisy;
    int obsnoise;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    Eigen::VectorXd calcdelstate();

    Eigen::MatrixXd EOSMatA();
    Eigen::MatrixXd EOSMatB();
    Eigen::MatrixXd EOOMatH(int index);
    int Qnormal();    // Normalize quaternion
  public:
    void state_initialize(int kaltf, int noisetf, int obs_noitf);
    int setstate(Eigen::Vector4d *qua, Eigen::Vector3d *ome);  // define quaternion
    int torque();

    Eigen::VectorXd nextstate();    //Runge-Kutta-Gill

    Eigen::MatrixXd QtoDCM(Eigen::VectorXd Q);
    Eigen::VectorXd Kalman_observe(Eigen::Vector3d DCMvec);

    Eigen::Vector4d getqua();  // get quarter in quaternion
    Eigen::Vector3d getome();
    Eigen::VectorXd getstate();
    Eigen::MatrixXd getPmat();
    Eigen::MatrixXd getQmat();
    Eigen::MatrixXd getRmat();
    Eigen::MatrixXd getDCM();
};

#endif  // posture
