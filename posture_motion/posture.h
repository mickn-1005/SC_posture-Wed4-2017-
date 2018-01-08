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
const std::vector<float> q0 = {1.0,0,0,0};  //initial quaternion  (t = 0)
const float Ix = 1.9;   // [kgm^2]
const float Iy = 1.6;
const float Iz = 2.0;
const float omega_s = 17; //[rpm] nominal angular verocity torque (around y axis)



/*---------------
define classes...
---------------*/

class quaternion{
  private:
    std::vector<float>  *quarter;
    Eigen::Matrix4d *odemat;
  public:
    void setquat(std::vector<float> *vec);  // define quaternion
    std::vector<float> getquat();  // get quarter in quaternion
    std::vector<float> eomquat(std::vector<float> *ome);  //  calc eom of posture
    void showquat();  // if quarter.size() ~= 4 -> segmentation fault!!!!!!!!!
};

class vecs {  //vector type showing
  public:
    void disp (std::vector<int> *vec);
    void disp (std::vector<float> *vec);
};

#endif  // posture
