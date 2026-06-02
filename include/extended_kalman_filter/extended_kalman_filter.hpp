#include "matplotlibcpp.h"

std::vector<float> x_hat;
std::vector<float> u;

float steering_angle;
float lon_acceleration;
float dt;
float wheelbase;

Eigen::Matrix4f P = Eigen::Matrix4f::Zero();
Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
Eigen::Matrix<float, 2, 4> H;
Eigen::Matrix2f R = Eigen::Matrix2f::Zero();

float measurements[3][2];

Eigen::Matrix<float, 4, 1> motion_model(std::vector<float> x, std::vector<float> u, float dt=0.1, float L=2.5);
void initialize_variables();