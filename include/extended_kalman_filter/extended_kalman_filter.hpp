#include "matplotlibcpp.h"

std::vector<float> x_hat;
std::vector<float> u;

float steering_angle;
float lon_acceleration;
float dt;
float wheelbase;

float process_noise[4][4];
float measurement_noise[2][2];
float measurements[3][2];
float H[2][4];

std::vector<float> motion_model(std::vector<float> x, std::vector<float> u, float dt=0.1, float L=2.5);
void initialize_variables(float& dt, float& steering_angle, float& wheelbase, float& lon_acceleration,
                          float& process_noise, float& measurement_noise, float& H);