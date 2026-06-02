#include <cstring>
#include <eigen3/Eigen/Dense>
#include "extended_kalman_filter/extended_kalman_filter.hpp"

void initialize_variables()
{
    dt = 0.1;
    steering_angle = 0.0;
    wheelbase = 2.5;
    lon_acceleration = 0.0;

    // process noise covariance matrix Q
    Q(0,0)=0.01f;
    Q(1,1)=0.01f;
    Q(2,2)=0.001f;
    Q(3,3)=0.1f;
    
    // state covariance matrix P
    P(0,0)=10.0f;
    P(1,1)=10.0f;
    P(2,2)=10.1f;
    P(3,3)=10.5f;
    
    // measurement noise covariance matrix R
    R(0,0)=0.5f;
    R(1,1)=0.5f;

    // measurement Jacobian H
    H << 1, 0, 0, 0,
         0, 1, 0, 0;


    float meas[3][2] = {
        {0.21, 0.02},
        {0.40, 0.08},
        {0.58, 0.19},
    };
    std::memcpy(measurements, meas, sizeof(meas));

    x_hat = {0.0, 0.0, 0.0, 2.0};
    u = {0.2, 0.0};
}

Eigen::Matrix<float, 4, 1> motion_model(std::vector<float> x_hat, std::vector<float> u, float dt, float wheelbase)
{
    float pos_x = x_hat[0];
    float pos_y = x_hat[1];
    float heading = x_hat[2];
    float velocity = x_hat[3];

    float steering_angle = u[0];
    float acceleration = u[1];

    Eigen::Matrix<float, 4, 1> x_pred;

    x_pred(0) = pos_x + velocity * std::cos(heading) * dt;
    x_pred(1) = pos_y + velocity * std::sin(heading) * dt;
    x_pred(2) = heading + (velocity/wheelbase) * std::tan(steering_angle) * dt;
    x_pred(3) = velocity  + acceleration * dt;
    
    return x_pred;
}

Eigen::Matrix4f compute_F(std::vector<float> x, std::vector<float> u, float dt, float wheelbase)
{
    float heading = x[2];
    float velocity = x[3];

    float steering_angle = u[0];

    Eigen::Matrix4f df; 
    df << 1,    0, -velocity * std::sin(heading) * dt, std::cos(heading) * dt,
          0,    1,  velocity * std::cos(heading) * dt, std::sin(heading) * dt,
          0,    0,  1,                            std::tan(steering_angle) * dt/wheelbase,
          0,    0,  0,                            1;

    return df;
}

int main(){
    // --------------- START INITIALIZATION ---------------
    initialize_variables();

    for (int i=0; i < 3; i++)
    {
        // ---- predict ----
        Eigen::Matrix<float, 4, 1> x_check = motion_model(x_hat, u, dt, wheelbase);
        Eigen::Matrix4f F = compute_F(x_hat, u, dt, wheelbase);
        Eigen::Matrix4f P_pred = F * P * F.transpose() + Q;

        // ---- update ----
        Eigen::Vector2f z(measurements[i][0], measurements[i][1]);
        Eigen::Vector2f z_pred(x_check[0], x_check[1]);
        Eigen::Vector2f y = z - z_pred;

        // kalman gain
        Eigen::Matrix2f S             = H * P_pred * H.transpose() + R;
        Eigen::Matrix<float, 4, 2> K  = P_pred * H.transpose() * S.inverse();

        Eigen::Vector4f x_updated = x_check + K * y;
        Eigen::Matrix4f P_updated = (Eigen::Matrix4f::Identity() - K * H) * P_pred;

        // ---- feed back ----
        x_hat = {x_updated[0], x_updated[1], x_updated[2], x_updated[3]};
        P     = P_updated;

        std::cout << "P_pred:\n" << P_pred << "\n";
        std::cout << "pos_x: " << x_hat[0] << "\n";
        std::cout << "pos_y: " << x_hat[1] << "\n";
        std::cout << "heading: " << x_hat[2] << "\n";
        std::cout << "velocity: " << x_hat[3] << "\n";
        std::cout << "P_updated:\n" << P_updated << "\n";
        std::cout << "\n";
    }
    // --------------- END INITIALIZATION ---------------
}
