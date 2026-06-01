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
    Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
    Q(0,0)=0.01f; Q(1,1)=0.01f; Q(2,2)=0.001f; Q(3,3)=0.1f;
    
    // state covariance matrix P
    Eigen::Matrix4f P = Eigen::Matrix4f::Zero();
    P(0,0)=1.0f; P(1,1)=1.0f; P(2,2)=0.1f; P(3,3)=0.5f;
    
    // measurement noise covariance matrix
    float mn[2][2] = {
        {0.5, 0  },
        {0,   0.5},
    };
    std::memcpy(measurement_noise, mn, sizeof(mn));

    float meas[3][2] = {
        {0.21, 0.02},
        {0.40, 0.08},
        {0.58, 0.19},
    };
    std::memcpy(measurements, meas, sizeof(meas));

    float H_jacobian[2][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
    };
    std::memcpy(H, H_jacobian, sizeof(H_jacobian));

    x_hat = {0.0, 0.0, 0.0, 2.0};
    u = {0.2, 0.0};
}

std::vector<float> motion_model(std::vector<float> x_hat, std::vector<float> u, float dt, float wheelbase)
{
    float pos_x = x_hat[0];
    float pos_y = x_hat[1];
    float heading = x_hat[2];
    float velocity = x_hat[3];

    float steering_angle = u[0];
    float acceleration = u[1];

    std::vector<float> x_pred(4);

    x_pred[0] = pos_x + velocity * std::cos(heading) * dt;
    x_pred[1] = pos_y + velocity * std::sin(heading) * dt;
    x_pred[2] = heading + (velocity/wheelbase) * std::tan(steering_angle) * dt;
    x_pred[3] = velocity  + acceleration * dt;
    
    return x_pred;
}

Eigen::Matrix4f compute_F(std::vector<float> x, std::vector<float> u, float dt, float wheelbase)
{
    float heading = x[2];
    float velocity = x[3];

    float steering_angle = u[0];
    float acceleration = u[1];

    Eigen::Matrix4f df; 
    df << 1,    0, -velocity * sin(heading) * dt, std::cos(heading) * dt,
          0,    1,  velocity * cos(heading) * dt, std::sin(heading) * dt,
          0,    0,  1,                            std::tan(steering_angle) * dt/wheelbase,
          0,    0,  0,                            1;

    return df;
}

int main(){
    // --------------- START INITIALIZATION ---------------
    initialize_variables();

    for (int i=0; i < 10; i++)
    {
        // ---- predict ----
        std::vector<float> x_check = motion_model(x_hat, u, dt, wheelbase);
        Eigen::Matrix4f F = compute_F(x_hat, u, dt, wheelbase);
        Eigen::Matrix4f P_pred = F * P * F.transpose() + Q;

        // ---- update ----
        Eigen::Vector2f z(measurements[i][0], measurements[i][1]);
        Eigen::Vector2f z_pred(x_check[0], x_check[1]);
        Eigen::Vector2f y = z - z_pred;

        Eigen::Matrix2f S             = H * P_pred * H.transpose() + R;
        Eigen::Matrix<float, 4, 2> K  = P_pred * H.transpose() * S.inverse();

        Eigen::Vector4f x_eigen(x_check[0], x_check[1], x_check[2], x_check[3]);
        Eigen::Vector4f x_updated = x_eigen + K * y;
        Eigen::Matrix4f P_updated = (Eigen::Matrix4f::Identity() - K * H) * P_pred;

        // ---- feed back ----
        x_hat = {x_updated[0], x_updated[1], x_updated[2], x_updated[3]};
        P     = P_updated;

        std::cout << "pos_x: " << x_hat[0] << "\n";
        std::cout << "pos_y: " << x_hat[1] << "\n";
        std::cout << "\n";
    }
    // --------------- END INITIALIZATION ---------------
}
