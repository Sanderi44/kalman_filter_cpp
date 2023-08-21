#include <iostream>
#include <cstdlib>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "kalman_filter.h"
#include "car.h"
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

int main(int, char**){
    srand(time(NULL)); // randomize seed
    std::cout << "Hello, from kalman_filter!\n";
    float steering_angle = 0.0;
    float dt = 0.1;

    Eigen::VectorXd position = Eigen::VectorXd(2);
    position << 1, 0;
    Eigen::VectorXd velocity = Eigen::VectorXd(2);
    velocity << cos(M_PI / 4.0), sin(M_PI / 4.0);
    Eigen::VectorXd acceleration = Eigen::VectorXd(2);
    acceleration << 0, 0;
    
    Car car(position, velocity, acceleration, dt, steering_angle);

    Eigen::VectorXd x_in(4);
    x_in << 1, 0, cos(M_PI / 4.0), sin(M_PI / 4.0);
    Eigen::MatrixXd P_in(4, 4);
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    Eigen::MatrixXd F_in(4, 4);
    F_in << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Eigen::MatrixXd G_in(4, 2);
    G_in << 0.5 * dt * dt, 0,
            0, 0.5 * dt * dt,
            dt, 0,
            0, dt;
    Eigen::VectorXd u_in(2);
    u_in << 0, 0;

    Eigen::MatrixXd Q_in(4, 4);
    Q_in << 0.5 * dt * dt, 0, 0, 0,
            0, 0.5 * dt * dt, 0, 0,
            0, 0, 0.5 * dt * dt, 0,
            0, 0, 0, 0.5 * dt * dt;
    // Q_in << 1.0, 0, 0, 0,
    //         0, 1.0, 0, 0,
    //         0, 0, 100.0, 0,
    //         0, 0, 0, 100.0;

    KalmanFilter kf(x_in, P_in, F_in, G_in, u_in, Q_in);
    Eigen::VectorXd acceleration_in(2);
    acceleration_in << 0, 0;
    car.Accelerate(0.0);

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> vx;
    std::vector<double> vy;
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;
    Eigen::VectorXd kf_state;
    std::vector<double> kf_x;
    std::vector<double> kf_y;
    std::vector<double> kf_vx;
    std::vector<double> kf_vy;
    float a = 1.0;
    float time = 0.0;
    float acc = 1.0;
    Eigen::VectorXd acc_cur;
    std::vector<double> acc_vec_x;
    std::vector<double> acc_vec_y;
    std::vector<double> time_vec;

    // car.Accelerate(acceleration_in);
    // steering_angle = 0.2;
    std::vector<double> steering_angles;
    Eigen::VectorXd measured_state(4);
    float noise = 1.0;
    float noise_x = -(noise / 2) + noise * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float noise_y = -(noise / 2) + noise * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    for (size_t i = 0; i < 100; i++)
    {
        // Actual Car
        time += dt;
        time_vec.push_back(time);
        float step = static_cast<float>(i);
        steering_angle = sin(step / 10.0);
        car.Accelerate(acc);
        car.Steer(steering_angle);
        steering_angles.push_back(steering_angle);
        car.Move();
        pos = car.GetPosition();
        x.push_back(pos[0]);
        y.push_back(pos[1]);
        vel = car.GetVelocity();
        vx.push_back(vel[0]);
        vy.push_back(vel[1]);
        cout << pos << endl;
        acc_cur = car.GetAcceleration();
        acc_vec_x.push_back(acc_cur[0]);
        acc_vec_y.push_back(acc_cur[1]);


        // Kalman Filter
        acceleration_in[0] = acc * cos(steering_angle);
        acceleration_in[1] = acc * sin(steering_angle);
        // kf.Control(acceleration_in);
        kf_state = kf.State();


        noise_x = -(noise / 2) + noise * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        noise_y = -(noise / 2) + noise * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);


        measured_state << kf_state[0],
                          kf_state[1],
                          vel[0] + noise_x,
                          vel[1] + noise_y;

        kf.Predict(measured_state, Q_in);
        kf_x.push_back(kf_state[0]);
        kf_y.push_back(kf_state[1]);
        kf_vx.push_back(kf_state[2]);
        kf_vy.push_back(kf_state[3]);

        // acceleration_in[0] = 0.0;
        // acceleration_in[1] = 0.0;

    }
    plt::plot(x, y, "r-");
    plt::plot(kf_x, kf_y, "b-");
    // plt::plot(time_vec, vx, "c-");
    // plt::plot(time_vec, vy, "y-");
    // plt::plot(time_vec, kf_vx, "r-");
    // plt::plot(time_vec, kf_vy, "g-");
    // plt::plot(time_vec, acc_vec_x, "g-");
    // plt::plot(time_vec, acc_vec_y, "b-");
    // plt::plot(kf_x, kf_y, "b-");
    // plt::plot(steering_angles, "g-");
    plt::show();

    return 0;
}
