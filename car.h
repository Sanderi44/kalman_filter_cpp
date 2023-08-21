#pragma once
#include <eigen3/Eigen/Dense>

class Car
{
public:
	Car(Eigen::VectorXd &x_in, Eigen::VectorXd &v_in, Eigen::VectorXd &a_in, float dt_in, float steering_angle_in);
	~Car() = default;

    void Steer(float steering_angle);
    void Accelerate(float acceleration);
    void Move();
    Eigen::VectorXd GetPosition() { return x_; };
    Eigen::VectorXd GetVelocity() { return v_; };
    Eigen::VectorXd GetAcceleration() { return a_; };

private:
    Eigen::VectorXd x_; // state vector
    Eigen::VectorXd v_; // velocity vector
    Eigen::VectorXd a_; // acceleration vector
    float acceleration_{0.0}; // acceleration
    float dt_{0.1}; // time step
    float steering_angle_{0.0}; // steering angle
};
