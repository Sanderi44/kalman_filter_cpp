#include "car.h"
#include <iostream>

using namespace std;

Car::Car(Eigen::VectorXd &x_in, Eigen::VectorXd &v_in, Eigen::VectorXd &a_in, float dt_in, float steering_angle_in)
    : x_(x_in), v_(v_in), a_(a_in), dt_(dt_in), steering_angle_(steering_angle_in)
{
    cout << "Car Constructor" << endl;
}

void Car::Steer(float steering_angle)
{
    cout << endl << "Car Steer: " << steering_angle << endl;
    steering_angle_ = steering_angle;
    float total_velocity = sqrt(v_[0] * v_[0] + v_[1] * v_[1]);
    v_[0] = total_velocity * cos(steering_angle_);
    v_[1] = total_velocity * sin(steering_angle_);
    a_[0] = acceleration_ * cos(steering_angle_);
    a_[1] = acceleration_ * sin(steering_angle_);
    cout << "Velocity: " << v_ << endl;
}

void Car::Accelerate(float acceleration)
{
    cout << endl << "Car Accelerate: " << acceleration << endl;
    acceleration_ = acceleration;
    a_[0] = acceleration_ * cos(steering_angle_);
    a_[1] = acceleration_ * sin(steering_angle_);
    cout << "Acceleration: " << a_ << endl;
}

void Car::Move()
{
    cout << "Car Move" << endl;
    x_ = x_ + v_ * dt_ + 0.5 * a_ * dt_ * dt_;
    v_ = v_ + a_ * dt_;
    cout << "Position: " << x_ << endl;
    cout << "Velocity: " << v_ << endl;
}
