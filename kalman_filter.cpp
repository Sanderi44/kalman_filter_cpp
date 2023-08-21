#include "kalman_filter.h"
#include <iostream>

using namespace std;

KalmanFilter::KalmanFilter()
{
    cout << "Kalman Filter Constructor" << endl;
}

KalmanFilter::KalmanFilter(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
               Eigen::MatrixXd &G_in, Eigen::VectorXd u_in, Eigen::MatrixXd &Q_in)
{
    cout << "Kalman Filter Constructor" << endl;
    Initialize(x_in, P_in, F_in, G_in, u_in, Q_in);
}

void KalmanFilter::Initialize(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
               Eigen::MatrixXd &G_in, Eigen::VectorXd u_in, Eigen::MatrixXd &Q_in)
{
    cout << "Kalman Filter Initialize" << endl;
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    G_ = G_in;
    u_ = u_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict(const Eigen::VectorXd &x, const Eigen::MatrixXd &Q)
{
    // cout << "Kalman Filter Predict" << endl;
    x_ = F_ * x;                            // State Prediction
    Eigen::MatrixXd Ft = F_.transpose();     // transpose of State Transition Matrix
    P_ = F_ * P_ * Ft + Q;                  // Covariance Prediction
    std::cout << "Predicton Covariance: " << P_ << std::endl;
}

void KalmanFilter::Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H_, const Eigen::MatrixXd &R_)
{
    // cout << "Kalman Filter Update" << endl;
    Eigen::VectorXd y = z - H_ * x_;         // Measurement Residual
    Eigen::MatrixXd Ht = H_.transpose();     // transpose of Observation Matrix
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;   // Prediction Covariance + Measurement Covariance
    Eigen::MatrixXd Si = S.inverse();        // Inverse of S
    Eigen::MatrixXd PHt = P_ * Ht;           // Covariance Prediction * Observation Matrix
    Eigen::MatrixXd K = PHt * Si;            // Kalman Gain
    

    // new state
    x_ = x_ + (K * y);                       // Update State
    long x_size = x_.size();                    
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    // P_ = (I - K * H_) * P_ * (I - K * H_).inverse() + (K * R_ * K.inverse());                  // Update Covariance
    P_ = (I - K * H_) * P_;                  // Update Covariance
}

void KalmanFilter::Control(Eigen::VectorXd &u_in)
{
    u_ = u_in;
}