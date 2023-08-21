#pragma once
#include <eigen3/Eigen/Dense>

class KalmanFilter
{
public:
	KalmanFilter();
    KalmanFilter(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &G_in, Eigen::VectorXd u_in, Eigen::MatrixXd &Q_in);
	~KalmanFilter() = default;
    void Initialize(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &G_in, Eigen::VectorXd u_in, Eigen::MatrixXd &Q_in);
    void Predict();
    void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H_, const Eigen::MatrixXd &R_);
    void Control(Eigen::VectorXd &u_in);

    Eigen::VectorXd State() { return x_; };
    Eigen::MatrixXd Covariance() { return P_; };

private:
    Eigen::VectorXd x_;  // state vector
    Eigen::MatrixXd P_;  // state covariance matrix
    Eigen::MatrixXd F_;  // state transition matrix
    Eigen::MatrixXd G_;  // Control Matrix
    Eigen::VectorXd u_;  // Control Vector
    Eigen::MatrixXd H_;  // measurement matrix
    Eigen::MatrixXd R_;  // measurement covariance matrix
    Eigen::MatrixXd Q_;  // process covariance matrix
};
