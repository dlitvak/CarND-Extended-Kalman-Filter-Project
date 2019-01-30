#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
        I_ = Eigen::MatrixXd::Identity(4,4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_r, MatrixXd &R_l, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_laser_ = H_in;
  Hj_radar_ = Hj_in;
  R_laser_ = R_l;
    R_radar_ = R_r;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
    
    x_ = F_*x_;
    P_ = F_*P_*F_.transpose() + Q_;
}

/**
 * Measurement update for Lidar
 */
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    VectorXd z_pred = H_laser_ * x_;
    VectorXd y = z - z_pred;
    Update_x_P(y, R_laser_, H_laser_);
}

/**
 * Measurement update for Radar
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    // Skip update to avoid division by zero
    if (x_[0] == 0. && x_[1] == 0)
        return;
    
    VectorXd polarX = tools.ConvertCartesianToPolar(x_);
    VectorXd y = z - polarX;

    Hj_radar_ = tools.CalculateJacobian(x_);
    y(1) = tools.NormalizeAngleRadians(y(1));

    Update_x_P(y, R_radar_, Hj_radar_);
}

void KalmanFilter::Update_x_P(const VectorXd &y, const MatrixXd &R_, const MatrixXd &H_) {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    P_ = (I_ - K * H_) * P_;
}

