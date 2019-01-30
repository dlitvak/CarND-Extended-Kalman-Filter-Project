#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

    // measurement matrix
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    ekf_.H_laser_ = H_laser_;
    
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
    ekf_.R_laser_ = R_laser_;
    
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
    ekf_.R_radar_ = R_radar_;
    
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
    P_ = MatrixXd(4, 4);
   P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
    ekf_.P_ = P_;

    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
    ekf_.F_ = F_;
    
    ekf_.Q_ = MatrixXd(4,4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF init: " << endl;
    ekf_.x_ = VectorXd(4);
      
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
        VectorXd cartV = this->tools.ConvertPolarToCartesian(measurement_pack.raw_measurements_);
        ekf_.x_ = cartV;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
        ekf_.x_ << measurement_pack.raw_measurements_[0],
        measurement_pack.raw_measurements_[1],
        0,
        0;
    }

      // init previous_timestamp_
      this->previous_timestamp_ = measurement_pack.timestamp_;
      
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   double dt = (measurement_pack.timestamp_ - this->previous_timestamp_) / 1000000.0;
    this->previous_timestamp_ = measurement_pack.timestamp_;
    
    ekf_.F_(0,2) = ekf_.F_(1,3) = dt;
    
    double dt4 = pow(dt, 4)/4.0;
    double dt3 = pow(dt, 3)/2.0;
    double dt2 = pow(dt, 2);
    ekf_.Q_ << dt4*noise_ax, 0, dt3*noise_ax, 0,
    0, dt4*noise_ay, 0, dt3*noise_ay,
    dt3*noise_ax, 0, dt2*noise_ax, 0,
    0, dt3*noise_ay, 0, dt2*noise_ay;
    
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates      
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
