#ifndef TOOLS_H_
#define TOOLS_H_
#define _USE_MATH_DEFINES

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

    /**
     * Convert cartesian Vector cartV to polar Vecor
     */
    Eigen::VectorXd ConvertCartesianToPolar(const Eigen::VectorXd &cartV);
    
    /**
     * Convert polar Vector polarV to cartesian Vecor
     */
    Eigen::VectorXd ConvertPolarToCartesian(const Eigen::VectorXd &polarV);
    
    double NormalizeAngleRadians(double ang);
};

#endif  // TOOLS_H_
