#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    size_t esz = estimations.size();
    size_t gsz = ground_truth.size();
    if (esz == 0 || esz != gsz) {
        return rmse;
    }

    for (unsigned int i=0; i < estimations.size(); ++i) {
        VectorXd diffV = estimations[i] - ground_truth[i];
        diffV = diffV.array()*diffV.array();
        
        rmse += diffV;
    }
    
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);
    
    double ro2 = px*px + py*py;
    double ro = sqrt(ro2);
    
    MatrixXd Hj = MatrixXd(3,4);
    Hj << px/ro, py/ro, 0, 0,
                -py/ro2, px/ro2, 0, 0,
                py*(vx*py-vy*px)/(ro*ro2), px*(vy*px-vx*py)/(ro*ro2), px/ro, py/ro;
    return Hj;
}

VectorXd Tools::ConvertCartesianToPolar(const Eigen::VectorXd &cartV) {
    VectorXd polarV = VectorXd(3);
    double px = cartV(0);
    double py = cartV(1);
    double vx = cartV(2);
    double vy = cartV(3);

    double sqr = sqrt(px*px + py*py);
    polarV << sqr,
                atan2(py, px),
                (px*vx + py*vy)/sqr;
    
    return polarV;
}

VectorXd Tools::ConvertPolarToCartesian(const Eigen::VectorXd &polarV) {
    VectorXd cartV = VectorXd(4);
    double ro = polarV(0);
    double phi = polarV(1);
    
    double px = ro * cos(phi);
    double py = ro * sin(phi);
    
    // We can't determine cartesian velocity from roDot.  Setting it to 0
    cartV << px, py, 0, 0;
    return cartV;
}

/**
 * Convert angle ang to between Pi and -PI
 **/
double Tools::NormalizeAngleRadians(double ang) {
    double add = (ang < 0 ? -2 * M_PI : 2 * M_PI);
    while (ang < -M_PI || ang > M_PI) {
        ang -= add;
    }
    
    return ang;
}
