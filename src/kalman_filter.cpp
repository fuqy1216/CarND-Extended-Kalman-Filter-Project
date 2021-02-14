#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  // measurement update
  VectorXd z_new = H_ * x_;
  VectorXd y = z - z_new;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //new state
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //parse elements
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  //calculate radar coordinate elements
  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);
  double rho_dot = (rho != 0 ? (px * vx + py * vy) / rho : 0);

  //define new state
  VectorXd z_new(3);
  z_new << rho, phi, rho_dot;

  //measurement update
  VectorXd y = z - z_new;

  //normalize angle
  double circle_360 = 2 * M_PI;   //
  double offsetValue = y(1) + M_PI;   // value relative to 0
  y(1) = (offsetValue - (floor(offsetValue / circle_360) * circle_360)) - M_PI;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //update new state
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}
