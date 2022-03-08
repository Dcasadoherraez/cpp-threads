#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <iostream>
#include <ekf.h>

using namespace std;


EKF::EKF(int s, double s_noise, double m_noise, double dt) {
    sensor_noise = s_noise;
    motion_noise = m_noise;
    delta_t = dt;
    scale = s;

    // initialize the pose to the origin of local coordinates
    x_t = Eigen::Matrix<double, dim, 1>::Zero();
    x_t_pred = Eigen::Matrix<double, dim, 1>::Zero();;

    // initialize the covariance matrix to:
    // - Zero in the pose
    // - Infinite in the landmarks
    sigma_t = Eigen::Matrix<double, dim, dim>::Zero();
    sigma_t.block(3, 3, 2*N, 2*N) = Eigen::Matrix<double, 2*N, 2*N>::Ones() * INT_MAX; // landmark init
    
    map_t = {};
    map_gt = {};
}

double EKF::ConstrainAngle(double angle){
  while (angle > M_PI) {
    angle = angle - 2 * M_PI;
  }
  while (angle < - M_PI) {
    angle = angle + 2 * M_PI;
  }
  return angle;
}


void EKF::PredictState() {
    Eigen::Vector3d update;

    update <<   u_t(0) * cos(x_t(2) + u_t(1) * delta_t),
                u_t(0) * sin(x_t(2) + u_t(1) * delta_t),
                ConstrainAngle(delta_t * (u_t(1) + u_t(2)));

    x_t_pred.block(0, 0, 3, 1) += update;
    x_t_pred(2) = ConstrainAngle(x_t_pred(2));

}


void EKF::PredictCovariance() {
    Eigen::Matrix3d G_t_x;

    G_t_x << 1, 0, -u_t(0) * sin(x_t(2) + u_t(1) * delta_t),
             0, 1,  u_t(0) * cos(x_t(2) + u_t(1) * delta_t),
             0, 0,  1;

    Eigen::Matrix3d R_t; 
    R_t << motion_noise / (scale),          0  ,               0,
                      0, motion_noise / (scale),               0,
                      0,          0  , motion_noise/(100 * scale * scale);

    // top left block
    sigma_t_pred.block(0, 0, 3, 3) = G_t_x * sigma_t.block(0, 0, 3, 3) * G_t_x.transpose() + R_t;

    // top right block
    sigma_t_pred.block(0, 3, 3, 2 * N) = G_t_x * sigma_t.block(0, 3, 3, 2 * N);

    // bottom left block
    sigma_t_pred.block(3, 0, 2 * N, 3) = (G_t_x * sigma_t.block(0, 3, 3, 2 * N)).transpose();

    // bottom right block
    sigma_t_pred.block(3, 3, 2 * N, 2 * N) = sigma_t.block(3, 3, 2 * N, 2 * N);

}

void EKF::CorrectionStep() {

    int m = 2 * z_t.size();
    Eigen::MatrixXd H_t = Eigen::MatrixXd::Zero(m,dim);
    Eigen::VectorXd Z_diff = Eigen::VectorXd::Zero(m);

    int ct = 0;
    
    for (auto obs : z_t) {
        int id = obs.first;
        Eigen::Vector2d observation = obs.second;
        // observation has the form of range sensor measurement: zt(r, phi)
        double r = observation(0);
        double phi = observation(1);

        int landmark_idx = 3 + id*2;

        if (!map_t.count(id)) {

            // first landmark location estimate
            x_t_pred(landmark_idx) = x_t_pred(0) + r * cos(phi + x_t_pred(2));
            x_t_pred(landmark_idx + 1) = x_t_pred(1) + r * sin(phi + x_t_pred(2));

            map_t[id] = x_t_pred.block(landmark_idx, 0, 2, 1);
        }

        // cout << "Predicted state: " << x_t_pred.block(0, 0, 2, 1).transpose() << endl;
        // cout << "Predicted landmark " << landmark_idx << ": " << x_t_pred(landmark_idx) << "," << x_t_pred(landmark_idx + 1) << endl;
        // cout << "Measurement: " << observation.transpose() << endl;

        // delta = landmark pose - robot pose (same as relative measurement)
        Eigen::Vector2d delta = x_t_pred.block(landmark_idx, 0, 2, 1) - x_t_pred.block(0, 0, 2, 1);

        double q = delta.transpose() * delta;
        double q_sqrt = pow(q, 0.5);

        Eigen::Vector2d z_pred;
        z_pred << q_sqrt, 
                  ConstrainAngle(atan2(delta(1), delta(0)) - x_t_pred(2));

        Z_diff(ct * 2) = r - z_pred(0);
        Z_diff(ct * 2 + 1) = ConstrainAngle(phi - z_pred(1));

        // Eigen::Matrix<double, 5, dim> F = Eigen::Matrix<double, 5, dim>::Zero();
        // F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        // F.block(3, 3 + 2*id, 2, 2) = Eigen::Matrix2d::Identity();

        Eigen::Matrix<double, 2, dim> H_low = Eigen::Matrix<double, 2, dim>::Zero();

        H_low.block(0, 0, 2, 3) <<  - q_sqrt * delta(0), - q_sqrt * delta(1),  0,
                                               delta(1),           - delta(0), -q;

        H_low.block(0, landmark_idx, 2, 2) << q_sqrt * delta(0), q_sqrt * delta(1),
                                                      -delta(1),          delta(0);

        H_t.block(ct * 2, 0, 2, dim) = H_low * 1/q;
        // cout << "q: " << q << endl;
        ct++;

    }
    // cout << H_t << endl;
    Eigen::MatrixXd Q_t = Eigen::MatrixXd::Identity(m, m) * sensor_noise ;

    Eigen::MatrixXd K_t(dim, m);
    // TODO: Test for non-inversible matrices
    K_t = sigma_t_pred * H_t.transpose() * (H_t * sigma_t_pred * H_t.transpose() + Q_t).inverse(); // size K_t: 3 + 2N x 5
    x_t_pred += K_t * Z_diff; // size: 3 + 2N x 1
    x_t_pred(2) = ConstrainAngle(x_t_pred(2));

    sigma_t_pred = (Eigen::Matrix<double, dim ,dim>::Identity() - K_t * H_t) * sigma_t_pred; //size: 3 + 2N x 3 + 2N
    // cout << "K_t: " << K_t << endl;
    // cout << "Z_diff: " << Z_diff << endl;
    x_t = x_t_pred;
    sigma_t = sigma_t_pred;
}


void EKF::PredictionStep() {
    PredictState();
    PredictCovariance();
}
