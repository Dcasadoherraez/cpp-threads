#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <iostream>
#include <ekf.h>

using namespace std;


EKF::EKF(Eigen::Matrix2d sensor_uncertainty, Eigen::Matrix3d motion_uncertainty, double dt) {
    Q = sensor_uncertainty;
    R_t = motion_uncertainty;
    delta_t = dt;

    // initialize the pose to the origin of local coordinates
    x_t = Eigen::Matrix<double, dim, 1>::Zero();
    x_gt = Eigen::Matrix<double, dim, 1>::Zero();
    x_t_pred = Eigen::Matrix<double, dim, 1>::Zero();;

    // initialize input to zero
    u_t = Eigen::Vector3d::Zero();

    // initialize the covariance matrix to:
    // - Zero in the pose
    // - Infinite in the landmarks
    sigma_t = Eigen::Matrix<double, dim, dim>::Zero();
    sigma_t.block(3, 3, 2*N, 2*N) = Eigen::Matrix<double, 2*N, 2*N>::Ones() * INT_MAX; // landmark init
    
    G_t = Eigen::Matrix<double, dim, dim>::Identity();

    map_t = {};
    map_gt = {};
}

double EKF::ConstrainAngle(double angle){
  while (angle > M_PI) {
    angle = angle - 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle = angle + 2 * M_PI;
  }
  return angle;
}

void EKF::PredictState() {
    Eigen::Vector3d update;

    update <<   u_t(0) * cos(ConstrainAngle(x_t(2) + u_t(1) * delta_t)),
                u_t(0) * sin(ConstrainAngle(x_t(2) + u_t(1) * delta_t)),
                (u_t(1) + u_t(2)) * delta_t;

    x_t_pred.block(0, 0, 3, 1) += update;
}


void EKF::PredictCovariance() {
    Eigen::Matrix3d G_t_x;

    G_t_x << 1, 0, -u_t(0) * sin(ConstrainAngle(x_t(2) + u_t(1) * delta_t)),
             0, 1,  u_t(0) * cos(ConstrainAngle(x_t(2) + u_t(1) * delta_t)),
             0, 0,  1;

    // top left block
    sigma_t_pred.block(0, 0, 3, 3) = G_t_x * sigma_t.block(0, 0, 3, 3) * G_t_x.transpose() + R_t;

    // top right block
    sigma_t_pred.block(0, 3, 3, 2 * N) = G_t_x * sigma_t.block(0, 3, 3, 2 * N);

    // bottom left block
    sigma_t_pred.block(3, 0, 2 * N, 3) = (G_t_x * sigma_t.block(0, 3, 3, 2 * N)).transpose();

    // bottom right block
    sigma_t_pred.block(3, 3, 2 * N, 2 * N) = sigma_t.block(3, 3, 2 * N, 2 * N);

}
    int ct = 0;

void EKF::correctionStep() {

    for (auto obs : z_t) {
        int id = obs.first;
        Eigen::Vector2d observation = obs.second;
        // observation has the form of range sensor measurement: zt(r, phi)
        double r = observation(0);
        double phi = observation(1);

        int landmark_idx = 3 + id*2;

        if (!map_t.count(id)) {
            Eigen::Vector2d relative_measurement;
            relative_measurement << r * cos(phi + x_t_pred(2)),
                                    r * sin(phi + x_t_pred(2));

            map_t[id] = x_t_pred.block(0, 0, 2, 1) + relative_measurement;

            // update landmark location estimate
            x_t_pred(landmark_idx) = x_t_pred(0) + relative_measurement(0);
            x_t_pred(landmark_idx + 1) = x_t_pred(1) + relative_measurement(1);
        }

        cout << "Predicted state: " << x_t_pred.block(0, 0, 2, 1).transpose() << endl;
        cout << "Predicted landmark " << landmark_idx << ": " << x_t_pred(landmark_idx) << "," << x_t_pred(landmark_idx + 1) << endl;
        cout << "Measurement: " << observation.transpose() << endl;

        // delta = landmark pose - robot pose (same as relative measurement)
        Eigen::Vector2d delta;
        delta << x_t_pred(landmark_idx) - x_t_pred(0),
                 x_t_pred(landmark_idx+ 1) - x_t_pred(1);

        double q = delta.transpose() * delta;
        double q_sqrt = pow(q, 0.5);

        if (q > 1e25) return;
        Eigen::Vector2d z_pred;
        z_pred << q_sqrt, 
                  atan2(delta(1), delta(0)) - x_t_pred(2);

        Eigen::Matrix<double, 5, dim> F = Eigen::Matrix<double, 5, dim>::Zero();
        F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        F.block(3, 3 + 2*id, 2, 2) = Eigen::Matrix2d::Identity();

        Eigen::Matrix<double, 2, 5> H_low;
        H_low << -q_sqrt * delta(0), -q_sqrt * delta(1),  0 , q_sqrt * delta(0), q_sqrt * delta(1),
                  delta(1)         , -delta(0)         , -q ,         -delta(1),          delta(0);
        H_low *= 1/q; // size: 2 x 5
        H_t = H_low * F; // size: 2 x 3 + 2N

        cout << q << endl;
        K_t = (sigma_t_pred * H_t.transpose()) * (H_t * sigma_t_pred * H_t.transpose() + Q).inverse(); // size K_t: 3 + 2N x 5
        x_t_pred = x_t_pred + K_t * (observation - z_pred); // size: 3 + 2N x 1

        sigma_t_pred = (Eigen::Matrix<double, dim ,dim>::Identity() - K_t * H_t) * sigma_t_pred; //size: 3 + 2N x 3 + 2N

    }

    x_t = x_t_pred;
    sigma_t = sigma_t_pred;
}


void EKF::PredictionStep() {
    PredictState();
    PredictCovariance();
}
