#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <iostream>
#include <ekf.h>

using namespace std;

EKF::EKF(Eigen::Matrix2d sensor_uncertainty, double dt) {
    Q = sensor_uncertainty;
    // initialize the pose to the origin of local coordinates
    x_t = Eigen::Matrix<double, dim, 1>::Zero();
    x_t_pred = x_t;
    x_gt = Eigen::Matrix<double, dim, 1>::Zero();

    // initialize input to zero
    u_t = Eigen::Matrix<double, 1, 2>::Zero();

    // initialize the covariance matrix to:
    // - Zero in the pose
    // - Infinite in the landmarks
    sigma_t = Eigen::Matrix<double, dim, dim>::Zero();
    sigma_t.block(3, 3, 2*N, 2*N) = Eigen::Matrix<double, dim, dim>::Constant(INT_MAX); // landmark init

    G_t = Eigen::Matrix<double, dim, dim>::Identity();

    delta_t = dt;

    map = {};
    map_gt = {};
}

void EKF::predictState() {
    Eigen::Vector3d update;

    if (u_t[1] != 0) {
        update << -u_t[0] / u_t[1] * sin(x_t[2]) + u_t[0] / u_t[1] * sin(x_t[2] + u_t[1] * delta_t),
                   u_t[0] / u_t[1] * cos(x_t[2]) - u_t[0] / u_t[1] * cos(x_t[2] + u_t[1] * delta_t),
                   u_t[1] * delta_t;
    } else {
        update << u_t[0] * cos(x_t[2]),
                  u_t[0] * sin(x_t[2]),
                  u_t[1] * delta_t;
    }
    
    x_t_pred.block(0, 0, 3, 1) += update;
}


void EKF::predictObservation() {
    Eigen::Matrix<double, 3, 3> G_t_x;

    if (u_t[1] != 0) {
        G_t_x << 1, 0, -u_t[0] / u_t[1] * cos(x_t_pred[2]) + u_t[0] / u_t[1] * cos(x_t_pred[2] + u_t[1] * delta_t),
                 0, 1, -u_t[0] / u_t[1] * sin(x_t_pred[2]) + u_t[0] / u_t[1] * sin(x_t_pred[2] + u_t[1] * delta_t),
                 0, 0, 1;
    } else {
        G_t_x << 1, 0, -u_t[0] * sin(x_t_pred[2]),
                 0, 1,  u_t[0] * cos(x_t_pred[2]),
                 0, 0,  1;
    }
    
    G_t.block(0, 0, 3, 3) = G_t_x; 

    R_t << 0.02 , 0  ,    0,
           0   , 0.02,    0,
           0   , 0  , 0.002;

    sigma_t_pred = G_t * sigma_t * G_t.transpose();
    sigma_t_pred.block(0, 0, 3, 3) += R_t;
}

void EKF::correctionStep() {

    for (auto obs : z_t) {
        int id = obs.first;
        Eigen::Vector2d observation = obs.second;

        // observation has the form of range sensor measurement: zt(r, phi)
        double r = observation[0];
        double phi = observation[1];

        if (map.count(id) == 0) {
            Eigen::Matrix<double, 2, 1> relative_measurement;
            relative_measurement << r * cos(phi + x_t_pred[2]),
                                    r * sin(phi + x_t_pred[2]);

            map[id] = x_t_pred.block(0, 0, 2, 1) + relative_measurement;

            // update landmark location estimate
            int landmark_idx = 3 + id*2;
            x_t_pred[landmark_idx] = x_t_pred[0] + relative_measurement[0];
            x_t_pred[landmark_idx + 1] = x_t_pred[1] + relative_measurement[1];
        }

        // delta = landmark pose - robot pose (same as relative measurement)
        Eigen::Matrix<double, 2, 1> delta;
        delta << map[id][0] - x_t_pred[0],
                 map[id][1] - x_t_pred[1];

        double q = delta.transpose() * delta;
        double q_sqrt = pow(q, 0.5);

        Eigen::Matrix<double, 2, 1> z_pred;
        z_pred << q_sqrt, 
                  atan2(delta[1], delta[0]) - x_t_pred[2];
        
        Eigen::Matrix<double, 5, dim> F = Eigen::Matrix<double, 5, dim>::Zero();
        F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        F.block(3, 3 + 2*id, 2, 2) = Eigen::Matrix2d::Identity();

        Eigen::Matrix<double, 2, 5> H_low;
        H_low << -q_sqrt * delta[0], -q_sqrt * delta[1],  0 , q_sqrt * delta[0], q_sqrt * delta[1],
                  delta[1]         , -delta[0]         , -q ,         -delta[1],          delta[0];

        H_low *= 1/q; // size: 2 x 5
        H_t = H_low * F; // size: 2 x 3 + 2N

        K_t = sigma_t_pred * H_t.transpose() * (H_t * sigma_t_pred * H_t.transpose() + Q).inverse(); // size K_t: 3 + 2N x 5
        x_t_pred = x_t_pred + K_t * (observation - z_pred); // size: 3 + 2N x 1

        sigma_t_pred = (Eigen::Matrix<double, dim ,dim>::Identity() - K_t * H_t) * sigma_t_pred; //size: 3 + 2N x 3 + 2N

    }

    x_t = x_t_pred;
    sigma_t = sigma_t_pred;
}

void EKF::PredictionStep() {
    predictState();
    predictObservation();
}

void EKF::ComputeStateGT() {
    Eigen::Vector3d update;

    if (u_t[1] != 0) {
        update << -u_t[0] / u_t[1] * sin(x_gt[2]) + u_t[0] / u_t[1] * sin(x_gt[2] + u_t[1] * delta_t),
                   u_t[0] / u_t[1] * cos(x_gt[2]) - u_t[0] / u_t[1] * cos(x_gt[2] + u_t[1] * delta_t),
                   u_t[1] * delta_t;
    } else {
        update << u_t[0] * cos(x_gt[2]),
                  u_t[0] * sin(x_gt[2]),
                  u_t[1] * delta_t;
    }
    
    x_gt.block(0, 0, 3, 1) += update;
}

void EKF::ComputeObsGT() {
    for (auto obs : z_t) {
        int id = obs.first;
        Eigen::Vector2d observation = obs.second;

        if (map_gt.count(id) == 0) {
            // observation has the form of range sensor measurement: zt(r, phi)
            double r = observation[0];
            double phi = observation[1];

            Eigen::Vector2d relative_measurement;
            relative_measurement << r * cos(phi + x_gt[2]),
                                    r * sin(phi + x_gt[2]);

            map_gt[id] = x_gt.block(0, 0, 2, 1) + relative_measurement;

            // update landmark location estimate
            int landmark_idx = 3 + id*2;
            x_gt[landmark_idx] = x_gt[0] + relative_measurement[0];
            x_gt[landmark_idx + 1] = x_gt[1] + relative_measurement[1];
        }
    }
}