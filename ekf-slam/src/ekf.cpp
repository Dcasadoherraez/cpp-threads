#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ekf.h>

EKF::EKF(Eigen::Matrix<double, 2, 2> sensor_uncertainty, double dt) {
    Q = sensor_uncertainty;
    R = Eigen::Matrix<double, dim, dim>::Zero();

    // initialize the pose to the origin of local coordinates
    x_t = Eigen::Matrix<double, dim, 1>::Zero();

    // initialize input to zero
    u_t = Eigen::Matrix<double, 1, 2>::Zero();

    // initialize the covariance matrix to:
    // - Zero in the pose
    // - Infinite in the landmarks
    sigma_t = Eigen::Matrix<double, dim, dim>::Zero();
    Eigen::Matrix<double, dim, dim> land_mark_init =  Eigen::Matrix<double, dim, dim>::Constant(INT_MAX);
    sigma_t.block(2, 2, N, N) = land_mark_init;

    delta_t = dt;
}

void EKF::predictState() {
    Eigen::Matrix<double, 3, 1> update;

    if (u_t[1] != 0) {
        update << -u_t[0] / u_t[1] * sin(x_t[2]) + u_t[0] / u_t[1] * sin(x_t[2] + u_t[1] * delta_t),
                   u_t[0] / u_t[1] * cos(x_t[2]) - u_t[0] / u_t[1] * cos(x_t[2] + u_t[1] * delta_t),
                   u_t[1] * delta_t;
    } else {
        update << u_t[0] * sin(x_t[2]),
                  u_t[0] * cos(x_t[2]),
                  u_t[1] * delta_t;

    }
    
    x_t_pred.block(0, 0, 2, 1) += update;
}
void EKF::predictObservation() {
    Eigen::Matrix<double, 3, 3> G_t_x;

    if (u_t[1] != 0) {
        G_t_x << 0, 0, -u_t[0] / u_t[1] * cos(x_t_pred[2]) + u_t[0] / u_t[1] * cos(x_t_pred[2] + u_t[1] * delta_t),
                 0, 0, -u_t[0] / u_t[1] * sin(x_t_pred[2]) + u_t[0] / u_t[1] * sin(x_t_pred[2] + u_t[1] * delta_t),
                 0, 0, 0;
    } else {
        G_t_x << 0, 0,  cos(x_t_pred[2]),
                 0, 0, -sin(x_t_pred[2]),
                 0, 0, 0;
    }
    
    G_t_x += Eigen::Matrix<double, 3, 3>::Identity();

    G_t.block(0, 0, 2, 2) = G_t_x; 

    sigma_t_pred = G_t * sigma_t * G_t.transpose() + R;
}

void EKF::correctionStep(Eigen::Matrix<double, 2, 1> observation, int id) {
    // observation has the form of range sensor measurement: zt(r, phi)
    double r = observation[0];
    double phi = observation[1];

    if (map[id].count() == 0) {
        Eigen::Matrix<double, 2, 1> relative_measurement;
        relative_measurement << r * cos(phi + x_t_pred[2]),
                                r * sin(phi + x_t_pred[2]);

        map[id] = x_t_pred.block(0, 0, 2, 1) + relative_measurement;
    }

    // delta = landmark pose - robot pose
    Eigen::Matrix<double, 2, 1> delta;
    delta << map[id][0] - x_t_pred[0],
             map[id][1] - x_t_pred[1];

    double q = delta.transpose() * delta;

    Eigen::Matrix<double, 2, 1> z_pred;
    z_pred << pow(q, 0.5), 
              atan2(delta[0], delta[1] - x_t_pred[2]);

    Eigen::Matrix<double, 5, dim> F = Eigen::Matrix<double, 5, dim>::Zero();
    F.block(0, 0, 2, 2) = Eigen::Matrix<double, 3, 3>::Identity();
    int set_idx = 3 + 2*id;
    F(3, set_idx) = 1;
    F(4, set_idx + 1) = 1;

    Eigen::Matrix<double, 2, 5> H_low;
    H_low << -pow(q, 0.5) * delta[0], -pow(q, 0.5) * delta[1],  0 , pow(q, 0.5) * delta[0], pow(q, 0.5) * delta[1],
              delta[1]              , -delta[0]              , -q ,              -delta[1],               delta[0];
    H_low *= 1/q; // size: 2 x 5
    H_t = H_low * F; // size: 2 x 3 + 2N

    K_t = (sigma_t_pred * H_t.transpose()) * (H_t * sigma_t_pred * H_t.transpose() + Q).inverse(); // size K_t: 3 + 2N x 2
    x_t = x_t_pred + K_t * (observation - z_pred); // size: 3 + 2N x 1

    sigma_t = (Eigen::Matrix<double, dim ,dim>::Identity() - K_t * H_t) * sigma_t_pred; //size: 3 + 2N x 3 + 2N
}
