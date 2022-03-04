#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ekf.h"
#include "drawer.h"

using namespace std;


int main(int argc, char **argv) {
    cout << "Kalman Filter" << endl;
    string background = "../plot.jpg";

    Eigen::Matrix<double, 2, 2> Q = Eigen::Matrix<double, 2, 2>::Zero();
    double dt = 0.1;

    EKF::Ptr ekf(new EKF(Q, dt)); 
    Drawer::Ptr drawer(new Drawer(background));

    Eigen::Matrix<double, 2, 1> u_t, z_t, obs;
    u_t << 1, 1;
    obs << 2, 2;

    drawer->DrawPrediction(obs);
    cv::imshow("image", drawer->current_drawing);
    cv::waitKey(0);

    ekf->u_t = u_t;
    ekf->predictState();
    ekf->predictObservation();
    ekf->correctionStep(obs, 0);

    // for all observed features 
    
    return 0;
}
