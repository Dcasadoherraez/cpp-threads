#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <csignal>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ekf.h"
#include "drawer.h"

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286

using namespace std;

void signalHandler( int signum ) {
   cout << "Interrupt signal (" << signum << ") received.\n";
   exit(signum);  
}

Eigen::Vector2d getInput() {
    cout << "Input robot command [v, w]: ";
    Eigen::Vector2d u_t;
    cin >> u_t[0];
    cin >> u_t[1];

    u_t[1] *= PI/180;
    return u_t;
}

vector<pair<int, Eigen::Vector2d>> getObservations(int N) {
    vector<pair<int, Eigen::Vector2d>> observationList;
    for (int i = 0; i < N; i++) {
        cout << "Input new observation, write -1 when done [id, r, phi]: ";
        Eigen::Vector2d obs;
        int id;
        cin >> id;
        if (id == -1) break;
        cin >> obs[0];
        cin >> obs[1];

        obs[1] *= PI/180;
        pair<int, Eigen::Vector2d> obsPair = {id, obs};
        observationList.push_back(obsPair);
    }

    return observationList;
}

void printState(Eigen::Matrix<double, 23, 1> x_gt) {
    cout << "Real state: " << "[" << x_gt[0] << "," << x_gt[1] << "," << x_gt[2] << "]" << endl;
}

void printLandmarks(std::unordered_map<int, Eigen::Vector2d> map_gt) {
    cout << "Landmark poses: " << endl;

    for (auto lm : map_gt) {
        cout << lm.first << "[" << lm.second[0] << "," << lm.second[1] << "]" << endl; 
    }
}

int main(int argc, char **argv) {
    cout << "Kalman Filter" << endl;
    string background = "../plot.jpg";

    Eigen::Matrix<double, 2, 2> Q = Eigen::Matrix<double, 2, 2>::Zero();
    double dt = 1;

    EKF::Ptr ekf(new EKF(Q, dt)); 
    Drawer::Ptr drawer(new Drawer(background));

    vector<Eigen::Vector2d> inputs;
    //                  id  2d landmark location                             
    vector<vector<pair<int, Eigen::Vector2d>>> observations;

    while (1)
    {
        ekf->u_t = getInput();
        ekf->z_t = getObservations(ekf->N);

        ekf->ComputeStateGT();
        ekf->ComputeObsGT();

        printState(ekf->x_gt);
        printLandmarks(ekf->map_gt);
        // ekf->predictState();
        // ekf->predictObservation();
        // ekf->correctionStep();


        drawer->DrawState(ekf->x_gt.block(0, 0, 3, 1));
        drawer->DrawLandmarks(ekf->map_gt);
        cout << "h";
        cv::imshow("image", drawer->current_drawing);
        cv::waitKey(0);
    }
    


    // for all observed features 
    
    return 0;
}
