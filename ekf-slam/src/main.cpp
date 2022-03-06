#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

// signals
#include <signal.h>
#include <atomic>
std::atomic<bool> quit(false);    // signal flag

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ekf.h"
#include "drawer.h"

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286

using namespace std;

void signalHandler( int signum ) {
   quit.store(true); 
}

Eigen::Vector2d getInput() {
    cout << "Input robot command [v, w]: ";
    Eigen::Vector2d u_t;
    cin >> u_t[0];
    cin >> u_t[1];

    u_t[1] *= PI/180;
    return u_t;
}

unordered_map<int, Eigen::Vector2d> getObservations(int N) {
    unordered_map<int, Eigen::Vector2d> observationList;
    for (int i = 0; i < N; i++) {
        cout << "Input new observation, write -1 when done [id, r, phi]: ";
        Eigen::Vector2d obs;
        int id;
        cin >> id;
        if (id == -1) break;
        cin >> obs[0];
        cin >> obs[1];

        obs[1] *= PI/180;
        observationList[id] = obs;
        cout << "Your input: " << id << ", [" << obs[0] << "," << obs[1] << "]" << endl;
    }

    return observationList;
}

void printState(Eigen::Matrix<double, 23, 1> x_gt) {
    cout << "Real state: " << "[" << x_gt[0] << "," << x_gt[1] << "," << x_gt[2] << "]" << endl;
}

void printLandmarks(std::unordered_map<int, Eigen::Vector2d> map_gt) {
    cout << "Landmark poses: " << endl;

    for (auto lm : map_gt) {
        cout << lm.first << ": [" << lm.second[0] << "," << lm.second[1] << "]" << endl; 
    }
}

int main(int argc, char **argv) {

    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = signalHandler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);


    cout << "Kalman Filter" << endl;
    string background = "../plot.jpg";

    double sigma_v = 0;
    double sigma_w = 0;

    Eigen::Matrix2d Q;
    Q << sigma_v,    0, 
            0, sigma_w;
            
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

        ekf->ComputeObsGT();
        ekf->ComputeStateGT();

        printState(ekf->x_gt);
        printLandmarks(ekf->map_gt);

        // Predict 
        ekf->PredictionStep();
        cout << "Predicted pose (prediction step): \n" << ekf->x_t_pred << endl;
        cout << "Predicted sigma (prediction step): \n" << ekf->sigma_t_pred << endl;

        drawer->DrawState(ekf->x_gt.block(0, 0, 3, 1), Eigen::Matrix3d::Zero(), false);
        drawer->DrawLandmarks(ekf->map_gt, false);

        // Correct
        ekf->correctionStep();
        cout << "Predicted pose (correction step): \n" << ekf->x_t << endl;
        cout << "Predicted sigma (correction step): \n" << ekf->sigma_t << endl;

        drawer->DrawState(ekf->x_t.block(0, 0, 3, 1), ekf->sigma_t.block(0, 0, 3, 3), true);
        drawer->DrawLandmarks(ekf->map, true);


        cv::imshow("image", drawer->current_drawing);
        cv::waitKey(0);
        if( quit.load() ) break;    // exit normally after SIGINT

    }
    

    // for all observed features 
    return 0;
}
