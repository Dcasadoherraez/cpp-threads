#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unistd.h>

// signals
#include <signal.h>
#include <atomic>
std::atomic<bool> quit(false);    // signal flag

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ekf.h"
#include "drawer.h"
#include "file_reader.h"

using namespace std;

int scale = 90;
double scale_sqrt = pow(scale, 0.5);
double scale_square = scale * scale;

void signalHandler( int signum ) {
   quit.store(true); 
}

void printState(Eigen::Matrix<double, 23, 1> x_gt) {
    cout << "Real state: " << "[" << x_gt[0] << "," << x_gt[1] << "," << x_gt[2] << "]" << endl;
}

void printLandmarks(unordered_map<int, Eigen::Vector2d> map_gt) {
    cout << "Landmark poses: " << endl;

    for (auto lm : map_gt) {
        cout << lm.first << ": [" << lm.second[0] << "," << lm.second[1] << "]" << endl; 
    }
}

int main(int argc, char **argv) {

    bool parallel = false;
    if (*argv[1] == '1') {
        parallel = true;
        cout << "Running in parallel" << endl;
    } else if (*argv[1] == '0') {
        cout << "Running in sequential" << endl;
        parallel = false;
    } else {
        cout << "Invalid arg" << endl;
        return -1;
    }

    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = signalHandler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);

    cout << "Kalman Filter" << endl;

    double sensor_noise, motion_noise;

    sensor_noise = 0.01; // * scale_square;
    motion_noise = 0.01; // * scale_square;

    double dt = 1;
    EKF::Ptr ekf(new EKF(scale, sensor_noise, motion_noise, dt)); 
    Drawer::Ptr drawer(new Drawer());
    FileReader::Ptr filereader(new FileReader(scale));

    // Read data.csv
    string data("../data_fake.csv");
    string world("../world.csv");
    unordered_map<int, vector<string>> sensor_data = filereader->ReadFile(data);
    unordered_map<int, vector<string>> map_data = filereader->ReadFile(world);

    int data_ct = 0;
    unordered_map<int, Eigen::Vector2d> lmMap = filereader->GetMap(map_data);   
 
    // SLAM loop
    cout << "Going over " << sensor_data.size() << " states" << endl;
    while (data_ct < sensor_data.size() - 1) 
    {
        usleep(100000);
        // ekf->u_t = getUserInput();
        // ekf->z_t = getUserObservations(ekf->N);

        ekf->u_t = filereader->GetInput(data_ct, sensor_data);
        ekf->z_t = filereader->GetObservations(data_ct, sensor_data);
    
        // printState(ekf->x_gt);
        // printLandmarks(ekf->map_gt);

        // Predict 
        auto startP = chrono::system_clock::now();
        ekf->PredictionStep(parallel);
        auto endP = chrono::system_clock::now();
        chrono::duration<float, milli> durationP = endP - startP;
        cout << "Pred. t: " << durationP.count();
        // cout << "Predicted pose (prediction step): \n" << ekf->x_t_pred << endl;
        // cout << "Predicted sigma (prediction step): \n" << ekf->sigma_t_pred << endl;

        // drawer->DrawState(ekf->x_gt.block(0, 0, 3, 1), Eigen::Matrix3d::Zero(), false);
        // drawer->DrawLandmarks(ekf->map_gt, false);

        // Correct
        auto startC = chrono::system_clock::now();
        ekf->CorrectStep(parallel);
        auto endC = chrono::system_clock::now();
        chrono::duration<float, milli> durationC = endC - startC;
        cout << " Corr. t: " << durationC.count() << endl;

        // cout << "Predicted pose (correction step): \n" << ekf->x_t << endl;
        // cout << "Predicted sigma (correction step): \n" << ekf->sigma_t << endl;

        drawer->Clear();
        drawer->DrawState(ekf->x_t.block(0, 0, 3, 1), ekf->sigma_t.block(0, 0, 2, 2), true);
        drawer->DrawLandmarks(ekf->map_t, true); // draw estimation
        drawer->DrawLandmarks(lmMap, false); // draw map lm

        // cout << "Cov: \n" << ekf->sigma_t.block(0, 0, 3, 3) << endl;
        if( quit.load() ) break;    // exit normally after SIGINT



        cv::imshow("image", drawer->current_drawing);
        cv::waitKey(1);
    }
    
  
    return 0;
}
