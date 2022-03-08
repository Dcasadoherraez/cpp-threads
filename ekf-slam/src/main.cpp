#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unistd.h>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ekf.h"
#include "time_display.h"
// #include "drawer.h"
// #include "file_reader.h"

using namespace std;

int scale = 90;
double scale_sqrt = pow(scale, 0.5);
double scale_square = scale * scale;


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

    cout << "Kalman Filter" << endl;

    double sensor_noise, motion_noise;

    sensor_noise = 0.01; // * scale_square;
    motion_noise = 0.01; // * scale_square;

    double dt = 1;

    EKF::Ptr ekf = make_shared<EKF>(scale, sensor_noise, motion_noise, dt); 
    Drawer::Ptr drawer = make_shared<Drawer>();
    FileReader::Ptr filereader = make_shared<FileReader>(scale);
    TimeDisplay::Ptr timedisplay = make_shared<TimeDisplay>();
    
    string data("../data.csv");
    string world("../world.csv");

    ekf->drawer = drawer;
    timedisplay->drawer = drawer;
    ekf->filereader = filereader;
    

    thread draw = thread(&Drawer::DrawerLoop, drawer);
    thread ekf_slam = thread(&EKF::MainLoop, ekf, data, world, parallel);
    thread time_display = thread(&TimeDisplay::DisplayTime, timedisplay);

    ekf_slam.join();

    timedisplay->SetStop(true);
    time_display.join();

    drawer->SetStop(true);
    draw.join();

    return 0;
}
