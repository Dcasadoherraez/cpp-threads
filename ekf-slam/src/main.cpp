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
    
    cout << "Running EKF-SLAM" << endl;
    double sensor_noise, motion_noise;

    sensor_noise = 0.01; // * scale_square;
    motion_noise = 0.01; // * scale_square;

    double dt = 1;

    // Object initialization
    EKF::Ptr ekf = make_shared<EKF>(scale, sensor_noise, motion_noise, dt); 
    Drawer::Ptr drawer = make_shared<Drawer>();
    FileReader::Ptr filereader = make_shared<FileReader>(scale);
    TimeDisplay::Ptr timedisplay = make_shared<TimeDisplay>();
    
    string data("../data.csv");
    string world("../world.csv");

    // Pointer assinment
    ekf->drawer = drawer;
    timedisplay->drawer = drawer;
    ekf->filereader = filereader;
    
    // Thread spawning
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
