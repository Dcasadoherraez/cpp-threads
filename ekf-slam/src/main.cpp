#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include <sstream>
#include <unordered_map>

// signals
#include <signal.h>
#include <atomic>
std::atomic<bool> quit(false);    // signal flag

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ekf.h"
#include "drawer.h"

using namespace std;

int scale = 90;
double scale_sqrt = pow(scale, 0.5);
double scale_square = scale * scale;

void signalHandler( int signum ) {
   quit.store(true); 
}

string readFileIntoString(const string& path) {
    auto ss = ostringstream{};
    ifstream input_file(path);
    if (!input_file.is_open()) {
        cerr << "Could not open the file - '"
             << path << "'" << endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    return ss.str();
}

unordered_map<int, vector<string>> readFile(string filename) {
    string file_contents;
    unordered_map<int, vector<string>> csv_contents;
    char delimiter = ',';

    file_contents = readFileIntoString(filename);

    istringstream sstream(file_contents);
    vector<string> items;
    string record;

    int counter = 0;
    while (getline(sstream, record)) {
        istringstream line(record);
        while (getline(line, record, delimiter)) {
            items.push_back(record);
        }

        csv_contents[counter] = items;
        items.clear();
        counter += 1;
    }
    return csv_contents;
}

Eigen::Vector3d getInput(int &ct, unordered_map<int, vector<string>> data) {
    Eigen::Vector3d u_t;
    if (data[ct][0] == "ODOMETRY") {
        u_t[0] = stof(data[ct][3]) * scale; // v
        u_t[1] = stof(data[ct][2]); // w1
        u_t[2] = stof(data[ct][4]); // w2

        ct++;
    }

    return u_t;
}

unordered_map<int, Eigen::Vector2d> getObservations(int &ct, unordered_map<int, vector<string>> data) {
    unordered_map<int, Eigen::Vector2d> observationList;

    while (ct < data.size() - 1 && data[ct][0] == "SENSOR") {

        Eigen::Vector2d obs;
        int id = stoi(data[ct][1]);
        obs[0] = stof(data[ct][2]) * scale;
        obs[1] = stof(data[ct][3]);

        observationList[id] = obs;
        ct++;
    }
    return observationList;
}

unordered_map<int, Eigen::Vector2d> getMap(unordered_map<int, vector<string>> lmMap) {
    unordered_map<int, Eigen::Vector2d> landmarks;

    for (auto lm : lmMap) {
        Eigen::Vector2d pos;
        pos << stof(lm.second[1]) * scale,
               stof(lm.second[2]) * scale;
        landmarks[stoi(lm.second[0])] = pos;
    }

    return landmarks;
}

Eigen::Vector2d getUserInput() {
    cout << "Input robot command [v, w]: ";
    Eigen::Vector2d u_t;
    cin >> u_t[0];
    cin >> u_t[1];

    u_t[1] *= M_PI/180;
    return u_t;
}

unordered_map<int, Eigen::Vector2d> getUserObservations(int N) {
    unordered_map<int, Eigen::Vector2d> observationList;
    for (int i = 0; i < N; i++) {
        cout << "Input new observation, write -1 when done [id, r, phi]: ";
        Eigen::Vector2d obs;
        int id;
        cin >> id;
        if (id == -1) break;
        cin >> obs[0];
        cin >> obs[1];

        obs[1] *= M_PI/180;
        observationList[id] = obs;
    }

    return observationList;
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

    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = signalHandler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);

    cout << "Kalman Filter" << endl;

    Eigen::Matrix2d Q;
    Eigen::Matrix3d R;
    double sigma_r, sigma_phi, motion_noise;

    sigma_r =   1 / scale_square;
    sigma_phi = 1;

    motion_noise = 0.001 / scale_square;

    Q << sigma_r,    0, 
            0, sigma_phi;

    R << motion_noise, 0  ,    0,
           0   , motion_noise,    0,
           0   , 0  , motion_noise/10;

    // Q << 0, 0, 0, 0;
    // R << 0, 0, 0, 0, 0, 0, 0, 0, 0;
           
    double dt = 1;
    EKF::Ptr ekf(new EKF(Q, R, dt)); 
    Drawer::Ptr drawer(new Drawer());

    // Read data.csv
    string data("../data.csv");
    string world("../world.csv");
    unordered_map<int, vector<string>> sensor_data = readFile(data);
    unordered_map<int, vector<string>> map_data = readFile(world);

    int data_ct = 0;
    unordered_map<int, Eigen::Vector2d> lmMap = getMap(map_data);   
    drawer->DrawLandmarks(lmMap, false);
 
    // SLAM loop
    cout << "Going over " << sensor_data.size() << " states" << endl;
    while (data_ct < sensor_data.size() - 10) // 
    {
        // ekf->u_t = getUserInput();
        // ekf->z_t = getUserObservations(ekf->N);
        ekf->u_t = getInput(data_ct, sensor_data);
        ekf->z_t = getObservations(data_ct, sensor_data);

        // printState(ekf->x_gt);
        // printLandmarks(ekf->map_gt);

        // Predict 
        ekf->PredictionStep();
        // cout << "Predicted pose (prediction step): \n" << ekf->x_t_pred << endl;
        // cout << "Predicted sigma (prediction step): \n" << ekf->sigma_t_pred << endl;

        // drawer->DrawState(ekf->x_gt.block(0, 0, 3, 1), Eigen::Matrix3d::Zero(), false);
        // drawer->DrawLandmarks(ekf->map_gt, false);

        // Correct
        ekf->correctionStep();
        // cout << "Predicted pose (correction step): \n" << ekf->x_t << endl;
        // cout << "Predicted sigma (correction step): \n" << ekf->sigma_t << endl;

        drawer->DrawState(ekf->x_t.block(0, 0, 3, 1), ekf->sigma_t.block(0, 0, 2, 2), true);
        drawer->DrawLandmarks(ekf->map_t, true);

        // cout << "Cov: \n" << ekf->sigma_t << endl;
        if( quit.load() ) break;    // exit normally after SIGINT


  
    }
    
        cv::imshow("image", drawer->current_drawing);
        cv::waitKey(0);
    return 0;
}
