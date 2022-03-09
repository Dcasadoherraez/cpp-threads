#pragma once
#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <unordered_map>
#include <thread>

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <unistd.h>

#include <mutex>
#include <shared_mutex>

#include <atomic>
#include <signal.h>

#include "drawer.h"
#include "file_reader.h"

using namespace std;

class EKF
{
public:
    typedef shared_ptr<EKF> Ptr;
    
    mutex data_mutex;

    Drawer::Ptr drawer;
    FileReader::Ptr filereader;

    static const int N = 100; // num of landmarks
    static const int dim = 2 * N + 3;
    double delta_t;
    int scale;

    double sensor_noise;
    double motion_noise;

    // current input
    Eigen::Vector3d u_t;
    unordered_map<int, Eigen::Vector2d> z_t;

    Eigen::Matrix<double, dim, 1> x_t, x_t_pred;
    Eigen::Matrix<double, dim, dim> sigma_t, sigma_t_pred;

    // landmark vector {id, [x, y]}
    unordered_map<int, Eigen::Vector2d> map_t, map_gt;

    EKF(int scale, double sensor_uncertainty, double motion_uncertainty, double dt);

    ~EKF() {}

    void MainLoop(string data, string world, bool parallel);

    double ConstrainAngle(double x);

    // EKF-SLAM Steps
    void PredictionStep(bool is_parallel);
    void CorrectStep(bool is_parallel);

    // Sequential processing
    void PredictState();
    void PredictCovariance();
    void CorrectionStep();

    // Parallel processing
    void ParallelPrint();
    void PredictCovarianceParallel();
    void GetTopLeft(Eigen::Matrix3d &G_t_x, Eigen::Matrix3d &R_t);
    void GetTopRight(Eigen::Matrix3d &G_t_x);
    void GetBottomLeft(Eigen::Matrix3d &G_t_x);
    void GetBottomRight(Eigen::Matrix3d &G_t_x);

    void CorrectionStepParallel();
    void ComputeObservationH(int ct, int id, Eigen::Vector2d observation, int &m, Eigen::MatrixXd &H_t, Eigen::VectorXd &Z_diff);
    void PerformUpdate(int m, Eigen::MatrixXd &H_t, Eigen::VectorXd &Z_diff);

    void InitMap() {
        unique_lock<std::mutex> lck(data_mutex);
        map_t = {};
    }
    void SetMap(int id, Eigen::Vector2d lm_pos) {
        unique_lock<std::mutex> lck(data_mutex);
        map_t[id] = lm_pos;
    }

    std::unordered_map<int, Eigen::Vector2d> GetMap() {
        unique_lock<std::mutex> lck(data_mutex);
        return map_t;
    }


    bool IsInMap(int id) {
        unique_lock<std::mutex> lck(data_mutex);
        return map_t.count(id);
    }
};

#endif