#pragma once
#ifndef DRAWER_H
#define DRAWER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <iostream>

// for cv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <thread>

using namespace std;

class Drawer
{
public:
    typedef shared_ptr<Drawer> Ptr;
    cv::Mat current_drawing;

    mutex data_mutex;

    unordered_map<int, Eigen::Vector2d> real_map;
    unordered_map<int, Eigen::Vector2d> predicted_map;
    Eigen::Vector3d pose;
    Eigen::Matrix2d cov;
    string time;
    bool stop;
    bool new_data;
    bool new_time;

    thread drawer_thread;

    Drawer();
    void DrawerLoop();

    void Clear();
    void DrawState(bool prediction);
    void DrawLandmarks(unordered_map<int, Eigen::Vector2d> positions, bool prediction);
    void DrawTime();
    void Update();
    cv::RotatedRect GetErrorEllipse();

    void SetMap(unordered_map<int, Eigen::Vector2d> real_m, unordered_map<int, Eigen::Vector2d> pred_m)
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        real_map = real_m;
        predicted_map = pred_m;
    }

    void SetPoseCov(Eigen::Vector3d x, Eigen::Matrix2d c)
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        pose = x;
        cov = c;
    }

    void SetStop(bool ss)
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        stop = ss;
    }

    void SetNewData(bool new_d)
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        new_data = new_d;
    }

    void SetNewTime(string t, bool nt)
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        time = nt ? t : time;
        new_time = nt;
    }

};

#endif
