#include <Eigen/Core>
#include <Eigen/Geometry>

// for cv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "drawer.h"

using namespace std;

Drawer::Drawer(string background_path) {
    current_drawing = cv::imread(background_path);
}

void Drawer::DrawPrediction(Eigen::Vector2d pos) {
    cv::Point center;
    center.x = pos[0];
    center.y = pos[1];

    cv::circle( current_drawing,
      center,
      5,
      cv::Scalar( 0, 0, 255 ),
      cv::FILLED);
}

void Drawer::DrawState(Eigen::Vector2d pos) {
    cv::Point center;
    center.x = pos[0];
    center.y = pos[1];

    cv::circle( current_drawing,
      center,
      5,
      cv::Scalar( 0, 255, 0 ),
      cv::FILLED);

}

void Drawer::Update() {

}