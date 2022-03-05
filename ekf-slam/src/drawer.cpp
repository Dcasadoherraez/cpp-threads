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

void Drawer::DrawPrediction(Eigen::Vector3d pos) {
    cv::Point center;
    center.x = pos[0];
    center.y = pos[1];

    cv::Point orientation;
    orientation.x = 10 * cos(pos[2]);
    orientation.y = 10 * sin(pos[2]);

    cv::circle( current_drawing,
      center,
      5,
      cv::Scalar( 0, 0, 255 ),
      cv::FILLED);

    cv::arrowedLine( current_drawing, 
      center, orientation,
      cv::Scalar(0, 0, 255 )
      );

}

void Drawer::DrawState(Eigen::Vector3d pos) {
    cv::Point center;
    center.x = pos[1];
    center.y = pos[0];

    cv::Point orientation;
    orientation.x = center.x + 100 * cos(pos[2]);
    orientation.y = center.y + 100 * sin(pos[2]);

    cv::circle( current_drawing,
      center,
      5,
      cv::Scalar( 0, 255, 0 ),
      cv::FILLED);

    cv::arrowedLine( current_drawing, 
      center, orientation,
      cv::Scalar(0, 255, 0)
      );

}

void Drawer::DrawLandmarks(unordered_map<int, Eigen::Matrix<double, 2, 1>> positions) {

  for (auto pos : positions) {

    cv::Point topleft, bottomright;
    topleft.x = pos.second[0];
    topleft.y = pos.second[1];

    bottomright.x = pos.second[0] + 10;
    bottomright.y = pos.second[1] + 10;

    cv::rectangle( current_drawing,
      topleft, bottomright,
      cv::Scalar( 255, 0, 0 ),
      cv::FILLED);

    cv::putText(current_drawing, //target image
      to_string(pos.first), //text
      topleft, //top-left position
      cv::FONT_HERSHEY_DUPLEX,
      0.8,
      cv::Scalar( 255, 0, 0 ),
      2);
  }

}


void Drawer::Update() {

}
