#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

// for cv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "drawer.h"

using namespace std;

Drawer::Drawer() {
  Clear();
}

void Drawer::Clear() {
  cv::Mat bg(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
  current_drawing = bg;
}

void Drawer::DrawState(Eigen::Vector3d pos, Eigen::Matrix2d pose_cov, bool prediction) {

  cv::RotatedRect ellipse = GetErrorEllipse(pos.block(0, 0, 2, 1), pose_cov);

  cv::Scalar color;
  if (prediction) {
    // color magenta
    color = cv::Scalar(255, 0, 255);
  } else {
    // color blue
    color = cv::Scalar(255, 0, 0);
  }

  cv::Point center;
  center.x = pos[0];
  center.y = pos[1];

  cv::Point orientation;
  orientation.x = center.x + 50 * cos(pos[2]);
  orientation.y = center.y + 50 * sin(pos[2]);

  cv::circle( current_drawing,
    center,
    5,
    color,
    cv::FILLED);

  cv::arrowedLine( current_drawing, 
    center, orientation,
    color
    );

	cv::ellipse(current_drawing, ellipse, color, 1);

}

void Drawer::DrawLandmarks(unordered_map<int, Eigen::Vector2d> positions, bool prediction) {

  cv::Scalar color;
  if (prediction) {
    // color magenta
    color = cv::Scalar(255, 0, 255);
  } else {
    // color blue
    color = cv::Scalar(255, 0, 0);
  }

  for (auto pos : positions) {

    cv::Point topleft, bottomright;
    topleft.x = pos.second[0] - 5;
    topleft.y = pos.second[1] - 5;

    bottomright.x = pos.second[0] + 5;
    bottomright.y = pos.second[1] + 5;

    cv::rectangle( current_drawing,
      topleft, bottomright,
      color,
      cv::FILLED);

    cv::putText(current_drawing, //target image
      to_string(pos.first), //text
      topleft, //top-left position
      cv::FONT_HERSHEY_DUPLEX,
      0.8,
      color,
      2);
  }

}

// https://gist.github.com/eroniki/2cff517bdd3f9d8051b5

cv::RotatedRect Drawer::GetErrorEllipse(Eigen::Vector2d pt, Eigen::Matrix2d cov){
	
  cv::Point2f mean;
  mean.x = pt[0];
  mean.y = pt[1];

  cv::Mat covmat;
  cv::eigen2cv(cov, covmat);

  double chisquare_val = 1000; // 2.4477
	//Get the eigenvalues and eigenvectors
	cv::Mat eigenvalues, eigenvectors;
	cv::eigen(covmat, eigenvalues, eigenvectors);

	//Calculate the angle between the largest eigenvector and the x-axis
	double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));

	//Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
	if(angle < 0)
		angle += 2*M_PI;

	//Conver to degrees instead of radians
	angle = 180*angle/M_PI;

	//Calculate the size of the minor and major axes
	double halfmajoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(0));
	double halfminoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(1));

  double ax = max(halfmajoraxissize, halfminoraxissize);
  halfmajoraxissize = isnan(halfmajoraxissize) ? halfminoraxissize : halfmajoraxissize;
  halfminoraxissize = isnan(halfminoraxissize) ? halfmajoraxissize : halfminoraxissize;

  // cout << covmat << endl;
  // cout << "Ellipse: " << halfmajoraxissize << "," << halfminoraxissize << endl;

	//Return the oriented ellipse
	//The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
	return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

}

void Drawer::Update() {

}
