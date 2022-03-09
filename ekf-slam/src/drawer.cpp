#include "drawer.h"

Drawer::Drawer()
{
  new_data = false;
  stop = false;
  time = "";

  Clear();
  drawer_thread = thread(&Drawer::DrawerLoop, this);
}

void Drawer::DrawerLoop()
{
  while (!stop)
  {
    cv::imshow("image", current_drawing);
    cv::waitKey(1);

    if (new_data) {
      Clear();
      DrawState(true);
      DrawLandmarks(real_map, false);       // draw estimation
      DrawLandmarks(predicted_map, true); // draw map lm
      SetNewData(false);
    }

    if (new_time) {
        DrawTime();
        SetNewTime("", false);
    }

  }

  cout << "Closing drawer..." << endl;
}

void Drawer::Clear()
{
  current_drawing = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));;
}

void Drawer::DrawTime() {

  cv::Point pos;
  pos.x = 600;
  pos.y = 50;

  cv::putText(current_drawing,      // target image
              time, // text
              pos,              // top-left position
              cv::FONT_HERSHEY_DUPLEX,
              0.8,
              cv::Scalar(255, 0, 255),
              2);
}

void Drawer::DrawState(bool prediction)
{

  cv::RotatedRect ellipse = GetErrorEllipse();

  cv::Scalar color;
  if (prediction)
  {
    // color magenta
    color = cv::Scalar(255, 0, 255);
  }
  else
  {
    // color blue
    color = cv::Scalar(255, 0, 0);
  }

  cv::Point center;
  center.x = pose[0];
  center.y = pose[1];

  cv::Point orientation;
  orientation.x = center.x + 50 * cos(pose[2]);
  orientation.y = center.y + 50 * sin(pose[2]);

  cv::circle(current_drawing,
             center,
             5,
             color,
             cv::FILLED);

  cv::arrowedLine(current_drawing,
                  center, orientation,
                  color);

  cv::ellipse(current_drawing, ellipse, color, 1);
}

void Drawer::DrawLandmarks(unordered_map<int, Eigen::Vector2d> positions, bool prediction)
{

  cv::Scalar color;
  if (prediction)
  {
    // color magenta
    color = cv::Scalar(255, 0, 255);
  }
  else
  {
    // color blue
    color = cv::Scalar(255, 0, 0);
  }

  for (auto pos : positions)
  {

    cv::Point topleft, bottomright;
    topleft.x = pos.second[0] - 5;
    topleft.y = pos.second[1] - 5;

    bottomright.x = pos.second[0] + 5;
    bottomright.y = pos.second[1] + 5;

    cv::rectangle(current_drawing,
                  topleft, bottomright,
                  color,
                  cv::FILLED);

    cv::putText(current_drawing,      // target image
                to_string(pos.first), // text
                topleft,              // top-left position
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                color,
                2);
  }
}

// https://gist.github.com/eroniki/2cff517bdd3f9d8051b5

cv::RotatedRect Drawer::GetErrorEllipse()
{

  cv::Point2f mean;
  mean.x = pose[0];
  mean.y = pose[1];

  cv::Mat covmat;
  cv::eigen2cv(cov, covmat);

  double chisquare_val = 150; // 2.4477
  // Get the eigenvalues and eigenvectors
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(covmat, eigenvalues, eigenvectors);

  // Calculate the angle between the largest eigenvector and the x-axis
  double angle = atan2(eigenvectors.at<double>(0, 1), eigenvectors.at<double>(0, 0));

  // Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
  if (angle < 0)
    angle += 2 * M_PI;

  // Conver to degrees instead of radians
  angle = 180 * angle / M_PI;

  // Calculate the size of the minor and major axes
  double halfmajoraxissize = chisquare_val * sqrt(eigenvalues.at<double>(0));
  double halfminoraxissize = chisquare_val * sqrt(eigenvalues.at<double>(1));

  double ax = max(halfmajoraxissize, halfminoraxissize);
  halfmajoraxissize = isnan(halfmajoraxissize) ? halfminoraxissize : halfmajoraxissize;
  halfminoraxissize = isnan(halfminoraxissize) ? halfmajoraxissize : halfminoraxissize;

  // cout << covmat << endl;
  // cout << "Ellipse: " << halfmajoraxissize << "," << halfminoraxissize << endl;

  // Return the oriented ellipse
  // The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
  return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);
}

void Drawer::Update()
{
}
