#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

// for cv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

class Drawer {
public: 
    typedef shared_ptr<Drawer> Ptr;
    cv::Mat current_drawing;

    Drawer();
    void Clear();
    void DrawState(Eigen::Vector3d pos, Eigen::Matrix2d pose_cov, bool prediction);
    void DrawLandmarks(unordered_map<int, Eigen::Matrix<double, 2, 1>>positions, bool prediction);
    cv::RotatedRect GetErrorEllipse(Eigen::Vector2d pt, Eigen::Matrix2d cov);
    void Update();
};