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

    Drawer(string background_path);

    void DrawPrediction(Eigen::Vector3d pos);
    void DrawState(Eigen::Vector3d pos);
    void Update();
    void DrawLandmarks(unordered_map<int, Eigen::Matrix<double, 2, 1>>positions);
};