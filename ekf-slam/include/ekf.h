#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <unordered_map>

using namespace std;


class EKF {
public:
    typedef shared_ptr<EKF> Ptr;

    static const int N = 10; // num of landmarks
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
    
    double ConstrainAngle(double x);
    
    void PredictState();
    void PredictCovariance();

    void PredictionStep();
    void CorrectionStep();

};