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

    Eigen::Matrix2d Q;
    Eigen::Matrix3d R_t;

    // current input
    Eigen::Vector3d u_t;
    unordered_map<int, Eigen::Vector2d> z_t;

    Eigen::Matrix<double, dim, 1> x_t, x_t_pred, x_gt;
    Eigen::Matrix<double, dim, dim> sigma_t, sigma_t_pred;

    Eigen::Matrix<double, dim, dim> G_t;
    Eigen::Matrix<double, 2, dim> H_t;

    Eigen::Matrix<double, dim, 2> K_t;

    // landmark vector {id, [x, y]}
    unordered_map<int, Eigen::Vector2d> map_t, map_gt;

    EKF(Eigen::Matrix2d sensor_uncertainty, Eigen::Matrix3d motion_uncertainty, double dt);

    ~EKF() {}
    
    double ConstrainAngle(double x);
    
    void PredictState();
    void PredictCovariance();

    void PredictionStep();
    void correctionStep();

};