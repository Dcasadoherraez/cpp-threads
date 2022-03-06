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

    Eigen::Matrix<double, 2, 2> Q;
    Eigen::Matrix<double, dim, dim> R;

    // current input
    Eigen::Matrix<double, 1, 2> u_t;
    unordered_map<int, Eigen::Vector2d> z_t;

    Eigen::Matrix<double, dim, 1> x_t, x_t_pred, x_gt;
    Eigen::Matrix<double, dim, dim> sigma_t, sigma_t_pred;

    Eigen::Matrix<double, dim, dim> G_t;
    Eigen::Matrix<double, 2, dim> H_t;

    Eigen::Matrix<double, dim, 2> K_t;

    // landmark vector {id, [x, y]}
    unordered_map<int, Eigen::Vector2d> map, map_gt;

    EKF(Eigen::Matrix<double, 2, 2> sensor_uncertainty, double dt);

    ~EKF() {}

    void predictState();
    void predictObservation();
    void correctionStep();

    void ComputeStateGT();
    void ComputeObsGT();
};