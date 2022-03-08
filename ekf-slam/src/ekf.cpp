#include "ekf.h"

std::atomic<bool> quit(false); // signal flag
void signalHandler(int signum)
{
    quit.store(true);
}

using namespace std;

EKF::EKF(int s, double s_noise, double m_noise, double dt)
{
    sensor_noise = s_noise;
    motion_noise = m_noise;
    delta_t = dt;
    scale = s;

    // initialize the pose to the origin of local coordinates
    x_t = Eigen::Matrix<double, dim, 1>::Zero();
    x_t_pred = Eigen::Matrix<double, dim, 1>::Zero();
    ;

    // initialize the covariance matrix to:
    // - Zero in the pose
    // - Infinite in the landmarks
    sigma_t = Eigen::Matrix<double, dim, dim>::Zero();
    sigma_t.block(3, 3, 2 * N, 2 * N) = Eigen::Matrix<double, 2 * N, 2 * N>::Ones() * INT_MAX; // landmark init

    map_t = {};
    map_gt = {};

}

double EKF::ConstrainAngle(double angle)
{
    while (angle > M_PI)
    {
        angle = angle - 2 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle = angle + 2 * M_PI;
    }
    return angle;
}

void EKF::PredictionStep(bool is_parallel)
{
    PredictState();

    if (is_parallel)
        PredictCovarianceParallel();
    else
        PredictCovariance();
}

void EKF::PredictState()
{
    Eigen::Vector3d update;

    update << u_t(0) * cos(x_t(2) + u_t(1) * delta_t),
        u_t(0) * sin(x_t(2) + u_t(1) * delta_t),
        ConstrainAngle(delta_t * (u_t(1) + u_t(2)));

    x_t_pred.block(0, 0, 3, 1) += update;
    x_t_pred(2) = ConstrainAngle(x_t_pred(2));
}

void EKF::GetTopLeft(Eigen::Matrix3d &G_t_x, Eigen::Matrix3d &R_t)
{
    sigma_t_pred.block(0, 0, 3, 3) = G_t_x * sigma_t.block(0, 0, 3, 3) * G_t_x.transpose() + R_t;
}

void EKF::GetTopRight(Eigen::Matrix3d &G_t_x)
{
    sigma_t_pred.block(0, 3, 3, 2 * N) = G_t_x * sigma_t.block(0, 3, 3, 2 * N);
}
void EKF::GetBottomLeft(Eigen::Matrix3d &G_t_x)
{
    sigma_t_pred.block(3, 0, 2 * N, 3) = (G_t_x * sigma_t.block(0, 3, 3, 2 * N)).transpose();
}

void EKF::GetBottomRight(Eigen::Matrix3d &G_t_x)
{
    sigma_t_pred.block(3, 3, 2 * N, 2 * N) = sigma_t.block(3, 3, 2 * N, 2 * N);
}

void EKF::PredictCovarianceParallel()
{
    Eigen::Matrix3d G_t_x;

    G_t_x << 1, 0, -u_t(0) * sin(x_t(2) + u_t(1) * delta_t),
        0, 1, u_t(0) * cos(x_t(2) + u_t(1) * delta_t),
        0, 0, 1;

    Eigen::Matrix3d R_t;
    R_t << motion_noise / (scale), 0, 0,
        0, motion_noise / (scale), 0,
        0, 0, motion_noise / (100 * scale * scale);

    vector<thread *> threads;

    threads.push_back(new thread(&EKF::GetTopLeft, this, ref(G_t_x), ref(R_t)));
    threads.push_back(new thread(&EKF::GetTopRight, this, ref(G_t_x)));
    threads.push_back(new thread(&EKF::GetBottomLeft, this, ref(G_t_x)));
    threads.push_back(new thread(&EKF::GetBottomRight, this, ref(G_t_x)));

    for (thread *t : threads)
    {
        t->join();
    }
}

void EKF::PredictCovariance()
{
    Eigen::Matrix3d G_t_x;

    G_t_x << 1, 0, -u_t(0) * sin(x_t(2) + u_t(1) * delta_t),
        0, 1, u_t(0) * cos(x_t(2) + u_t(1) * delta_t),
        0, 0, 1;

    Eigen::Matrix3d R_t;
    R_t << motion_noise / (scale), 0, 0,
        0, motion_noise / (scale), 0,
        0, 0, motion_noise / (100 * scale * scale);

    // top left block
    GetTopLeft(G_t_x, R_t);

    // top right block
    GetTopRight(G_t_x);

    // bottom left block
    GetBottomLeft(G_t_x);

    // bottom right block
    GetBottomRight(G_t_x);
}

void EKF::ComputeObservationH(int ct, int &id, Eigen::Vector2d &observation, int &m, Eigen::MatrixXd &H_t, Eigen::VectorXd &Z_diff)
{

    usleep(1000);
    // observation has the form of range sensor measurement: zt(r, phi)
    double r = observation(0);
    double phi = observation(1);

    int landmark_idx = 3 + id * 2;

    if (!map_t.count(id))
    {

        // first landmark location estimate
        x_t_pred(landmark_idx) = x_t_pred(0) + r * cos(phi + x_t_pred(2));
        x_t_pred(landmark_idx + 1) = x_t_pred(1) + r * sin(phi + x_t_pred(2));

        map_t[id] = x_t_pred.block(landmark_idx, 0, 2, 1);
    }

    // cout << "Predicted state: " << x_t_pred.block(0, 0, 2, 1).transpose() << endl;
    // cout << "Predicted landmark " << landmark_idx << ": " << x_t_pred(landmark_idx) << "," << x_t_pred(landmark_idx + 1) << endl;
    // cout << "Measurement: " << observation.transpose() << endl;

    // delta = landmark pose - robot pose (same as relative measurement)
    Eigen::Vector2d delta = x_t_pred.block(landmark_idx, 0, 2, 1) - x_t_pred.block(0, 0, 2, 1);

    double q = delta.transpose() * delta;
    double q_sqrt = pow(q, 0.5);

    Eigen::Vector2d z_pred;
    z_pred << q_sqrt,
        ConstrainAngle(atan2(delta(1), delta(0)) - x_t_pred(2));

    Z_diff(ct * 2) = r - z_pred(0);
    Z_diff(ct * 2 + 1) = ConstrainAngle(phi - z_pred(1));

    // Eigen::Matrix<double, 5, dim> F = Eigen::Matrix<double, 5, dim>::Zero();
    // F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    // F.block(3, 3 + 2*id, 2, 2) = Eigen::Matrix2d::Identity();

    Eigen::Matrix<double, 2, dim> H_low = Eigen::Matrix<double, 2, dim>::Zero();

    H_low.block(0, 0, 2, 3) << -q_sqrt * delta(0), -q_sqrt * delta(1), 0,
        delta(1), -delta(0), -q;

    H_low.block(0, landmark_idx, 2, 2) << q_sqrt * delta(0), q_sqrt * delta(1),
        -delta(1), delta(0);

    H_low *= 1 / q;

    H_t.block(ct * 2, 0, 2, dim) = H_low * 1 / q;
}

void EKF::CorrectStep(bool is_parallel)
{
    if (is_parallel)
        CorrectionStepParallel();
    else
        CorrectionStep();
}

void EKF::CorrectionStepParallel()
{

    int m = 2 * z_t.size();
    Eigen::MatrixXd H_t = Eigen::MatrixXd::Zero(m, dim);
    Eigen::VectorXd Z_diff = Eigen::VectorXd::Zero(m);

    vector<thread *> threads;
    int ct = 0;

    for (auto obs : z_t)
    {
        int id = obs.first;
        Eigen::Vector2d observation = obs.second;
        threads.push_back(new thread(&EKF::ComputeObservationH, this, ct, ref(id), ref(observation), ref(m), ref(H_t), ref(Z_diff)));
        ct++;
    }

    for (thread *t : threads)
    {
        t->join();
    }

    PerformUpdate(m, H_t, Z_diff);
}

void EKF::CorrectionStep()
{

    int m = 2 * z_t.size();
    Eigen::MatrixXd H_t = Eigen::MatrixXd::Zero(m, dim);
    Eigen::VectorXd Z_diff = Eigen::VectorXd::Zero(m);

    int ct = 0;

    for (auto obs : z_t)
    {
        int id = obs.first;
        Eigen::Vector2d observation = obs.second;
        ComputeObservationH(ct, id, observation, m, H_t, Z_diff);
        ct++;
    }

    PerformUpdate(m, H_t, Z_diff);
}

void EKF::PerformUpdate(int m, Eigen::MatrixXd &H_t, Eigen::VectorXd &Z_diff)
{
    Eigen::MatrixXd Q_t = Eigen::MatrixXd::Identity(m, m) * sensor_noise;

    Eigen::MatrixXd K_t(dim, m);
    // TODO: Test for non-inversible matrices
    K_t = sigma_t_pred * H_t.transpose() * (H_t * sigma_t_pred * H_t.transpose() + Q_t).inverse(); // size K_t: 3 + 2N x 5
    x_t_pred += K_t * Z_diff;                                                                      // size: 3 + 2N x 1
    x_t_pred(2) = ConstrainAngle(x_t_pred(2));

    sigma_t_pred = (Eigen::Matrix<double, dim, dim>::Identity() - K_t * H_t) * sigma_t_pred; // size: 3 + 2N x 3 + 2N
    x_t = x_t_pred;
    sigma_t = sigma_t_pred;
}


void EKF::MainLoop(string data, string world, bool parallel)
{

    // struct sigaction sa;
    // memset(&sa, 0, sizeof(sa));
    // sa.sa_handler = signalHandler;
    // sigfillset(&sa.sa_mask);
    // sigaction(SIGINT, &sa, NULL);

    // Read data.csv
    unordered_map<int, vector<string>> sensor_data = filereader->ReadFile(data);
    unordered_map<int, vector<string>> map_data = filereader->ReadFile(world);

    unordered_map<int, Eigen::Vector2d> lmMap = filereader->GetMap(map_data);

    // SLAM loop
    int data_ct = 0;
    cout << "Going over " << sensor_data.size() << " states" << endl;
    while (data_ct < sensor_data.size() - 1)
    {
        usleep(100000);

        u_t = filereader->GetInput(data_ct, sensor_data);
        z_t = filereader->GetObservations(data_ct, sensor_data);

        // Predict
        auto startP = chrono::system_clock::now();
        PredictionStep(parallel);
        auto endP = chrono::system_clock::now();
        chrono::duration<float, milli> durationP = endP - startP;
        cout << "Pred. t: " << durationP.count();
        // cout << "Predicted pose (prediction step): \n" << ekf->x_t_pred << endl;
        // cout << "Predicted sigma (prediction step): \n" << ekf->sigma_t_pred << endl;

        // drawer->DrawState(ekf->x_gt.block(0, 0, 3, 1), Eigen::Matrix3d::Zero(), false);
        // drawer->DrawLandmarks(ekf->map_gt, false);

        // Correct
        auto startC = chrono::system_clock::now();
        CorrectStep(parallel);
        auto endC = chrono::system_clock::now();
        chrono::duration<float, milli> durationC = endC - startC;
        cout << " Corr. t: " << durationC.count() << endl;

        // cout << "Predicted pose (correction step): \n" << ekf->x_t << endl;
        // cout << "Predicted sigma (correction step): \n" << ekf->sigma_t << endl;

        drawer->SetPoseCov(x_t.block(0, 0, 3, 1), sigma_t.block(0, 0, 2, 2));
        drawer->SetMap(lmMap, map_t); // draw map lm
        drawer->SetNewData(true);
        // cout << "Cov: \n" << ekf->sigma_t.block(0, 0, 3, 3) << endl;
        // if (quit.load())
        //     break; // exit normally after SIGINT

    }

    cout << "Closing EKF-SLAM..." << endl;
}