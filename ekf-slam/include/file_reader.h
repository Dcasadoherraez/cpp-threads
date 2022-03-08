#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <memory>

#include <Eigen/Core>

using namespace std;

class FileReader
{
public:
    typedef shared_ptr<FileReader> Ptr;
    double scale = 0;

    FileReader() {}
    FileReader(double s) : scale(s) {}

    string ReadFileIntoString(const string &path);
    unordered_map<int, vector<string>> ReadFile(string filename);

    Eigen::Vector3d GetInput(int &ct, unordered_map<int, vector<string>> data);
    unordered_map<int, Eigen::Vector2d> GetObservations(int &ct, unordered_map<int, vector<string>> data);
    unordered_map<int, Eigen::Vector2d> GetMap(unordered_map<int, vector<string>> lmMap);

    Eigen::Vector3d GetUserInput();
    unordered_map<int, Eigen::Vector2d> GetUserObservations(int &N);
};