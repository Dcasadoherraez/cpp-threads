#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

#include <Eigen/Core>

#include "file_reader.h"

string FileReader::ReadFileIntoString(const string& path) {
    auto ss = ostringstream{};
    ifstream input_file(path);
    if (!input_file.is_open()) {
        cerr << "Could not open the file - '"
             << path << "'" << endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    return ss.str();
}

unordered_map<int, vector<string>> FileReader::ReadFile(string filename) {
    string file_contents;
    unordered_map<int, vector<string>> csv_contents;
    char delimiter = ',';

    file_contents = ReadFileIntoString(filename);

    istringstream sstream(file_contents);
    vector<string> items;
    string record;

    int counter = 0;
    while (getline(sstream, record)) {
        istringstream line(record);
        while (getline(line, record, delimiter)) {
            items.push_back(record);
        }

        csv_contents[counter] = items;
        items.clear();
        counter += 1;
    }
    return csv_contents;
}

Eigen::Vector3d FileReader::GetInput(int &ct, unordered_map<int, vector<string>> data) {
    Eigen::Vector3d u_t;
    if (data[ct][0] == "ODOMETRY") {
        u_t[0] = stof(data[ct][3]) * scale; // v
        u_t[1] = stof(data[ct][2]); // w1
        u_t[2] = stof(data[ct][4]); // w2

        ct++;
    }

    return u_t;
}

unordered_map<int, Eigen::Vector2d> FileReader::GetObservations(int &ct, unordered_map<int, vector<string>> data) {
    unordered_map<int, Eigen::Vector2d> observationList;

    while (ct < data.size() - 1 && data[ct][0] == "SENSOR") {

        Eigen::Vector2d obs;
        int id = stoi(data[ct][1]);
        obs[0] = stof(data[ct][2]) * scale;
        obs[1] = stof(data[ct][3]);

        observationList[id] = obs;
        ct++;
    }
    return observationList;
}

unordered_map<int, Eigen::Vector2d> FileReader::GetMap(unordered_map<int, vector<string>> lmMap) {
    unordered_map<int, Eigen::Vector2d> landmarks;

    for (auto lm : lmMap) {
        Eigen::Vector2d pos;
        pos << stof(lm.second[1]) * scale,
               stof(lm.second[2]) * scale;
        landmarks[stoi(lm.second[0])] = pos;
    }

    return landmarks;
}

Eigen::Vector3d FileReader::GetUserInput() {
    cout << "Input robot command in deg. [v, w]: ";
    Eigen::Vector3d u_t;
    cin >> u_t[0];
    cin >> u_t[1];
    cin >> u_t[2];

    u_t[1] *= M_PI/180;
    u_t[2] *= M_PI/180;

    return u_t;
}

unordered_map<int, Eigen::Vector2d> FileReader::GetUserObservations(int &N) {
    unordered_map<int, Eigen::Vector2d> observationList;
    for (int i = 0; i < N; i++) {
        cout << "Input new observation, write -1 when done [id, r, phi]: ";
        Eigen::Vector2d obs;
        int id;
        cin >> id;
        if (id == -1) break;
        cin >> obs[0];
        cin >> obs[1];

        obs[1] *= M_PI/180;
        observationList[id] = obs;
    }

    return observationList;
}