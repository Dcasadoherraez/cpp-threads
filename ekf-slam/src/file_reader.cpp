#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

#include <Eigen/Core>

#include "file_reader.h"

// Read entire file to a string
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

// Read an entire file and convert it into map with the form of {line_id: vector of words}
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

// Get the odometry input for the robot
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

// Get the sensor readings for the current state
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

// Get the environment map from file
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
