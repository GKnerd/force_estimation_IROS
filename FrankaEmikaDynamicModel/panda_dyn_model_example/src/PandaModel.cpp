#include <iostream>
#include <filesystem>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <nlohmann/json.hpp> // JSON library
#include <franka_model.h> 

using json = nlohmann::json;
using namespace std;
namespace fs = std::filesystem;

int main()
{
    // Define the path to the JSON file
    fs::path file_path = "/home/giorgos_katranis/robot_data/joint_data/joint_values/joint_vectorial_data.json";

    // Open and parse the JSON file
    ifstream file(file_path);
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << file_path << endl;
        return EXIT_FAILURE;
    }

    json joint_data;
    file >> joint_data;
    file.close();

    // Check if the required fields exist
    if (!joint_data.contains("q") || !joint_data.contains("dq") || !joint_data.contains("ddq")) {
        cerr << "Error: Missing required fields (q, dq, ddq) in JSON file." << endl;
        return EXIT_FAILURE;
    }

    // Extract joint data
    vector<Eigen::Vector7d> q_values, dq_values, ddq_values;
    for (size_t i = 0; i < joint_data["q"].size(); ++i) {
        Eigen::Vector7d q, dq, ddq;

        for (size_t j = 0; j < 7; ++j) {
            q[j] = joint_data["q"][i][j];
            dq[j] = joint_data["dq"][i][j];
            ddq[j] = joint_data["ddq"][i][j];
        }

        q_values.push_back(q);
        dq_values.push_back(dq);
        ddq_values.push_back(ddq);
    }

    // Compute torques for each time step
    json output_data;
    for (size_t i = 0; i < q_values.size(); ++i) {
        Eigen::Matrix7d M = MassMatrix(q_values[i]);
        Eigen::Matrix7d C = CoriolisMatrix(q_values[i], dq_values[i]);
        Eigen::Vector7d G = GravityVector(q_values[i]);
        Eigen::Vector7d F = Friction(dq_values[i]);

        Eigen::Vector7d tau = M * ddq_values[i] + C * dq_values[i] + G + F;
        
        vector<double> tau_vector(7);
        for (int j = 0; j < 7; ++j) {
            tau_vector[j] = tau[j];
        }

        output_data["torques"].push_back(tau_vector);
    }

    // Save torques to JSON file
    ofstream torque_file("computed_torques.json");
    if (!torque_file.is_open()) {
        cerr << "Error: Could not create output file for torques." << endl;
        return EXIT_FAILURE;
    }
    
    torque_file << output_data.dump(4); // Pretty-print with indentation of 4 spaces
    torque_file.close();

    cout << "Computed torques saved to computed_torques.json" << endl;

    return EXIT_SUCCESS;
}
