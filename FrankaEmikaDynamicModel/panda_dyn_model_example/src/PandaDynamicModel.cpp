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

string generate_unique_filename(const string& base_path, const string& extension) {
    // Get current time as a string (timestamp)
    auto now = chrono::system_clock::now();
    auto now_time = chrono::system_clock::to_time_t(now);
    stringstream ss;
    ss << base_path << "/computed_torques_" << now_time << extension;
    return ss.str();
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        cerr << "Error: Please provide the input JSON file as an argument." << endl;
        return EXIT_FAILURE;
    }

    // Define the path to the input file from the command line
    fs::path file_path = argv[1];

    // Check if the file exists
    if (!fs::exists(file_path)) {
        cerr << "Error: Input file does not exist: " << file_path << endl;
        return EXIT_FAILURE;
    }

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

    // Save torques to a unique JSON file in the specified directory
    string output_directory = "/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/coexistence/external_force_estimates/";
    if (!fs::exists(output_directory)) {
        fs::create_directories(output_directory);
    }

    string output_file = generate_unique_filename(output_directory, ".json");

    ofstream torque_file(output_file);
    if (!torque_file.is_open()) {
        cerr << "Error: Could not create output file for torques." << endl;
        return EXIT_FAILURE;
    }
    
    torque_file << output_data.dump(4); // Pretty-print with indentation of 4 spaces
    torque_file.close();

    cout << "Computed torques saved to " << output_file << endl;

    return EXIT_SUCCESS;
}
