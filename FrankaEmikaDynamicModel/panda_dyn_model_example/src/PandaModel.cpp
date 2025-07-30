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
    fs::path datasets_dir = fs::current_path() / "datasets";

    if (!fs::exists(datasets_dir) || !fs::is_directory(datasets_dir)) {
        cerr << "Error: datasets directory not found at " << datasets_dir << endl;
        return EXIT_FAILURE;
    }

    for (const auto& scenario_entry : fs::directory_iterator(datasets_dir)) {
        if (!scenario_entry.is_directory()) continue;

        fs::path scenario_path = scenario_entry.path();
        fs::path input_file = scenario_path / "external_force_estimates" / "joint_states" / "aggregated_joint_states.json";
        fs::path output_dir = scenario_path / "external_force_estimates"  / "torque_values";
        fs::path output_file = output_dir / "computed_torques.json";

        if (!fs::exists(input_file)) {
            cout << "[SKIP] " << scenario_path.filename() << ": No aggregated_joint_states.json found.\n";
            continue;
        }

        // Create output directory if it doesn't exist
        fs::create_directories(output_dir);

        // Open and parse the JSON file
        ifstream file(input_file);
        if (!file.is_open()) {
            cerr << "Error: Could not open file " << input_file << endl;
            continue;
        }

        json joint_data;
        file >> joint_data;
        file.close();

        // Check if the required fields exist
        if (!joint_data.contains("q") || !joint_data.contains("dq") || !joint_data.contains("ddq")) {
            cerr << "Error: Missing required fields (q, dq, ddq) in " << input_file << endl;
            continue;
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
        ofstream torque_file(output_file);
        if (!torque_file.is_open()) {
            cerr << "Error: Could not create output file " << output_file << endl;
            continue;
        }

        torque_file << output_data.dump(4); // Pretty-print with indentation of 4 spaces
        torque_file.close();

        cout << "[DONE] " << scenario_path.filename() << " → saved to " << output_file << endl;
    }

    return EXIT_SUCCESS;
}
