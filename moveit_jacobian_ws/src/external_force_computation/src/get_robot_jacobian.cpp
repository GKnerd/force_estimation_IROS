// Libraries
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>  
#include <moveit/robot_state/robot_state.h>

#include <cstring>
#include <iostream>
#include <filesystem>
#include <memory>
#include <Eigen/Dense>
#include <fstream>
#include <nlohmann/json.hpp>


using json = nlohmann::json;
using moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("jacobian");

class JacobianProvider
{

    private:
        const std::string PLANNING_GROUP_MANIPULATOR;
        std::vector<double> joint_angles;

    public:
        std::string input_file;
        rclcpp::Node::SharedPtr node_;
        MoveGroupInterface move_group_arm; 
        JacobianProvider(const rclcpp::NodeOptions& options);

        void parse_JSON_data(const std::string& file);
        void get_robot_jacobian();
};

JacobianProvider::JacobianProvider(const rclcpp::NodeOptions& options)
: PLANNING_GROUP_MANIPULATOR("panda_arm"),
node_{std::make_shared<rclcpp::Node>("velocity_computation_node", options)},
move_group_arm(node_, PLANNING_GROUP_MANIPULATOR)
{
    node_->declare_parameter("robot_joint_states_file", "");
    node_->get_parameter("robot_joint_states_file", input_file);
}

void JacobianProvider::parse_JSON_data(const std::string& file)
{
    std::ifstream input_file(file);
    if (!input_file.is_open()) {   
        RCLCPP_ERROR(node_->get_logger(), "Could not open file: %s", file.c_str());
        return;
    }

    json json_data;
    input_file >> json_data;

    // Ensure "q" exists in JSON
    if (!json_data.contains("q") || !json_data["q"].is_array()) {
        // RCLCPP_ERROR(node_->get_logger(), "Invalid JSON format: missing 'q' array.");
        return;
    }

    joint_angles.clear();
    for (const auto& positions : json_data["q"]) {
        joint_angles.insert(joint_angles.end(), positions.begin(), positions.end());
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded %zu joint configurations.", json_data["q"].size());
}


void JacobianProvider::get_robot_jacobian()
{  
    moveit::core::RobotModelConstPtr robot_model = move_group_arm.getRobotModel();
    moveit::core::RobotState current_state(robot_model);
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP_MANIPULATOR);

    if (input_file.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No input file provided.");
        return;
    }

    parse_JSON_data(input_file); // Load data from JSON

    auto num_joints = joint_model_group->getVariableCount();
    auto num_configs = joint_angles.size() / num_joints;

    if (num_configs == 0) {
        RCLCPP_ERROR(node_->get_logger(), "No valid joint configurations found.");
        return;
    }

    json output_json;
    output_json["jacobians"] = json::array();  // Create an array in JSON


    for (unsigned long i = 0; i < num_configs; ++i) {
        std::vector<double> joint_positions(joint_angles.begin() + i * num_joints, 
                                            joint_angles.begin() + (i + 1) * num_joints);

        current_state.setJointGroupPositions(joint_model_group, joint_positions);
        
        Eigen::MatrixXd jacobian;
        current_state.getJacobian(joint_model_group, 
                                  current_state.getLinkModel(move_group_arm.getEndEffectorLink()), 
                                  Eigen::Vector3d::Zero(), 
                                  jacobian);
                
        // Convert Eigen matrix to JSON array
        json jacobian_json = json::array();
        for (int row = 0; row < jacobian.rows(); ++row) {
            json row_json = json::array();
            for (int col = 0; col < jacobian.cols(); ++col) {
                row_json.push_back(jacobian(row, col));
            }
            jacobian_json.push_back(row_json);
        }

        output_json["jacobians"].push_back(jacobian_json);
    }

    // Write to JSON file
    std::ofstream output_file("jacobian_matrices.json");
    if (!output_file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create output file.");
        return;
    }
    output_file << std::setw(4) << output_json << std::endl;  // Pretty-print JSON
    output_file.close();
    }


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<JacobianProvider>(options);
    
    // node->parse_JSON_data(node->input_file);
    node->get_robot_jacobian();

    rclcpp::shutdown();
    return 0;
}