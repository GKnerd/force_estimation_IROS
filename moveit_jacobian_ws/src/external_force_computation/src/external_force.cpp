#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>

using json = nlohmann::json;
using moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("external_force_estimation");

class ForceEstimator
{
    private:
        std::string json_file_;

    public:
        rclcpp::Node::SharedPtr node_;
        MoveGroupInterface move_group_arm;
        // moveit::core::RobotModelPtr robot_model_;
        // moveit::core::RobotStatePtr robot_state_;

    ForceEstimator(const rclcpp::NodeOptions &options, const std::string &json_file);
    void computeEndEffectorForce();
};

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
}

ForceEstimator::ForceEstimator(const rclcpp::NodeOptions &options, const std::string &json_file)
    : node_(std::make_shared<rclcpp::Node>("force_estimator_node", options)),
      move_group_arm(node_, "panda_arm"),
      json_file_(json_file)
{
    robot_model_ = move_group_arm.getRobotModel();
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
}

void ForceEstimator::computeEndEffectorForce()
{
    std::ifstream file(json_file_);
    if (!file.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open file: %s", json_file_.c_str());
        return;
    }

    json data;
    file >> data;
    file.close();

    if (!data.contains("joint_positions") || !data.contains("external_torques"))
    {
        RCLCPP_ERROR(node_->get_logger(), "Invalid JSON format: missing required fields");
        return;
    }

    std::vector<double> joint_positions = data["joint_positions"];
    std::vector<double> external_torques = data["external_torques"];

    if (joint_positions.size() != 7 || external_torques.size() != 7)
    {
        RCLCPP_ERROR(node_->get_logger(), "Expected 7 values for joint positions and external torques");
        return;
    }

    robot_state_->setJointGroupPositions("panda_arm", joint_positions);
    robot_state_->update();

    Eigen::MatrixXd jacobian;
    robot_state_->getJacobian(robot_state_->getJointModelGroup("panda_arm"),
                              robot_state_->getLinkModel(move_group_arm.getEndEffectorLink()),
                              Eigen::Vector3d::Zero(), jacobian);

    Eigen::VectorXd tau(7);
    for (size_t i = 0; i < 7; ++i)
        tau(i) = external_torques[i];

    Eigen::VectorXd end_effector_force = pseudoInverse(jacobian.transpose()) * tau;

    RCLCPP_INFO(node_->get_logger(), "End-effector force: [%f, %f, %f, %f, %f, %f]",
                end_effector_force[0], end_effector_force[1], end_effector_force[2],
                end_effector_force[3], end_effector_force[4], end_effector_force[5]);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (argc < 2)
    {
        std::cerr << "Usage: force_estimator <json_file>" << std::endl;
        return 1;
    }

    std::string json_file = argv[1];
    rclcpp::NodeOptions options;
    auto estimator = std::make_shared<ForceEstimator>(options, json_file);
    estimator->computeEndEffectorForce();
    
    rclcpp::shutdown();
    return 0;
}
