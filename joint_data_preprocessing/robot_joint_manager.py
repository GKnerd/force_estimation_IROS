import logging
from pathlib import Path
from typing import Dict, Union, List
from joint_data_preprocessing.robot_data_processor import RobotDataProcessor
from joint_data_preprocessing.robot_joint_data import RobotJointData
from joint_data_preprocessing.json_helper import parse_json, load_directory


class RobotJointManager:
    """Manages the processing and storage of robot joint data."""

    def __init__(self) -> None:
        """
        Initializes the RobotJointManager, which manages RobotJointData instances
        and the RobotDataProcessor.
        """
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)

        self.data_processor = RobotDataProcessor()  # Processes raw data
        self.managed_joints: Dict[str, RobotJointData] = {}  # Stores RobotJointData instances

    def initialize_joints(self) -> None:
        """Initializes RobotJointData objects based on the joint names from the processor."""
        if not self.managed_joints:  # Only initialize if it's empty
            for joint_name in self.data_processor.joint_names:
                self.managed_joints[joint_name] = RobotJointData(joint_name)
            self.logger.info(f"Initialized {len(self.managed_joints)} joints: {list(self.managed_joints.keys())}")

    def update_joint_data(self) -> None:
        """Updates the RobotJointData instances with the latest structured data from the processor."""
        structured_data = self.data_processor.structured_joint_data
        latest_timestamp = self.data_processor.current_timestamp[-1]  # Get latest timestamp

        if latest_timestamp is None or any(value is None for value in latest_timestamp):
            self.logger.warning("Latest timestamp is None, skipping update.")
            return
        
        # print(structured_data.items())
        latest_time = latest_timestamp[0] + latest_timestamp[1] * 1e-9  # Convert sec/nanosec to float
        for joint_name, joint_data in structured_data.items():
            if joint_name in self.managed_joints:
                joint_instance = self.managed_joints[joint_name]  # Get corresponding RobotJointData instance
                joint_instance.update_data(
                    timestamp=latest_time,
                    position=structured_data[joint_name]["position"][-1],
                    velocity=structured_data[joint_name]["velocity"][-1],
                    effort=structured_data[joint_name]["effort"][-1]
                )

        self.logger.info("Updated latest joint data using RobotJointData methods.")

    def process_file(self, json_file: Union[str, Path]) -> None:
        """Processes a single JSON file."""
        data = parse_json(json_file)
        self.logger.info(f"Processing file: {json_file}")

        self.data_processor.receive_robot_joint_state(data)
        self.data_processor.process_data()

        self.initialize_joints()
        self.update_joint_data()

    def process_directory(self, directory: Union[str, Path]) -> None:
        """Processes all JSON files in a directory."""
        files_data = load_directory(directory)
        self.logger.info(f"Processing directory: {directory}, found {len(files_data)} files.")

        for data in files_data:
            self.data_processor.receive_robot_joint_state(data)
            self.data_processor.process_data()

            self.initialize_joints()
            self.update_joint_data()
