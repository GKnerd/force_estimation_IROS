import logging
from typing import Dict, List, Mapping, Tuple

class RobotDataProcessor:
    """
    Processes robot joint data from JSON files.

    This class takes in raw joint data from a dictionary-like object,
    processes and structures it, and provides access to the restructured data.

    Attributes:
        unprocessed_data (Dict): Stores the latest received raw data.
        _joint_names (List[str]): List of joint names detected from the data.
        _current_timestamp (List[Tuple[float, float]]): List of timestamps extracted from the data.
        _structured_joint_data (Dict[str, Dict[str, List[float]]]):
            Dictionary containing structured data where each joint name maps to:
                - "position": List of position values over time.
                - "velocity": List of velocity values over time.
                - "effort": List of effort values over time.
    """

    def __init__(self):
        self.unprocessed_data: Dict = None
        self._joint_names: List[str] = None
        self._current_timestamp: List[Tuple[float, float]] = []
        self._structured_joint_data: Dict[str, Dict[str, List[float]]] = {}

        # Logging setup
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)
    
    @property
    def joint_names(self) -> List[str]:
        return self._joint_names
    @joint_names.setter
    def joint_names(self, value: List[str]):
        self.logger.info(f"Setting joint names: {value}")
        self._joint_names = value

    @property
    def current_timestamp(self) -> List[Tuple[float, float]]:
        return self._current_timestamp
    @current_timestamp.setter
    def current_timestamp(self, value: Tuple[float, float]):
        self.logger.info(f"Adding timestamp: {value}")
        self._current_timestamp.append(value)

    @property
    def structured_joint_data(self) -> Dict[str, Dict[str, List[float]]]:
        return self._structured_joint_data
    @structured_joint_data.setter
    def structured_joint_data(self, value: Dict[str, Dict[str, List[float]]]):
        self.logger.info("Updating structured joint data.")
        self._structured_joint_data = value
    
    def receive_robot_joint_state(self, data: Mapping) -> None:
        """
        Receives and stores robot joint data for further processing.

        Args:
            data (Mapping): A dictionary-like object containing robot joint data.

        Raises:
            ValueError: If the input data is not a dictionary-like object.
        """
        if not isinstance(data, Mapping):
            raise ValueError("Dict-like object expected.")
        
        self.logger.info("Received new robot joint state data.")
        self.unprocessed_data = data
    
    def extract_timestamp(self, data: Dict) -> None:
        """
        Extracts the timestamp from the given data and appends it to the timestamp list.

        Args:
            data (Dict): The input data containing timestamp information.
        """
        sec = self.find_key(d=data, key_path=["header", "stamp", "sec"])
        nanosec = self.find_key(d=data, key_path=["header", "stamp", "nanosec"])
        
        if sec is not None and nanosec is not None:
            self.current_timestamp = (sec, nanosec)
        else:
            self.logger.warning(f"Values for timestamp are {sec}, {nanosec}")
            self.current_timestamp = (None, None)
    
    def find_key(self, d: Mapping, key_path: List[str]):
        """
        Traverses a nested dictionary using a key path.
        
        Args:
            d (Mapping): The dictionary to traverse.
            key_path (List[str]): A list of keys representing the path to the desired value.
        
        Returns:
            Any: The value found at the key path or None if not found.
        """
        for key in key_path:
            if isinstance(d, Mapping) and key in d:
                d = d[key]
            else:
                self.logger.warning(f"Cannot find the values in the specified key path {key_path}")
                return None
        return d
    
    def structure_data(self, data: Mapping) -> Dict[str, Dict[str, List[float]]]:
        """
        Formats data to store lists of positions, velocities, and efforts for each joint.

        Args:
            data (Mapping): A dictionary containing joint states with position, velocity, and effort.

        Returns:
            Dict[str, Dict[str, List[float]]]: A structured dictionary with historical joint data.
        """
        if not self._structured_joint_data:
            self.logger.info("Initializing structured joint data storage.")
            self._structured_joint_data = {
                joint: {"position": [], "velocity": [], "effort": []} for joint in self.joint_names
            }
        
        for joint in self.joint_names:
            position = data["position"].get(joint, None)
            velocity = data["velocity"].get(joint, None)
            effort = data["effort"].get(joint, None)
            
            self._structured_joint_data[joint]["position"].append(position)
            self._structured_joint_data[joint]["velocity"].append(velocity)
            self._structured_joint_data[joint]["effort"].append(effort)
            
            self.logger.info(
                f"Updated joint {joint}: position={position}, velocity={velocity}, effort={effort}"
            )
        
        return self._structured_joint_data
    
    def process_data(self):
        """
        Processes the received data by extracting timestamps and structuring joint data.
        """
        if self.joint_names is None:
            self.joint_names = list(self.unprocessed_data["position"].keys())
        
        self.extract_timestamp(data=self.unprocessed_data)
        self.structured_joint_data = self.structure_data(data=self.unprocessed_data)
    
    # def run(self):
    #     """
    #     Placeholder run method. This class does not manage file handling.
    #     """
    #     self.logger.info("Starting Data Processing...")
    #     self.logger.info("Processing Complete.")
