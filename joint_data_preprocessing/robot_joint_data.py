import logging
from pathlib import Path
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt
import numpy as np
from scipy.fft import fft, ifft, fftfreq
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt
from joint_data_preprocessing.json_helper import save_data_to_json

class RobotJointData:
    """
    Stores and processes joint data, including position, velocity, effort, and acceleration.
    Provides methods for updating data, computing acceleration via Fourier transform,
    and visualizing joint data.
    """
    
    def __init__(self, joint_name: str):
        """Initializes storage for the joint's data."""
        self.joint_name = joint_name
        self._timestamps: List[float] = []
        self._positions: List[float] = []
        self._velocities: List[float] = []
        self._efforts: List[float] = []
        self._accelerations: List[float] = []  # Computed acceleration values
        
        # Logger setup
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)

    @property
    def timestamps(self):
        return self._timestamps
    @timestamps.setter
    def timestamps(self, value):
        self._timestamps = value
    @property
    def positions(self):
        return self._positions
    @positions.setter
    def positions(self,value):
        self._positions = value
    @property
    def velocities(self):
        return self._velocities
    @velocities.setter
    def velocities(self,value):
        self._velocities = value
    @property
    def efforts(self):
        return self._efforts
    @efforts.setter
    def efforts(self,value):
        self._efforts = value
    @property
    def accelerations(self):
        return self._accelerations
    @accelerations.setter
    def accelerations(self,value):
        self._accelerations = value
    
    def to_dict(self):
        """Returns the joint data as a dictionary with numpy floats converted to Python floats."""
        def convert_numpy(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, np.generic):  # Handles np.float32, np.float64, np.int32, etc.
                return obj.item()
            return obj
        
        return {
            "joint_name": self.joint_name,
            "timestamps": [convert_numpy(t) for t in self.timestamps],
            "positions": [convert_numpy(p) for p in self.positions],
            "velocities": [convert_numpy(v) for v in self.velocities],
            "efforts": [convert_numpy(e) for e in self.efforts],
            "accelerations": [convert_numpy(a) for a in self.accelerations]
        }

    def update_data(self, timestamp: float, position: float, velocity: float, effort: float):
        """
        Updates the stored values with new data.

        Args:
            timestamp (float): Time at which the data was recorded.
            position (float): Joint position value.
            velocity (float): Joint velocity value.
            effort (float): Joint effort value.
        """
        self.timestamps.append(timestamp)
        self.positions.append(position)
        self.velocities.append(velocity)
        self.efforts.append(effort)
        self.logger.info(f"Updated data for {self.joint_name} at timestamp {timestamp}.")


    def evenly_interpolate_timestamps(self, uneven_timestamps: List[float], target_length: int) -> List[float]:
        """
        Interpolates a list of unevenly spaced timestamps to a new list of evenly spaced timestamps 
        with the exact desired length.
        This method ensures that the output list contains exactly `target_length` timestamps, 
        distributed evenly within the original time range, while maintaining the characteristics 
        of the original timestamps through linear interpolation.

        Args:
            uneven_timestamps (List[float]): A list of timestamps that are unevenly spaced.
                                            The timestamps should be in ascending order.
                                            If the list contains fewer than two timestamps,
                                            no interpolation will be performed.
            target_length (int): The desired number of evenly spaced timestamps in the output list.
                                This number must be greater than or equal to 2.

        Returns:
            List[float]: A list of `target_length` evenly spaced timestamps, 
                        interpolated from the original uneven timestamps.

        Raises:
            ValueError: If `target_length` is less than 2, an error will be raised, as a minimum 
                        of two timestamps are needed to perform interpolation.
        """
        if len(uneven_timestamps) < 2:
            return uneven_timestamps  # No interpolation needed if there's no or just one timestamp

        # Create a target time vector with evenly spaced values
        start_time = min(uneven_timestamps)
        end_time = max(uneven_timestamps)
        
        # Create the new target timestamps with the exact target length
        target_timestamps = np.linspace(start_time, end_time, target_length)
        
        # Create an interpolation function based on the original timestamps
        interpolation_function = interp1d(uneven_timestamps, uneven_timestamps, kind='linear', fill_value="extrapolate")
        
        return interpolation_function(target_timestamps)
    

    def compute_acceleration_via_fourier(self):
        """
        Computes acceleration from velocity using FFT and differentiation in the frequency domain.
        Returns a list of computed acceleration values.

        Returns:
            List[float]: Computed acceleration values.
        """
        if len(self.velocities) < 2:
            self.logger.warning(f"Not enough data to compute acceleration for {self.joint_name}.")
            return None
        
        # Interpolate timestamps to ensure they are evenly spaced and save to timestamps
        self.timestamps = self.evenly_interpolate_timestamps(self.timestamps, len(self.timestamps))

        # Compute time step
        time_diffs = np.diff(self.timestamps)
        if np.any(time_diffs <= 0):
            self.logger.error("Timestamps are not strictly increasing. Check data collection.")
            return None
        
        dt = np.mean(time_diffs)  # Average time step
        fs = 1/dt # sampling frequency
        velocity_signal = np.array(self.velocities)

        # Apply Butterworth LP to the velocity signal
        velocity_signal = self.butter_lowpass_filter(velocity_signal,cutoff=2.0, fs=fs,order= 4)

        # Compute FFT
        N = len(velocity_signal)
        freq = fftfreq(N, d=dt)  # Frequency bins
        velocity_fft = fft(velocity_signal)

        # Differentiate in frequency domain: acceleration = jω * velocity
        omega = 2j * np.pi * freq  # jω term
        acceleration_fft = omega * velocity_fft

        # Compute inverse FFT to get acceleration in time domain
        acceleration_signal = ifft(acceleration_fft).real
        # Apply Butterworth low-pass filter
        acceleration_signal = self.butter_lowpass_filter(acceleration_signal,cutoff=1.0, fs=fs,order= 2)

        # Store computed acceleration
        self.accelerations = acceleration_signal.tolist()
        self.logger.info(f"Computed acceleration for {self.joint_name} using Fourier transform.")
        
        return self.accelerations
    
    def save_plot(self, path: Path, show: bool):
        """Save current matplotlib figure to path and optionally show it."""
        path.parent.mkdir(parents=True, exist_ok=True)
        plt.tight_layout()
        plt.savefig(path)
        if show:
            plt.show()
        plt.close()

    def plot_and_save_joint_data(self, joint, save_dir: Path, stage: str, show: bool):
        """Plots and saves position/velocity/effort/acceleration over time."""
        if len(joint.timestamps) < 2:
            joint.logger.warning(f"Not enough data to plot for {joint.joint_name}.")
            return

        fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)

        axs[0].plot(joint.timestamps, joint.positions, label="Position", color="b")
        axs[0].set_ylabel("Position")
        axs[0].legend()

        axs[1].plot(joint.timestamps, joint.velocities, label="Velocity", color="g")
        axs[1].set_ylabel("Velocity")
        axs[1].legend()

        axs[2].plot(joint.timestamps, joint.efforts, label="Effort", color="r")
        axs[2].set_ylabel("Effort")
        axs[2].legend()

        if not joint.accelerations:
            joint.compute_acceleration_via_fourier()

        if joint.accelerations:
            axs[3].plot(joint.timestamps, joint.accelerations, label="Acceleration (FFT)", color="m")
            axs[3].set_ylabel("Acceleration")
            axs[3].legend()

        axs[3].set_xlabel("Time (s)")
        plt.suptitle(f"{stage} Joint Data for {joint.joint_name}")

        save_path = save_dir / f"{stage}_joint_data.png"
        self.save_plot(save_path, show)

    def plot_and_save_frequency_spectrum(self, joint, signal, timestamps, save_dir: Path, stage: str, signal_type: str, show: bool):
        """Plots and saves frequency spectrum for a signal."""
        interp_timestamps = joint.evenly_interpolate_timestamps(timestamps, len(timestamps))
        time_diffs = np.diff(interp_timestamps)
        if np.any(time_diffs <= 0):
            joint.logger.error("Timestamps are not strictly increasing.")
            return
        dt = np.mean(time_diffs)
        N = len(signal)

    def plot_position_data(self):
        """Plots only position data over time."""
        if len(self.timestamps) < 2:
            self.logger.warning(f"Not enough data to plot position for {self.joint_name}.")
            return
        
        plt.figure(figsize=(8, 4))
        plt.plot(self.timestamps, self.positions, label="Position", color="b")
        plt.xlabel("Time (s)")
        plt.ylabel("Position")
        plt.title(f"Position Data for {self.joint_name}")
        plt.legend()
        plt.show()

    def plot_velocity_data(self):
        """Plots only velocity data over time."""
        if len(self.timestamps) < 2:
            self.logger.warning(f"Not enough data to plot velocity for {self.joint_name}.")
            return
        
        plt.figure(figsize=(8, 4))
        plt.plot(self.timestamps, self.velocities, label="Velocity", color="g")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity")
        plt.title(f"Velocity Data for {self.joint_name}")
        plt.legend()
        plt.show()
    
    def plot_effort_data(self):
        """Plots effort data over time."""
        if len(self.timestamps) < 2:
            self.logger.warning(f"Not enough data to plot effort and acceleration for {self.joint_name}.")
            return
        
        plt.figure(figsize=(8, 4))
        plt.plot(self.timestamps, self.efforts, label="Effort", color="r")
        plt.xlabel("Time (s)")
        plt.ylabel("Joint Torque")
        plt.title(f"Torque Data for {self.joint_name}")
        plt.legend()
        plt.show()
    
    def save_data(self, filepath:str):
        """
        Saves joint data to a JSON file using the custom save_data_to_json function.

        Args:
            filepath (str): The directory where the JSON file will be saved.
        """
        filename = f"{self.joint_name}_data.json"
        save_data_to_json(filepath, filename, **self.to_dict())
        self.logger.info(f"Joint data for {self.joint_name} saved to {filepath}/{filename}.")


    def butter_lowpass_filter(self, data, cutoff, fs, order=4):
        """
        Apply a Butterworth low-pass filter to the data.

        Args:
            data (np.array): The input signal.
            cutoff (float): The cutoff frequency in Hz.
            fs (float): The sampling frequency in Hz.
            order (int): The order of the Butterworth filter.

        Returns:
            np.array: The filtered signal.
        """
        nyquist = 0.5 * fs  # Nyquist frequency
        normal_cutoff = cutoff / nyquist  # Normalize cutoff frequency
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return filtfilt(b, a, data)  # Apply filter with zero-phase delay

    def apply_low_pass_filter(self, signal_name: str, cutoff: float, order: int = 4):
        """
        Applies a Butterworth low-pass filter to the specified signal.

        Args:
            signal_name (str): The name of the signal to filter ("positions", "velocities", "efforts", or "accelerations").
            cutoff (float): The cutoff frequency in Hz.
            order (int, optional): The order of the Butterworth filter. Defaults to 4.

        Raises:
            ValueError: If the signal name is invalid or contains insufficient data.
        """
        valid_signals = {
            "positions": self.positions,
            "velocities": self.velocities,
            "efforts": self.efforts,
            "accelerations": self.accelerations
        }

        if signal_name not in valid_signals:
            raise ValueError(f"Invalid signal name '{signal_name}'. Choose from {list(valid_signals.keys())}.")

        data = valid_signals[signal_name]

        if len(data) < 2:
            self.logger.warning(f"Not enough data to filter {signal_name} for {self.joint_name}.")
            return None

        # Ensure timestamps are evenly spaced before filtering
        self.timestamps = self.evenly_interpolate_timestamps(self.timestamps, len(self.timestamps))
        
        # Compute sampling frequency
        time_diffs = np.diff(self.timestamps)
        if np.any(time_diffs <= 0):
            self.logger.error("Timestamps are not strictly increasing. Check data collection.")
            return None

        fs = 1 / np.mean(time_diffs)  # Sampling frequency

        # Apply Butterworth low-pass filter
        filtered_data = self.butter_lowpass_filter(np.array(data), cutoff=cutoff, fs=fs, order=order)

        # Store the filtered data back in the respective attribute
        if signal_name == "positions":
            self.positions = filtered_data.tolist()
        elif signal_name == "velocities":
            self.velocities = filtered_data.tolist()
        elif signal_name == "efforts":
            self.efforts = filtered_data.tolist()
        elif signal_name == "accelerations":
            self.accelerations = filtered_data.tolist()

        self.logger.info(f"Applied low-pass filter (cutoff={cutoff} Hz, order={order}) to {signal_name} for {self.joint_name}.")
