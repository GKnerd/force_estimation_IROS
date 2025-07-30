import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from joint_data_preprocessing.json_helper import parse_json
import json


#JSON Files
torques_modeled_file = Path("/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/handover_danger/external_force_estimates/torque_values/computed_joint_torques_handover_danger.json")
torques_measured_file = Path("/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/handover_danger/external_force_estimates/joint_states/vectorial_joint_data.json")

# Output JSON file for residuals
residuals_file = Path("/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/handover_danger/external_force_estimates/torque_values/torque_residuals.json")

# JSON Dicts
torques_measured_dict = parse_json(torques_measured_file)
torques_modeled_dict = parse_json(torques_modeled_file)

# torques_measured = torques_measured_dict["tau"]
# torques_modeled = torques_modeled_dict["torques"]

# print(f"Length of measured torque list: {len(torques_measured)} and \n Length of modeled torques list: {len(torques_modeled)}")
# # print(f"Measured torque list: {torques_measured} and \n Modeled torques list: {torques_modeled}")

# # Extract torque data
torques_measured = np.array(torques_measured_dict["tau"])# Shape (7, N)
torques_modeled = np.array(torques_modeled_dict["torques"])  # Shape (7, N)

# Compute residuals
residuals = torques_measured - torques_modeled
# Print shape to confirm
print(f"Residuals shape: {residuals.shape}")  

# Save residuals to JSON file (preserving shape)
residuals_dict = {"residuals": residuals.tolist()}  # Convert NumPy array to list
with open(residuals_file, "w") as f:
    json.dump(residuals_dict, f, indent=4)


# Ensure the shape is correct (reshape if necessary)
if residuals.shape[0] != 7:
    residuals = residuals.T  # Transpose if needed
    print(f"Transposed residuals shape: {residuals.shape}")  # Should now be (7, 2516)

# Plot residuals
plt.figure(figsize=(12, 8))
joint_labels = [f'Joint {i+1}' for i in range(7)]

for i in range(7):
    plt.subplot(4, 2, i+1)
    plt.plot(residuals[i], label=f'Residuals Joint {i+1}')
    plt.ylim(0,10)
    plt.xlabel('Time Step')
    plt.ylabel('Torque Residual')
    plt.legend()
    plt.grid()

plt.tight_layout()
plt.show()
