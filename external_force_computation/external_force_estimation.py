import numpy as np
from pathlib import Path
from joint_data_preprocessing.json_helper import parse_json
import matplotlib.pyplot as plt
from scipy.linalg import pinv

def plot_force_magnitude(magnitude_array, label, color):
    plt.figure(figsize=(6, 3))
    plt.plot(magnitude_array, color=color, linewidth=1.2, label=label)
    plt.xlim(0, 1900)
    plt.ylim(-1, 50)
    plt.xlabel('Timestep', fontsize=11)
    plt.ylabel('External Force [N]', fontsize=11)
    plt.grid(True, linestyle='--', linewidth=0.5, alpha=0.7)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    plt.tight_layout()
    plt.show()


## NON DANGEROUS ##
no_danger_jacobian_dir = Path("/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/handover_danger/external_force_estimates/jacobians/jacobian_matrices.json")
no_danger_residual_torques = Path("/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/handover_danger/external_force_estimates/torque_values/torque_residuals.json")

no_danger_jacobian_dict = parse_json(no_danger_jacobian_dir)

## DANGEROUS ##
danger_jacobian_dir = Path("/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/handover_danger/external_force_estimates/jacobians/jacobian_matrices.json")
danger_residual_torques = Path("/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/handover_danger/external_force_estimates/torque_values/torque_residuals.json")

danger_jacobian_dict = parse_json(danger_jacobian_dir)

# Jacobians
no_danger_jacobian_matrix_list = np.array(no_danger_jacobian_dict["jacobians"]) #(1159 x 7 x 6)
print(no_danger_jacobian_matrix_list.shape)

# Compute pseudo-inverse for each Jacobian matrix
no_danger_pseudo_inv_list = np.array([pinv(jacobian.T) for jacobian in no_danger_jacobian_matrix_list])  # Shape: (1159, 7, 6)

# Print results
print("Pseudo-inverse matrices shape:", no_danger_pseudo_inv_list.shape)


# # Pseudoinverse
# inv_jacobian_matrix_list = np.array(jacobian_dict["inverse_Jacobian"]) #(1159 x 7x6)
# print(inv_jacobian_matrix_list.shape)

# Residuals
no_danger_torques_dict = parse_json(no_danger_residual_torques)
no_danger_residuals = np.array(no_danger_torques_dict["residuals"]) # (1159 x 7 x1 )
print(no_danger_residuals.shape)

# print("Residuals (first 5 entries):", residuals[:5])
# print(inv_jacobian_matrix_list[0])  # Check the first Jacobian
# print(residuals[0])  # Check the first residual
# print(np.matmul(inv_jacobian_matrix_list[0], residuals[0]))  


# Vectorized computation of external forces: F_ext = inv(J).T * tau
# inv_jacobian_matrix_list.T has shape (1159, 6, 7), and residuals has shape (1159, 7, 1)
no_danger_external_forces_list = []
# external_forces_compensated = []

# Loop through each time step and compute external forces
for i in range(len(no_danger_residuals)):  # Loop over the 1159 time steps
    # Compute external forces for the current time step using matrix multiplication
    no_danger_external_forces = np.matmul(no_danger_pseudo_inv_list[i], no_danger_residuals[i])  # Shape: (6, 1)
    no_danger_external_forces_list.append(no_danger_external_forces.flatten()[:3]) # Get the first 3 components

external_forces_uncompensated = np.array(no_danger_external_forces_list)
gravity_bias_estimate = np.mean(external_forces_uncompensated[:5], axis=0)
print("Estimated gravity bias:", gravity_bias_estimate)
no_danger_external_forces_compensated = external_forces_uncompensated - gravity_bias_estimate

# Convert the list of external forces to a numpy array
no_danger_external_forces_array = np.array(no_danger_external_forces_list)  # Shape: (1159, 3)
# Extract individual components
no_danger_force_x = no_danger_external_forces_array[:, 0]  # All rows, first column (x component)
no_danger_force_y = no_danger_external_forces_array[:, 1]  # All rows, second column (y component)
no_danger_force_z = no_danger_external_forces_array[:, 2]  # All rows, third column (z component)
# # Calculate the magnitudes of the external forces using Euclidean norm
no_danger_magnitudes = np.linalg.norm(no_danger_external_forces_array, axis=1)  # Shape: (1159,)
# print(magnitudes)


# # Plot the magnitudes
# plt.plot(magnitudes)
# plt.xlabel('Time Step')
# plt.ylabel('Magnitude of External Forces (N)')
# plt.title('Magnitude of External Forces Over Time')
# plt.show()

# Compensated
# Convert the list of external forces to a numpy array
no_danger_external_forces_compensated_array = np.array(no_danger_external_forces_compensated)  # Shape: (1159, 3)
# Extract individual components
no_danger_force_x = no_danger_external_forces_compensated_array[:, 0]  # All rows, first column (x component)
no_danger_force_y = no_danger_external_forces_compensated_array[:, 1]  # All rows, second column (y component)
no_danger_force_z = no_danger_external_forces_compensated_array[:, 2]  # All rows, third column (z component)
# # Calculate the magnitudes of the external forces using Euclidean norm
no_danger_compensated_magnitudes = np.linalg.norm(no_danger_external_forces_compensated_array, axis=1)  # Shape: (1159,)
# print(magnitudes)


# # Plot the magnitudes
# plt.plot(compensated_magnitudes)
# plt.xlabel('Time Step')
# plt.ylabel('Magnitude of External Forces (N)')
# plt.title('Magnitude of External Forces Over Time')
# plt.show()



# Plot 1: Uncompensated
plot_force_magnitude(no_danger_magnitudes, label="Uncompensated", color="tab:orange")

# Plot 2: Compensated
plot_force_magnitude(no_danger_compensated_magnitudes, label="Compensated", color="tab:blue")

