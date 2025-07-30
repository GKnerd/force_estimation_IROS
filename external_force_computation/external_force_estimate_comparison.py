import numpy as np
from pathlib import Path
from json_helper import parse_json
import matplotlib.pyplot as plt
from scipy.linalg import pinv

def plot_overlayed_magnitudes(data_dict):
    plt.figure(figsize=(7, 3))
    for label, (magnitudes, color) in data_dict.items():
        plt.plot(magnitudes, linewidth=1.2, label=label, color=color)
    plt.xlim(0, 1900)
    plt.ylim(-1, 50)
    plt.xlabel('Timestep', fontsize=11)
    plt.ylabel('External Force [N]', fontsize=11)
    plt.grid(True, linestyle='--', linewidth=0.5, alpha=0.7)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    plt.legend(fontsize=9)
    plt.tight_layout()
    plt.show()

def compute_force_magnitudes(jacobian_path: Path, residuals_path: Path):
    jacobian_dict = parse_json(jacobian_path)
    residuals_dict = parse_json(residuals_path)

    jacobians = np.array(jacobian_dict["jacobians"])  # (T x 7 x 6)
    residuals = np.array(residuals_dict["residuals"])  # (T x 7 x 1)

    # Compute pseudoinverses
    pseudo_inverses = np.array([pinv(jacobian.T) for jacobian in jacobians])  # (T x 7 x 6)

    # External force estimation
    ext_forces = np.array([
        np.matmul(pseudo_inverses[i], residuals[i]).flatten()[:3]
        for i in range(len(residuals))
    ])
    magnitudes_uncompensated = np.linalg.norm(ext_forces, axis=1)

    # Gravity bias estimation from first 5 steps
    gravity_bias = np.mean(ext_forces[:5], axis=0)
    ext_forces_compensated = ext_forces - gravity_bias
    magnitudes_compensated = np.linalg.norm(ext_forces_compensated, axis=1)

    return magnitudes_uncompensated, magnitudes_compensated

# === Define paths ===

non_danger_jacobian_path = Path("/home/georgios-katranis/IROS_force_estimation/datasets/collaboration/external_force_estimates/jacobians/jacobian_matrices.json")
non_danger_residuals_path = Path("/home/georgios-katranis/IROS_force_estimation/datasets/collaboration/external_force_estimates/torque_values/torque_residuals.json")

danger_jacobian_path = Path("/home/georgios-katranis/IROS_force_estimation/datasets/collaboration_danger/external_force_estimates/jacobians/jacobian_matrices.json")
danger_residuals_path = Path("/home/georgios-katranis/IROS_force_estimation/datasets/collaboration_danger/external_force_estimates/torque_values/torque_residuals.json")

# === Compute data ===
non_danger_uncomp, non_danger_comp = compute_force_magnitudes(non_danger_jacobian_path, non_danger_residuals_path)
danger_uncomp, danger_comp = compute_force_magnitudes(danger_jacobian_path, danger_residuals_path)

# === Plot overlayed results ===
plot_overlayed_magnitudes({
    "Non-Dangerous (Compensated)":   (non_danger_comp, "tab:blue"),
    "Dangerous (Compensated)":       (danger_comp, "tab:green")
})
