import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import json
from json_helper import parse_json


# --- Config ---
SHOW_PLOTS = False  # Set True to display plots interactively

# --- Paths ---
project_root = Path.cwd()
datasets_dir = project_root / "datasets"

for scenario_dir in datasets_dir.iterdir():
    if not scenario_dir.is_dir():
        continue
    
    scenario_name = scenario_dir.name
    ext_force_dir = scenario_dir / "external_force_estimates"
    joint_states_file = ext_force_dir / "joint_states" / "aggregated_joint_states.json"
    torque_values_dir = ext_force_dir / "torque_values"
    modeled_torque_file = torque_values_dir / f"computed_torques.json"
    residuals_file = torque_values_dir / "torque_residuals.json"
    residuals_plot_file = torque_values_dir / "torque_residuals_plot.png"

    # Check input files
    if not joint_states_file.exists() or not modeled_torque_file.exists():
        print(f"[SKIP] {scenario_name}: Required input files not found.")
        continue

    print(f"[PROCESSING] {scenario_name}")

    # Load torques
    torques_measured_dict = parse_json(joint_states_file)
    torques_modeled_dict = parse_json(modeled_torque_file)

    torques_measured = np.array(torques_measured_dict["tau"])
    torques_modeled = np.array(torques_modeled_dict["torques"])

    # Shape correction
    if torques_measured.shape[1] != 7:
        torques_measured = torques_measured.T
    if torques_modeled.shape[1] != 7:
        torques_modeled = torques_modeled.T

    # Residuals
    residuals = torques_measured - torques_modeled
    print(f"Residuals shape: {residuals.shape}")

    # Save residuals to JSON
    residuals_dict = {"residuals": residuals.tolist()}
    with open(residuals_file, "w") as f:
        json.dump(residuals_dict, f, indent=4)

    print(f"[SAVED] Residuals → {residuals_file}")

    # --- Save Residual Plot ---
    plt.figure(figsize=(12, 8))
    for i in range(7):
        plt.subplot(4, 2, i+1)
        plt.plot(residuals[:, i], label=f'Joint {i+1}')
        plt.xlabel("Time Step")
        plt.ylabel("Torque Residual")
        plt.title(f"Joint {i+1}")
        plt.ylim(0, 10)
        plt.grid(True)
        plt.legend()

    plt.tight_layout()
    plt.savefig(residuals_plot_file)
    print(f"[SAVED] Residual plot → {residuals_plot_file}")

    if SHOW_PLOTS:
        plt.show()

    plt.close()
