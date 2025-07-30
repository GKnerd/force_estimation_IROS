import argparse
import logging
from pathlib import Path
from joint_data_preprocessing.robot_joint_manager import RobotJointManager
from joint_data_preprocessing.json_helper import parse_json, save_data_to_json

def process_scenario(scenario_path: Path, plot: bool):
    joint_dir = scenario_path / "robot" / "joint_states"
    if not joint_dir.exists():
        print(f"[WARNING] Skipping {scenario_path.name}: joint_states directory not found.")
        return

    manager = RobotJointManager()
    manager.process_directory(joint_dir)

    output_data_dir = scenario_path / "external_force_estimates" / "joint_states"
    plot_dir_base = scenario_path / "external_force_estimates" / "plots"

    arm_joints = [f"panda_joint{i}" for i in range(1, 8)]
    for joint_name in arm_joints:
        if joint_name not in manager.managed_joints:
            print(f"[WARNING] {joint_name} not found. Skipping.")
            continue

        joint = manager.managed_joints[joint_name]
        joint_plot_dir = plot_dir_base / joint_name

        # 1. Pre-filter plots
        joint.plot_and_save_joint_data(joint, joint_plot_dir, "pre_filter", plot)
        joint.plot_and_save_frequency_spectrum(joint, joint.efforts, joint.timestamps, joint_plot_dir, "pre_filter", "effort", plot)
        joint.plot_and_save_frequency_spectrum(joint, joint.accelerations, joint.timestamps, joint_plot_dir, "pre_filter", "acceleration", plot)

        # 2. Apply filters
        joint.apply_low_pass_filter("efforts", cutoff=1.0, order=2)
        joint.apply_low_pass_filter("accelerations", cutoff=1.0, order=2)

        # 3. Post-filter plots
        joint.plot_and_save_frequency_spectrum(joint, joint.efforts, joint.timestamps, joint_plot_dir, "post_filter", "effort", plot)
        joint.plot_and_save_frequency_spectrum(joint, joint.accelerations, joint.timestamps, joint_plot_dir, "post_filter", "acceleration", plot)
        joint.plot_and_save_joint_data(joint, joint_plot_dir, "post_filter", plot)

        # 4. Save processed data
        joint.save_data(output_data_dir)

    print(f"[DONE] {scenario_path.name} → {output_data_dir.relative_to(Path.cwd())}\n")

    # === POST-PROCESSING: Aggregate joint vectors ===
    output_files = sorted([f for f in output_data_dir.glob("*.json") if f.name.endswith(".json")])
    if not output_files:
        print(f"[INFO] No joint JSON files found in {output_data_dir}")
        return

    json_dicts = [parse_json(file) for file in output_files]

    try:
        timestamps = json_dicts[0]["timestamps"]
        position_vectors = [d["positions"] for d in json_dicts]
        velocity_vectors = [d["velocities"] for d in json_dicts]
        effort_vectors = [d["efforts"] for d in json_dicts]
        acceleration_vectors = [d["accelerations"] for d in json_dicts]

        zipped_positions = list(zip(*position_vectors))
        zipped_velocities = list(zip(*velocity_vectors))
        zipped_efforts = list(zip(*effort_vectors))
        zipped_accelerations = list(zip(*acceleration_vectors))

        save_data_to_json(filepath= output_data_dir, 
                          filename = "aggregated_joint_states.json",
                          q= zipped_positions,
                          dq= zipped_velocities,
                          ddq= zipped_accelerations,
                          tau= zipped_efforts
                          )
        
        print(f"[AGGREGATED] Saved multi-joint data to {output_data_dir.relative_to(Path.cwd())}")

    except Exception as e:
        print(f"[ERROR] Aggregation failed: {e}")
