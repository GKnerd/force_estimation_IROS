import argparse
import logging
from pathlib import Path
from joint_data_preprocessing.batch_joint_data_processing import process_scenario
from output_directory_creation.create_output_dirs import create_estimate_dirs


def main():
    parser = argparse.ArgumentParser(description="Batch preprocess joint state data.")
    parser.add_argument("--no-plot", action="store_true", help="Disable plotting for speed.")
    parser.add_argument("--log", type=str, default="batch_joint_processing.log", help="Log file path.")
    args = parser.parse_args()

    # Assume this script lives in: joint_data_preprocessing/
    base_dir = Path.cwd() / "datasets"

    logging.basicConfig(
        filename=args.log,
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

    if not base_dir.exists():
        print(f"[ERROR] Dataset directory does not exist: {base_dir}")
        return

    # Discover scenarios
    scenario_dirs = [d for d in base_dir.iterdir() if d.is_dir()]
    print(f"scenario_dirs: {scenario_dirs}")
    if not scenario_dirs:
        print("[ERROR] No scenario directories found in datasets/")
        return

    print(f"[INFO] Found {len(scenario_dirs)} scenario(s)")

    for scenario_path in scenario_dirs:
        print(f"\n[INFO] Processing: {scenario_path.name}")

        # 1. Create output directory for this scenario
        create_estimate_dirs(scenario_path)

        # 2. Preprocess joint data with optional plotting
        process_scenario(scenario_path, plot=not args.no_plot)

    print("\n[INFO] All scenarios completed.")

if __name__ == "__main__":
    main()
