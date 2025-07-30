import os
from pathlib import Path


# Detect datasets directory relative to script execution location
BASE_DIR = Path.cwd() / "datasets"
TARGET_DIRNAME = "external_force_estimates"
MAX_VERSIONS = 99

# Subfolders inside each scenario’s output dir
SUBFOLDERS = ["external_forces", "jacobians", "joint_states", "torque_values"]

import os
from pathlib import Path

# Detect datasets directory relative to script execution location
BASE_DIR = Path.cwd() / "datasets"
TARGET_DIRNAME = "external_force_estimates"
MAX_VERSIONS = 99

# Subfolders inside each scenario’s output dir
SUBFOLDERS = ["external_forces", "jacobians", "joint_states", "torque_values"]

def get_available_target(base_path: Path, name: str, max_versions: int = 99) -> Path:
    """
    Returns a non-existing directory like external_force_estimates, or
    external_force_estimates_2, ..., up to max_versions.
    """
    candidate = base_path / name
    if not candidate.exists():
        return candidate

    for i in range(2, max_versions + 1):
        candidate = base_path / f"{name}_{i}"
        if not candidate.exists():
            return candidate

    raise RuntimeError(f"All folder names '{name}_2' to '_{max_versions}' are taken in {base_path}")

def create_estimate_dirs(scenario_path: Path):
    if not scenario_path.is_dir():
        print(f"[SKIP] {scenario_path} is not a directory.")
        return

    target_dir = get_available_target(scenario_path, TARGET_DIRNAME)

    for sub in SUBFOLDERS:
        sub_dir = target_dir / sub
        sub_dir.mkdir(parents=True, exist_ok=False)

    print(f"[OK] Created: {target_dir.relative_to(Path.cwd())}")

# def test_main():
#     if not BASE_DIR.exists():
#         print(f"[ERROR] 'datasets/' folder not found in {Path.cwd()}")
#         return

#     for scenario in sorted(BASE_DIR.iterdir()):
#         if scenario.is_dir():
#             create_estimate_dirs(scenario)

# if __name__ == "__main__":
#     test_main()
