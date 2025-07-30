## Force Estimation ##

A semi-automated version of the code. 

## Prerequisites

1) To successfully run the scripts the directory structure needs to be as described below. In the root of the cloned project
the labels need to be added to the project. It should look like this:

/project_root
    /datasets
    --> coexistence
        --> lidar/labels
        --> robot/joint_states
    --> coexistence_danger
        --> lidar/labels
        --> robot/joint_states
    --> collaboration
        --> lidar/labels
        --> robot/joint_states
    --> collaboration_danger
        --> lidar/labels
        --> robot/joint_states
    --> handover
        --> lidar/labels
        --> robot/joint_states
    --> handover_danger
        --> lidar/labels
        --> robot/joint_states
    /rest_of_the cloned_project


2) The FrankaEmikaDynamicModel needs to be build. See the README.md in the subdirectory for more details on how to build and what is needed.

3) The Dockerfile under `moveit2_humble_docker` needs to be built via `build_image.sh`

4) It is advised to create a virtual environment, where the packages from `requirements.txt` file are imported.


## Data Preprocessing

from CLI run the `main.py` file with the cli argument `--no-plot` to skip plotting. The command is `python3 main.py --no-plot`
This creates added directories in the datasets file, it looks like this (example on one scenario):

Creation of subdirs to save data into:

--> coexistence/
    --> external_force_estimates/
        --> external_forces
        --> jacobians
        --> joint_states
        --> torque_values
    --> lidar/labels
    --> robot/joint_states

Once this has been run successfully the joint data for all joints all scenarios should be readily available. 

## Computation of Model Torques

from cli run:

```bash

cd ~/<project_root>

./FrankaEmikaDynamicModel/build/Panda_Model
```

this creates the necessary model torque values.

## Computation of Jacobians

Here it gets a bit tedious.

```bash
./moveit2_humble_docker/run_moveit2_humble_container.sh
```

Be careful the name of the mounted directories need to reflect the local file paths of the host machine

Inside the container:

```bash
cd moveit_jacobian_ws

colcon build --symlink-install

source install/setup.sh
```

To launch the jacobian computation use:


```bash
ros2 launch external_force_computation jacobian_comp.launch.py
```
tedious task

inside the launch file the path to the scenarion can be specified under the path:

`<host_machine_path>/moveit_jacobian_ws/src/external_force_computation/launch/jacobian_comp.launch.py`

run the node. once computation is finished shut it down with ctr+c. The jacobian_matrices.json file should be in the root directory of the project. Move it manually (apologies) to  <path_to_scenario>/external_force_estimates/jacobians.

Repeat this step for all scenarios. 

## Residuals

Navigate to /external_force_computation dir.

Run residuals.py to get the residuals

## External Force computation


run either external_force_comparison.py or external_force estimation.py

YOU NEED to modify the dirs by hand. also apologies for that. 

In the end it plots the external forces.

# Force Estimation – Semi-Automated Pipeline

This project provides a semi-automated workflow to compute external forces based on joint states and model dynamics for various human-robot interaction scenarios.

---

## 📁 Directory Structure (Required)

Ensure that the following directory structure exists **at the root** of the cloned project:

```
/project_root
│
├── datasets/
│   ├── coexistence/
│   │   ├── lidar/labels/
│   │   └── robot/joint_states/
│   ├── coexistence_danger/
│   ├── collaboration/
│   ├── collaboration_danger/
│   ├── handover/
│   └── handover_danger/
│
├── FrankaEmikaDynamicModel/
├── moveit2_humble_docker/
├── requirements.txt
└── rest_of_the_project_files...
```

⚠️ **Make sure all scenario directories contain both:**
- `lidar/labels`
- `robot/joint_states`

---

## 🛠️ Prerequisites

1. **Build the Franka Emika Dynamic Model**  
   Navigate to the `FrankaEmikaDynamicModel` directory and follow its internal `README.md` for build instructions.

2. **Build Docker Image**  
   Run the following command from the root directory:
   ```bash
   ./moveit2_humble_docker/build_image.sh
   ```

3. **Set up a Python virtual environment**  
   It's recommended to use a venv:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

---

## Joint Data Preprocessing

Run from CLI:
```bash
python3 main.py --no-plot
```

This creates additional subdirectories per scenario:
```
datasets/
└── <scenario>/
    └── external_force_estimates/
        ├── external_forces/
        ├── jacobians/
        ├── joint_states/
        └── torque_values/
```

After this step, all joint data is prepared for all scenarios.

---

## Model Torque Computation

Run the C++ torque computation binary:
```bash
cd FrankaEmikaDynamicModel/build
./Panda_Model
```

This generates `computed_torques.json` for each scenario.

---

## Jacobian Matrix Computation

### 1. Launch the Docker container
```bash
./moveit2_humble_docker/run_moveit2_humble_container.sh
```

🔴 **Important:**  
Ensure the host directory paths match inside the container. The dataset directory must be correctly mounted.

### 2. Inside the container:
```bash
cd moveit_jacobian_ws
colcon build --symlink-install
source install/setup.sh
```

### 3. Launch the Jacobian computation
Edit:
`moveit_jacobian_ws/src/external_force_computation/launch/jacobian_comp.launch.py`
to reflect the correct path to the scenario.

Then launch:
```bash
ros2 launch external_force_computation jacobian_comp.launch.py
```

🔴 **After computation ends (Ctrl+C):**  
The `jacobian_matrices.json` will be saved in the root directory.  
Move it **manually** to:
```
<scenario_path>/external_force_estimates/jacobians/
```

Repeat for each scenario. (Sorry dafür)

---

## 🔍 Residual Computation

From the root directory:
```bash
cd external_force_computation
python3 residuals.py
```

Residuals are saved to:
```
<scenario>/external_force_estimates/torque_values/torque_residuals.json
```

---

## External Force Estimation

Two alternatives exist:
- `external_force_estimation.py`
- `external_force_comparison.py`

🟡 **Note:**  
You must **manually update paths** inside these scripts before running.  


---

##  Summary

| Step | Description | Manual Effort? |
|------|-------------|----------------|
| ✅ Joint preprocessing | `python3 main.py` | No |
| ✅ Torque computation | `./Panda_Model` | No |
| ⚠️ Jacobian computation | `ros2 launch ...` | **Yes** (paths, move files) |
| ✅ Residuals | `residuals.py` | No |
| ⚠️ Force Estimation | Modify paths manually | **Yes** |

---




