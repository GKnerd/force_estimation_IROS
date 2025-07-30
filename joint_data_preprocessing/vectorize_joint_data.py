from pathlib import Path
from joint_data_preprocessing.json_helper import parse_json, save_data_to_json


dir = Path("/home/georgios-katranis/Projects/IROS_Paper_2025/datasets/handover_danger/external_force_estimates/joint_states")
json_files = sorted([file for file in dir.iterdir() if file.name.endswith(".json")])
json_dicts = [parse_json(file) for file in json_files]

# Extract the positions from each dictionary
timestamps  = json_dicts[0]["timestamps"]
position_vectors = [dict["positions"] for dict in json_dicts]
velocity_vectors = [dict["velocities"] for dict in json_dicts]
effort_vectors = [dict["efforts"] for dict in json_dicts]
acceleration_vectors = [dict["accelerations"] for dict in json_dicts]

# Zip the positions together into a list of tuples
zipped_positions = list(zip(*position_vectors))
zipped_velocities = list(zip(*velocity_vectors))
zipped_efforts = list(zip(*effort_vectors))
zipped_accelerations = list(zip(*acceleration_vectors))

# Convert the zipped tuples into lists
zipped_positions_lists = [list(tup) for tup in zipped_positions]
zipped_velocities_lists = [list(tup) for tup in zipped_velocities]
zipped_efforts_lists = [list(tup) for tup in zipped_efforts]
zipped_accelerations_lists = [list(tup) for tup in zipped_accelerations]

# Now, the lists are JSON serializable
print(zipped_positions_lists)
print(zipped_velocities_lists)
print(zipped_efforts_lists)
print(zipped_accelerations_lists)

save_data_to_json(filepath=dir, filename="vectorial_joint_data.json",
                  q= zipped_positions_lists,
                  dq=zipped_velocities_lists,
                  ddq=zipped_accelerations_lists, 
                  tau=zipped_efforts_lists)
