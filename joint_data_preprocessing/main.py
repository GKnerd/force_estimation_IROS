import argparse
import logging
from pathlib import Path
from robot_joint_manager import RobotJointManager

# Configure logging to write to a file instead of the console
log_file = "robot_joint_manager.log"
logging.basicConfig(
    filename=log_file,  # Log file path
    level=logging.INFO,  # Set logging level
    format="%(asctime)s - %(levelname)s - %(message)s",  # Format logs
    datefmt="%Y-%m-%d %H:%M:%S"  # Date format
)

def main():
    parser = argparse.ArgumentParser(description="Interactive CLI for Robot Joint Manager")
    parser.add_argument("-d", "--directory", type=Path, help="Path to the directory containing JSON files.")
    parser.add_argument("-f", "--file", type=Path, help="Path to a single JSON file.")
    parser.add_argument("-s", "--save", action="store_true", help="Save processed data to JSON.")
    
    args = parser.parse_args()
    manager = RobotJointManager()

    if args.directory:
        if args.directory.exists() and args.directory.is_dir():
            manager.process_directory(args.directory)
            print("Processed directory:", args.directory)
        else:
            print("Invalid directory path.")

    elif args.file:
        if args.file.exists() and args.file.is_file():
            manager.process_file(args.file)
            print("Processed file:", args.file)
        else:
            print("Invalid file path.")

    else:
        print("Please provide a directory (-d) or a file (-f) to process.")

    # Save data if requested
    if args.save:
        save_path = input("Enter the directory to save JSON files: ")
        save_dir = Path(save_path)
        if not save_dir.exists():
            print(f"Creating directory: {save_dir}")
            save_dir.mkdir(parents=True, exist_ok=True)

        for joint_name, joint_data in manager.managed_joints.items():
            joint_data.save_data(save_dir)
        print(f"Saved joint data to {save_dir}")

    # Interactive Mode
    while True:
        print("\nOptions:")
        print("1. List joints")
        print("2. Plot full joint data")
        print("3. Plot individual values (position, velocity, effort)")
        print("4. Save data")
        print("5. Get joint data")
        print("6. Plot frequency spectrum")
        print("7. Exit")
        print("8. Apply low-pass filter")
        print("9. Magic button (automate analysis & filtering for one joint)")
        print("10. Magic button for all arm joints (filter effort & acceleration, plot all data & spectra)")

        choice = input("Select an option: ")

        if choice == "1":
            print("Managed Joints:", list(manager.managed_joints.keys()))

        elif choice == "2":
            joint_name = input("Enter joint name: ")
            if joint_name in manager.managed_joints:
                manager.managed_joints[joint_name].plot_joint_data()
            else:
                print("Joint not found.")

        elif choice == "3":
            joint_name = input("Enter joint name: ")
            if joint_name in manager.managed_joints:
                print("1. Position\n2. Velocity\n3. Effort")
                sub_choice = input("Select value to plot: ")
                joint = manager.managed_joints[joint_name]

                if sub_choice == "1":
                    joint.plot_position_data()
                elif sub_choice == "2":
                    joint.plot_velocity_data()
                elif sub_choice == "3":
                    joint.plot_effort_data()
                else:
                    print("Invalid choice.")
            else:
                print("Joint not found.")

        elif choice == "4":
            save_path = input("Enter the directory to save JSON files: ")
            save_dir = Path(save_path)
            if not save_dir.exists():
                print(f"Creating directory: {save_dir}")
                save_dir.mkdir(parents=True, exist_ok=True)

            for joint_name, joint_data in manager.managed_joints.items():
                joint_data.save_data(save_dir)
            print(f"Saved joint data to {save_dir}")

        elif choice == "5":
            joint_name = input("Enter joint name: ")
            if joint_name in manager.managed_joints:
                joint = manager.managed_joints[joint_name]
                print("1. Timestamps\n2. Positions\n3. Velocities\n4. Efforts\n5. Accelerations")
                sub_choice = input("Select data to retrieve: ")
                
                if sub_choice == "1":
                    print(joint.timestamps)
                elif sub_choice == "2":
                    print(joint.positions)
                elif sub_choice == "3":
                    print(joint.velocities)
                elif sub_choice == "4":
                    print(joint.efforts)
                elif sub_choice == "5":
                    print(joint.accelerations)
                else:
                    print("Invalid choice.")
            else:
                print("Joint not found.")
        
        elif choice == "6":
            joint_name = input("Enter joint name: ")
            if joint_name in manager.managed_joints:
                joint = manager.managed_joints[joint_name]
                print("1. Position\n2. Velocity\n3. Effort\n4. Acceleration")
                sub_choice = input("Select data for frequency spectrum: ")
                
                if sub_choice == "1":
                    joint.plot_frequency_spectrum(joint.positions, joint.timestamps)
                elif sub_choice == "2":
                    joint.plot_frequency_spectrum(joint.velocities, joint.timestamps)
                elif sub_choice == "3":
                    joint.plot_frequency_spectrum(joint.efforts, joint.timestamps)
                elif sub_choice == "4":
                    joint.plot_frequency_spectrum(joint.accelerations, joint.timestamps)
                else:
                    print("Invalid choice.")
            else:
                print("Joint not found.")

        elif choice == "7":
            print("Exiting...")
            break

        elif choice == "8":
            joint_name = input("Enter joint name: ")
            if joint_name in manager.managed_joints:
                signal = input("Specify which signal you want to filter, you can choose from ['positions', 'velocities', 'efforts', 'accelerations']:")
                cutoff_frequency = float(input("Enter cutoff frequency for low-pass filter: "))
                order = int(input("Specify the order of the filter: "))
                joint = manager.managed_joints[joint_name]
                joint.apply_low_pass_filter(signal, cutoff_frequency, order)
                print(f"Applied low-pass filter with cutoff {cutoff_frequency} Hz to {joint_name}.")
            else:
                print("Joint not found.")
        elif choice == "9":
            joint_name = input("Enter joint name: ")
            if joint_name not in manager.managed_joints:
                print("Joint not found.")
                continue

            joint = manager.managed_joints[joint_name]

            # Plot unfiltered data
            print(f"\nPlotting unfiltered data for {joint_name}...")
            joint.plot_joint_data()

            # Plot frequency spectrum of all signals
            print("\nPlotting frequency spectra...")
            joint.plot_frequency_spectrum(joint.positions, joint.timestamps)
            joint.plot_frequency_spectrum(joint.velocities, joint.timestamps)
            joint.plot_frequency_spectrum(joint.efforts, joint.timestamps)
            joint.plot_frequency_spectrum(joint.accelerations, joint.timestamps)

            # Optionally apply low-pass filter
            signals = ['positions', 'velocities', 'efforts', 'accelerations']
            for signal in signals:
                apply = input(f"\nApply low-pass filter to {signal}? (y/n): ").strip().lower()
                if apply == "y":
                    cutoff_frequency = float(input(f"Enter cutoff frequency for {signal}: "))
                    order = int(input("Specify the order of the filter: "))
                    joint.apply_low_pass_filter(signal, cutoff_frequency, order)
                    print(f"Applied low-pass filter to {signal}.")

            # Re-plot filtered data
            print(f"\nPlotting filtered data for {joint_name}...")
            joint.plot_joint_data()
        
        elif choice == "10":
            arm_joints = [f"panda_joint{i}" for i in range(1, 8)]
            for joint_name in arm_joints:
                if joint_name not in manager.managed_joints:
                    print(f"Skipping {joint_name}: not found.")
                    continue

                joint = manager.managed_joints[joint_name]
                print(f"\n--- Processing {joint_name} ---")

                # Plot unfiltered full joint data
                print("Plotting unfiltered joint data...")
                joint.plot_joint_data()

                # Plot unfiltered frequency spectrum for effort and acceleration
                print("Plotting frequency spectra (unfiltered)...")
                joint.plot_frequency_spectrum(joint.efforts, joint.timestamps)
                joint.plot_frequency_spectrum(joint.accelerations, joint.timestamps)

                # Apply low-pass filter to effort and acceleration
                print("Applying 2nd-order low-pass filter with 1 Hz cutoff to effort...")
                joint.apply_low_pass_filter("efforts", cutoff=1.0, order=2)

                print("Applying 2nd-order low-pass filter with 1 Hz cutoff to acceleration...")
                joint.apply_low_pass_filter("accelerations", cutoff=1.0, order=2)

                # Plot filtered full joint data
                print("Plotting filtered joint data...")
                joint.plot_joint_data()

                # Plot frequency spectra after filtering
                print("Plotting frequency spectra (filtered)...")
                joint.plot_frequency_spectrum(joint.efforts, joint.timestamps)
                joint.plot_frequency_spectrum(joint.accelerations, joint.timestamps)

        else:
            print("Invalid choice. Try again.")

if __name__ == "__main__":
    main()
