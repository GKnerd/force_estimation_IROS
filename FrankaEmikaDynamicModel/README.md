# Franka Emika Panda Dynamic Model Computation

This project computes the dynamic model of the Franka Emika Panda robot based on joint position, velocity, and acceleration data. 
The program accepts a `.json` file as input, which contains the required joint data.

## Dependencies

Before building the project, ensure you have the following dependencies installed:

- **Eigen3 Library**:  
  The project uses Eigen3 for linear algebra operations. You can install it with the following command:
  ```bash
  sudo apt install libeigen3-dev

### CMake

CMake is required for building the project. It can be installed on Ubuntu with:

```bash
sudo apt install cmake
```

### Project Structure

```
panda_dyn_model
├── include                # Header files
├── src                    # Source files
├── CMakeLists.txt         # CMake configuration
└── build/                 # Build directory (to be created)
```

### Building the Code

Follow these steps to build the project:

1. **Create a Build Directory**

   In the root of the project directory, create a build folder:

   ```bash
   mkdir build
   cd build
   ```

2. **Configure the Project**

   Run cmake to configure the project:

   ```bash
   cmake ..
   ```

3. **Build the Code**
Build the project using make. Use -j$(nproc) to parallelize the build and speed up the process:

   ```bash
   make -j$(nproc)
   ```

### 4. Run the Model

After the build is complete, run the program with your .json input file:

```bash
./Model_example <path_to_your_json_file>
```

### Input Format

The .json input file should contain the joint data for position (`q`), velocity (`dq`), and acceleration (`ddq`). Here's an example of how the file should look:

```json
{
  "q": [
    [
      0.056114619377817476,
      -0.5534220730204362,
      -1.23234276234938,
      -1.5375574171501294,
      -0.542010481996893,
      1.4721460558838313,
      1.0545445427091331
    ]
    ...
  ],
  "dq": [
    [
      0.056114619377817476,
      -0.5534220730204362,
      -1.23234276234938,
      -1.5375574171501294,
      -0.542010481996893,
      1.4721460558838313,
      1.0545445427091331
    ]
    ...
  ],
  "ddq": [
    [
      0.056114619377817476,
      -0.5534220730204362,
      -1.23234276234938,
      -1.5375574171501294,
      -0.542010481996893,
      1.4721460558838313,
      1.0545445427091331
    ]
    ...
  ]
}
```

- `q`: Joint position data (radians)
- `dq`: Joint velocity data
- `ddq`: Joint acceleration data

Each of these fields contains an array of joint states at discrete time steps.

### Reference

The original publication for the dynamic model of the Franka Emika Panda is:

C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019

[Read the paper](https://ieeexplore.ieee.org/document/8772145)

The original code can be found at:
[Franka Emika Panda Dynamic Model GitHub Repository](https://github.com/marcocognetti/FrankaEmikaPandaDynModel/tree/master)

### Troubleshooting

#### Eigen3 Not Found:
If CMake can't find Eigen3, ensure it's installed correctly and try specifying the path to the Eigen3Config.cmake file. For example:

```bash
cmake -DEigen3_DIR=/path/to/eigen3/cmake ..
```

#### Missing .json File:
Make sure the .json file is formatted correctly and the file path is provided when running the program.


