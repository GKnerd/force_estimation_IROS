cmake_minimum_required(VERSION 3.10)
project(panda_dyn_model_example)

## Environment settings
set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17) # Use C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

## Find Dependencies
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########
message("Configuring main Project: ${PROJECT_NAME}")
message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

## Include directories
include_directories(
  include 
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_FRANKA_MODEL
  src/FrictionTorque.cpp
  src/CoriolisMatrix.cpp
  src/GravityVector.cpp
  src/MassMatrix.cpp
)

## Declare a C++ library
set(FRANKA_MODEL_LIBRARY franka_model)
add_library(${FRANKA_MODEL_LIBRARY} SHARED ${SRC_FRANKA_MODEL})
target_include_directories(${FRANKA_MODEL_LIBRARY} PUBLIC include)
set_target_properties(${FRANKA_MODEL_LIBRARY} PROPERTIES POSITION_INDEPENDENT_CODE ON)

## Declare a C++ executable
add_executable(Panda_Model src/PandaDynamicModel.cpp)

## Link Libraries
target_link_libraries(
                Panda_Model 
                ${FRANKA_MODEL_LIBRARY} 
                ${EIGEN3_INCLUDE_DIRS}
                )