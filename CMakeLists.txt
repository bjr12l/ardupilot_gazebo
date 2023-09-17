cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ardupilot_gazebo)

# --------------------------------------------------------------------------- #
# If ament_cmake is found build as an ament package, otherwise ignore.
# This is so the system may be built for Gazebo only, if ROS is not available.
find_package(ament_cmake QUIET)
if(${ament_cmake_FOUND})
  message("Building ${PROJECT_NAME} as an `ament_cmake` project.")
endif()

# --------------------------------------------------------------------------- #
# Compile as C++14.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------- #
# Find gazebo libs
set(PACKAGES
  sdformat13
  gz-cmake3
  gz-sim7
  gz-sensors7
  gz-math7
  gz-utils2
  gz-common5
  gz-msgs9
  gz-rendering7
  gz-transport12
  gz-plugin2
  # Add more packages as needed
)

foreach(PACKAGE ${PACKAGES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# --------------------------------------------------------------------------- #
# Find RapidJSON.
find_package(RapidJSON REQUIRED)

# --------------------------------------------------------------------------- #
# Find GStreamer.
find_package(PkgConfig REQUIRED)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)

# --------------------------------------------------------------------------- #
# Find OpenCV.
find_package(OpenCV REQUIRED)

# --------------------------------------------------------------------------- #
# Build plugin.

add_library(ArduPilotPlugin
    SHARED
    src/ArduPilotPlugin.cc
    src/Socket.cpp
    src/Util.cc
)
target_include_directories(ArduPilotPlugin PRIVATE
  include
)
target_link_libraries(ArduPilotPlugin PRIVATE
  gz-sim7::gz-sim7
)

add_library(ParachutePlugin
  SHARED
  src/ParachutePlugin.cc
)
target_include_directories(ParachutePlugin PRIVATE
  include
)
target_link_libraries(ParachutePlugin PRIVATE
  gz-sim7::gz-sim7
)

add_library(GstCameraPlugin
    SHARED
    src/GstCameraPlugin.cc
)
target_include_directories(GstCameraPlugin PRIVATE
  include
)
target_link_libraries(GstCameraPlugin PRIVATE
  ${OpenCV_LIBS}
  PkgConfig::gstreamer
  PkgConfig::gstreamer-app
  sdformat13::sdformat13
  gz-sim7::gz-sim7
  gz-sensors7::gz-sensors7
  gz-sensors7::gz-sensors7-camera
  gz-math7::gz-math7
  gz-utils2::gz-utils2
  gz-common5::gz-common5
  gz-msgs9::gz-msgs9
  gz-rendering7::gz-rendering7
  gz-transport12::gz-transport12
  gz-plugin2::gz-plugin2
)

# --------------------------------------------------------------------------- #
# Install.

install(
  TARGETS
  ArduPilotPlugin
  ParachutePlugin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# --------------------------------------------------------------------------- #
# Register as an ament package if ament_cmake is available.
if(${ament_cmake_FOUND})
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

  ament_package()
endif()