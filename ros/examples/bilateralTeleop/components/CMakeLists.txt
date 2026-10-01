#
# (C) Copyright 2025-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.10)
project (saw_intuitive_research_kit_example_bilateral_teleop VERSION 2.4.0)

# first test for ROS1
set (ROS1_depends
     cisst_ros_crtk roscpp roslib)

find_package (catkin QUIET
              COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
  catkin_package (INCLUDE_DIRS include "${CATKIN_DEVEL_PREFIX}/include"
                  LIBRARIES saw_intuitive_research_kit_example_bilateral_teleop
                  CATKIN_DEPENDS ${ROS1_depends})
  include_directories (${catkin_INCLUDE_DIRS})
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    set (ROS2_depends
      rclcpp
      std_msgs std_srvs
      geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs
      cisst_msgs crtk_msgs
      cisst_ros_bridge cisst_ros_crtk
      )
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find cisst and make sure the required libraries have been compiled
set (
  REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical)

find_package (cisst REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# set paths
cisst_set_output_path ()

# sawIntuitiveResearchKit has been compiled within cisst, we should
# find it automatically
find_package(sawIntuitiveResearchKit 2.4.0 REQUIRED)

# sawControllers, for mtsPIDConfiguration.h -- mtsBilateralTeleOperationPSM
# reads each arm's live PID "configuration" command directly (e.g. to
# confirm use_disturbance_observer's actual running state), not just what
# a JSON config file says
find_package(sawControllers REQUIRED)

# ONNX Runtime C++ SDK, used by mtsContactDetector for the contact
# classification model. Not packaged by ROS/apt on this system, so it's a
# vendored prebuilt release; hint at a couple of common locations and allow
# override via the ONNXRUNTIME_ROOT environment variable.
find_path (ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS
    "$ENV{ONNXRUNTIME_ROOT}/include"
    "$ENV{HOME}/software/onnxruntime-linux-x64-1.20.1/include"
    "/opt/onnxruntime/include")
find_library (ONNXRUNTIME_LIBRARY onnxruntime
  HINTS
    "$ENV{ONNXRUNTIME_ROOT}/lib"
    "$ENV{HOME}/software/onnxruntime-linux-x64-1.20.1/lib"
    "/opt/onnxruntime/lib")

if (NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message (FATAL_ERROR
    "ONNX Runtime C++ SDK not found. Download a prebuilt release from "
    "https://github.com/microsoft/onnxruntime/releases (linux-x64 tgz), "
    "extract it, and either place it at ~/software/onnxruntime-linux-x64-1.20.1 "
    "or set the ONNXRUNTIME_ROOT environment variable to its install prefix.")
endif ()

# libonnxruntime.so lives outside any standard system lib path, so bake its
# directory into the built library's RPATH -- otherwise it's only found at
# runtime if LD_LIBRARY_PATH happens to be set
get_filename_component (ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)

if (sawIntuitiveResearchKit_FOUND)

  set (saw_intuitive_research_kit_example_bilateral_teleop_INCLUDE_DIR
    "${saw_intuitive_research_kit_example_bilateral_teleop_SOURCE_DIR}/include")

  include_directories (
    ${saw_intuitive_research_kit_example_bilateral_teleop_INCLUDE_DIR}
    ${CMAKE_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    ${sawControllers_INCLUDE_DIR}
    ${ONNXRUNTIME_INCLUDE_DIR})

  link_directories (
    ${sawIntuitiveResearchKit_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR})


  set (BILATERAL_SOURCES
       "include/mtsBilateralTeleOperationPSM.h"
       "include/sawIntuitiveResearchKitBilateralTeleopExport.h"
       "code/mtsBilateralTeleOperationPSM.cpp"
       "include/mtsContactDetector.h"
       "code/mtsContactDetector.cpp"
       "include/mtsDynamicsCompensator.h"
       "code/mtsDynamicsCompensator.cpp")

  if (ROS_VERSION)
    set (BILATERAL_SOURCES ${BILATERAL_SOURCES}
         "include/mtsBilateralTeleOperationPSMROSBridge.h"
         "code/mtsBilateralTeleOperationPSMROSBridge.cpp")
  endif ()

  add_library (sawIntuitiveResearchKitBilateralTeleop
               ${BILATERAL_SOURCES})

  # base_dir used by mtsContactDetector to resolve relative onnx_model /
  # normalization_json paths from the teleop JSON config; this package has
  # no install() step for share/ today (dev-tree paths are used as-is,
  # matching how MTML_PSM2_teleop_config.json itself is already located)
  target_compile_definitions (sawIntuitiveResearchKitBilateralTeleop PRIVATE
    BILATERAL_TELEOP_SHARE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../share")

  set_target_properties (sawIntuitiveResearchKitBilateralTeleop PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
    INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")

  set (sawIntuitiveResearchKitBilateralTeleop_LINK_LIBRARIES
       ${cisst_LIBRARIES}
       ${sawIntuitiveResearchKit_LIBRARIES}
       ${sawControllers_LIBRARIES}
       ${ONNXRUNTIME_LIBRARY})

  if (ROS_VERSION EQUAL 1)
    list (APPEND sawIntuitiveResearchKitBilateralTeleop_LINK_LIBRARIES
          ${catkin_LIBRARIES})
  endif ()

  target_link_libraries (sawIntuitiveResearchKitBilateralTeleop
                         ${sawIntuitiveResearchKitBilateralTeleop_LINK_LIBRARIES})

  if (ROS_VERSION EQUAL 2)
    ament_target_dependencies (
      sawIntuitiveResearchKitBilateralTeleop
      ${ROS2_depends}
      )
    ament_package ()
  endif ()

endif (sawIntuitiveResearchKit_FOUND)
