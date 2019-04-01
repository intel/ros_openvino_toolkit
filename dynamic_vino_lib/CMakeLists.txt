# Copyright (c) 2018 Intel Corporation

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required (VERSION 2.8.3)

project(dynamic_vino_lib)

message(STATUS "Looking for inference engine configuration file at: ${CMAKE_PREFIX_PATH}")
find_package(InferenceEngine 1.1)
if (NOT InferenceEngine_FOUND)
    message(FATAL_ERROR "")
endif()

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  roslint
  std_msgs
  sensor_msgs
  object_msgs
  people_msgs
  image_transport
  cv_bridge
  InferenceEngine
)

# Find OpenCV libray if exists
find_package(
  OpenCV REQUIRED COMPONENTS
  core
  highgui
  imgproc
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  roslint
  cv_bridge
  image_transport
  object_msgs
)

include_directories (
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${InferenceEngine_INCLUDE_DIRS}
    ${InferenceEngine_INCLUDE_DIRS}/../samples
    ${InferenceEngine_DIR}/../src
    ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/gflags/include
    ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/extension/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../vino_param_lib/include
)

if (NOT DEFINED ENV{CPU_EXTENSION_LIB})
  message(FATAL_ERROR "Please set ENV CPU_EXTENSION_LIB with 'export CPU_EXTENSION_LIB=<path-to-libcpu_extension.so>'")
endif()
set (CpuExtension_lib $ENV{CPU_EXTENSION_LIB})
add_library(cpu_extension SHARED IMPORTED)
set_target_properties(cpu_extension PROPERTIES
  IMPORTED_LOCATION $ENV{CPU_EXTENSION_LIB})

# Flags
if(UNIX OR APPLE)
  # Linker flags.
  if( ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if( ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel" )
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

 # Add OpenMP support
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

# Add x86 intrinsic compiler support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
execute_process(
    COMMAND bash -c "lscpu | grep -qi flags | grep -qi flags | grep -qi f16c"
    RESULT_VARIABLE SUPPORT_F16C)
if (SUPPORT_F16C EQUAL 0)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mf16c")
  add_definitions(-DSUPPORT_MF16C)
endif()

execute_process(
    COMMAND bash -c "lscpu | grep -qi flags | grep -qi flags | grep -qi sse4_1"
    RESULT_VARIABLE SUPPORT_SSE41)
if (SUPPORT_SSE41 EQUAL 0)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
endif()

set(DEPENDENCIES realsense2 ${OpenCV_LIBS} cpu_extension)

add_library(${PROJECT_NAME} SHARED
  src/services/frame_processing_server.cpp
  src/factory.cpp
  src/pipeline.cpp
  src/pipeline_params.cpp
  src/pipeline_manager.cpp
  src/engines/engine.cpp
  src/inferences/base_inference.cpp
  src/inferences/emotions_detection.cpp
  src/inferences/age_gender_detection.cpp
  src/inferences/face_detection.cpp
  src/inferences/head_pose_detection.cpp
  src/inferences/object_detection.cpp
  src/inferences/object_segmentation.cpp
  src/inferences/person_reidentification.cpp
  src/inputs/realsense_camera.cpp
  src/inputs/realsense_camera_topic.cpp
  src/inputs/standard_camera.cpp
  src/inputs/video_input.cpp
  src/inputs/image_input.cpp
  src/models/base_model.cpp
  src/models/emotion_detection_model.cpp
  src/models/age_gender_detection_model.cpp
  src/models/face_detection_model.cpp
  src/models/head_pose_detection_model.cpp
  src/models/object_detection_model.cpp
  src/models/object_segmentation_model.cpp
  src/models/person_reidentification_model.cpp
  src/outputs/image_window_output.cpp
  src/outputs/ros_topic_output.cpp
  src/outputs/rviz_output.cpp
  src/outputs/base_output.cpp
  src/outputs/ros_service_output.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  "object_msgs"
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${DEPENDENCIES}
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
