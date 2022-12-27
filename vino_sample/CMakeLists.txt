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

cmake_minimum_required(VERSION 3.0.2)

project(vino_sample)

find_package(OpenVINO)
if (NOT OpenVINO_FOUND)
    message(FATAL_ERROR "")
endif()

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  roslint
  cv_bridge
  object_msgs
  vino_people_msgs
  vino_pipeline_srv_msgs
  vino_param_lib
  vino_core_lib
)

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
  object_msgs
  vino_param_lib
  vino_core_lib
)

if (CMAKE_BUILD_TYPE EQUAL "RELEASE")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
    message(STATUS "Create Debug Build.")
endif()

include_directories (
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenVINO_DIR}
  ${vino_param_lib_INCLUDE_DIRS}
)

if(OpenCV_FOUND)
    message(STATUS "OPENCV is enabled")
    message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
else()
    message(STATUS "OPENCV is disabled or not found, " ${PROJECT_NAME} " skiped")
    return()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(pipeline_with_params
  src/pipeline_with_params.cpp
)

add_dependencies(pipeline_with_params
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

set(OpenVINO_LIBRARIES openvino::runtime)
target_link_libraries(pipeline_with_params
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_people_server
  src/image_people_server.cpp
)

add_dependencies(image_people_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(image_people_server
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(image_people_client
  src/image_people_client.cpp
)

add_dependencies(image_people_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(image_people_client
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_object_server
  src/image_object_server.cpp
)

add_dependencies(image_object_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(image_object_server
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(image_object_client
  src/image_object_client.cpp
)

add_dependencies(image_object_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(image_object_client
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(image_segmentation_client
  src/image_segmentation_client.cpp
)

add_dependencies(image_segmentation_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(image_segmentation_client
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_segmentation_server
  src/image_segmentation_server.cpp
)

add_dependencies(image_segmentation_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(image_segmentation_server
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(image_reidentification_client
  src/image_reidentification_client.cpp
)

add_dependencies(image_reidentification_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(image_reidentification_client
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_reidentification_server
  src/image_reidentification_server.cpp
)

add_dependencies(image_reidentification_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(image_reidentification_server
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(pipeline_service_client
  src/pipeline_service_client.cpp
)

add_dependencies(pipeline_service_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${vino_core_lib_TARGETS}
)

target_link_libraries(pipeline_service_client
  ${catkin_LIBRARIES}
  gflags
  ${vino_param_lib_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

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

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
