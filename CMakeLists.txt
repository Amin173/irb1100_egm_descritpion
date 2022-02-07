cmake_minimum_required(VERSION 3.9.1)

####################################################################################################
# Metadata
####################################################################################################
# Read the package manifest.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" package_xml_str)

# Extract project name.
if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
  message(FATAL_ERROR "Could not parse project name from package manifest (aborting)")
else()
  set(extracted_name ${CMAKE_MATCH_1})
endif()

# Extract project version.
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
else()
  set(extracted_version ${CMAKE_MATCH_1})
endif()

####################################################################################################
# CMake project
####################################################################################################
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

#===================================================================================================
# Dependencies
#===================================================================================================
find_package(abb_egm_rws_managers REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  abb_robot_msgs
  angles
  roscpp
)

#===================================================================================================
# Catkin package specific configurations
#===================================================================================================
catkin_package(
  CATKIN_DEPENDS
    abb_robot_msgs
    angles
    roscpp
  DEPENDS
    abb_egm_rws_managers
)

#===================================================================================================
# Targets
#===================================================================================================

#---------------------------------------------------------------------------------------------------
# Executable target (manual irb1100 robot description example)
#---------------------------------------------------------------------------------------------------
set(src_files
  src/main.cpp
)

add_executable(${PROJECT_NAME} ${src_files})

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
  "${catkin_INCLUDE_DIRS}"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  abb_egm_rws_managers::abb_egm_rws_managers
  ${catkin_LIBRARIES}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  target_compile_options(${PROJECT_NAME} PRIVATE /W4)
endif()

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    OUTPUT_NAME "rc_description_irb1100_example_node"
)

#===================================================================================================
# Installation
#===================================================================================================
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
  DIRECTORY launch
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)
