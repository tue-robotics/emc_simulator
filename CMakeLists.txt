cmake_minimum_required(VERSION 3.0.2)
project(emc_simulator)

find_package(catkin REQUIRED COMPONENTS
    geolib2
    code_profiler
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    3rdparty
    ${catkin_INCLUDE_DIRS}
)

add_library(polypartition
    3rdparty/polypartition/polypartition.cpp
)

add_executable(pico_simulator
    src/main.cpp
    src/world.cpp
    src/heightmap.cpp
    src/visualization.cpp
    src/lrf.cpp
)
target_link_libraries(pico_simulator polypartition ${catkin_LIBRARIES})
