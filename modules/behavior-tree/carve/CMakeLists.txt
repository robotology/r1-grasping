cmake_minimum_required(VERSION 2.8)

add_library(carve_nodes   SHARED src/carve_nodes.cpp )

# This tutorial demonstrates how to use blackboards and NodeParameters

#########################################################
# FIND YARP
#########################################################

find_package(YARP REQUIRED)
find_package(behavior_tree_core REQUIRED)
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
ADD_DEFINITIONS(${YARP_DEFINES}) # optional

include_directories(${YARP_INCLUDE_DIRS}, include
                    )

add_executable(carve_fetch         src/carve_fetch.cpp )
target_link_libraries(carve_fetch  carve_nodes ${BEHAVIOR_TREE_LIBRARIES} ${YARP_LIBRARIES})


add_executable(environment_dummy         src/environment_dummy.cpp )
target_link_libraries(environment_dummy  ${YARP_LIBRARIES})

