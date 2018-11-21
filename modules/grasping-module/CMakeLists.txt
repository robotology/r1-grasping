# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

project(grasping-module)
find_package(YARP 3.1.0 REQUIRED COMPONENTS OS sig math dev)

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}/idl")
yarp_idl_to_dir(src/idl.thrift ${generated_libs_dir} sources headers include_dirs)

add_definitions(-D_USE_MATH_DEFINES)
include_directories(${include_dirs})
add_executable(${PROJECT_NAME} src/main.cpp ${sources} ${headers})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

file(GLOB ini app/conf/*.ini)
yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
