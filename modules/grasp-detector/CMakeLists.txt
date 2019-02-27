# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

project(grasp-detector)
find_package(YARP 3.1.0 REQUIRED COMPONENTS OS sig math dev)

yarp_add_idl(${PROJECT_NAME}_THRIFT_SRC src/idl.thrift)

add_executable(${PROJECT_NAME} src/main.cpp ${${PROJECT_NAME}_THRIFT_SRC})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

file(GLOB ini app/conf/*.ini)
yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
