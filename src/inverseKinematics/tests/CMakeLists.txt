cmake_minimum_required(VERSION 2.8.12)

# set the project name
project(InverseKinematicsTest)

# add the required cmake packages
# find_package(RTF)
# find_package(RTF COMPONENTS DLL YARP)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

# add include directories
# include_directories(${CMAKE_SOURCE_DIR}
#                     ${RTF_INCLUDE_DIRS}
#                     ${YARP_INCLUDE_DIRS})

# add required libraries 
# link_libraries(${RTF_LIBRARIES}
#                ${YARP_LIBRARIES})

# add the source codes to build the plugin library
add_executable(${PROJECT_NAME} iKinVersusLegacyTest.cpp)

target_link_libraries(${PROJECT_NAME} SYSTEM inverseKinematics ${YARP_LIBRARIES} iKin)


# set the installation options
# install(TARGETS ${PROJECTNAME}
#         EXPORT ${PROJECTNAME}
#         COMPONENT runtime
#         LIBRARY DESTINATION lib)
#
