cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)


project(OptimalControlTests)

#add_executable(OptimalControlTest OptimalControlTest.cpp)

#target_link_libraries(OptimalControlTest iDynTree::optimalcontrol ${iDynTree_LIBRARIES})

#add_test(NAME OptimalControlTest
#        COMMAND ${PROJECT_NAME})

add_executable(ConstraintsGroupTest ConstraintsGroupTest.cpp)

target_link_libraries(ConstraintsGroupTest PRIVATE idyntree-optimalcontrol ${iDynTree_LIBRARIES})


add_test(NAME ConstraintsGroupTest
        COMMAND ConstraintsGroupTest)


add_executable(RK4Test RK4Test.cpp)

target_link_libraries(RK4Test PRIVATE idyntree-optimalcontrol ${iDynTree_LIBRARIES})


add_test(NAME RK4Test
         COMMAND RK4Test)
