get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

if(IDYNTREE_USES_ICUB_MAIN)

macro(add_all_components_test testName)
    set(testsrc ${testName}_test.cpp testhumanoid.cpp testdoublejoint.cpp testkukalwr.cpp testsinglejoint.cpp testsinglelink.cpp)
    set(testbinary ${testName}_test)
    set(testname   test_${testName})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_TREE_INCLUDE_DIRS})
    target_link_libraries(${testbinary} idyntree-core idyntree-high-level idyntree-yarp-kdl idyntree-icub-kdl ${YARP_LIBRARIES} ${orocos_kdl_LIBRARIES})
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} ./${testbinary})
    endif()
endmacro()

add_all_components_test(inverse_inertia)
add_all_components_test(regressor_utils)
add_all_components_test(subtree_regressor)
add_all_components_test(inversedynamics_regressor)
add_all_components_test(inversedynamics_crba)
add_all_components_test(comjacobian)
add_all_components_test(inversedynamics_different_loops)
add_all_components_test(DynamicsComputationsConsistency)

  find_package(YARP)
  find_package(ICUB)
  add_executable(urdf2dh_test urdf2dh_tests.cpp)
  target_include_directories(urdf2dh_test PUBLIC ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
  target_link_libraries(urdf2dh_test iKin idyntree-kdl idyntree-yarp-kdl idyntree-icub idyntree-regressors  idyntree-modelio-urdf-kdl idyntree-modelio-ikin)
  # \todo reenable this test
  #add_test(NAME test_urdf2dh COMMAND urdf2dh_test urdf_icub_test.urdf)
endif()

add_executable(urdf_forward_kinematics urdf_forward_kinematics.cpp)
target_link_libraries(urdf_forward_kinematics idyntree-core idyntree-kdl idyntree-modelio-urdf-kdl ${orocos_kdl_LIBRARIES})

#add_test(NAME test_urdf_forward_kinematics  COMMAND urdf_forward_kinematics black_icub.urdf root_link r_hip_3)
