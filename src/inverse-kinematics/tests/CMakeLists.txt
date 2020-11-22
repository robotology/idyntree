macro(add_ik_test classname)
  set(testsrc ${classname}UnitTest.cpp)
  set(testbinary ${classname}UnitTest)
  set(testname   UnitTest${classname})
  add_executable(${testbinary} ${testsrc})
  target_link_libraries(${testbinary} PRIVATE idyntree-model idyntree-inverse-kinematics idyntree-testmodels Eigen3::Eigen)
  add_test(NAME ${testname} COMMAND ${testbinary})

  if(IDYNTREE_RUN_VALGRIND_TESTS)
    add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
  endif()
endmacro()

add_ik_test(ConvexHullHelpers)
add_ik_test(InverseKinematics)

