
add_custom_target(copy_matlab_tests_in_build ALL)
add_custom_command(TARGET copy_matlab_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})
add_custom_command(TARGET copy_matlab_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_SOURCE_DIR}/../+iDynTree ${CMAKE_BINARY_DIR}/+iDynTree)
add_custom_command(TARGET copy_matlab_swigref_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/../SwigRef.m ${CMAKE_BINARY_DIR}/SwigRef.m)

add_test(NAME matlab_idyntree_tests COMMAND ${Matlab_MAIN_PROGRAM} -nodisplay -nodesktop -nojvm -r "runiDynTreeTests")