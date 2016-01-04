
add_custom_target(copy_matlab_tests_in_build ALL)
add_custom_command(TARGET copy_matlab_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
add_custom_command(TARGET copy_matlab_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR}/../+iDynTree ${CMAKE_CURRENT_BINARY_DIR}/+iDynTree)
add_custom_command(TARGET copy_matlab_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_CURRENT_SOURCE_DIR}/../SwigRef.m ${CMAKE_CURRENT_BINARY_DIR}/SwigRef.m)
add_custom_command(TARGET copy_matlab_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_CURRENT_SOURCE_DIR}/../SwigMem.m ${CMAKE_CURRENT_BINARY_DIR}/SwigMem.m)

add_test(NAME matlab_idyntree_tests COMMAND ${Matlab_MAIN_PROGRAM} -nodisplay -nodesktop -nojvm -r "runiDynTreeTests")
