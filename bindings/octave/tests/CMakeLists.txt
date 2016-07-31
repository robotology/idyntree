
# For now the octave tests are exactly the matlab tests,
# and they are kept in ../../matlab/tests

add_custom_target(copy_octave_tests_in_build ALL)
add_custom_command(TARGET copy_octave_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR}/../../matlab/tests ${CMAKE_CURRENT_BINARY_DIR})

add_test(NAME octave_idyntree_tests
         COMMAND ${OCTAVE_EXECUTABLE} --no-gui --quiet
                 --eval "addpath('$<TARGET_FILE_DIR:${target_name}>');addpath(genpath('${IDYNTREE_INTERNAL_MOXUNIT_PATH}'));success=moxunit_runtests('-verbose');exit(~success);")
