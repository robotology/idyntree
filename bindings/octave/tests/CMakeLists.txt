
# For now the octave tests are exactly the matlab tests,
# and they are kept in ../../matlab/tests

# Call moxunit_runtests in ${CMAKE_CURRENT_SOURCE_DIR}
# Furthermore add to the pass the direcotry of iDynTreeMEX.mex* ,
# the location of +iDynTree and the location of MOxUnit
add_custom_target(copy_octave_model_in_build ALL)
add_custom_command(TARGET copy_octave_model_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                            ${CMAKE_CURRENT_SOURCE_DIR}/../../matlab/tests/model.urdf ${CMAKE_CURRENT_BINARY_DIR}/model.urdf)

add_test(NAME octave_idyntree_tests
         COMMAND ${OCTAVE_EXECUTABLE} --no-gui --quiet
         --eval "addpath('$<TARGET_FILE_DIR:${target_name}>');addpath('${CMAKE_CURRENT_SOURCE_DIR}/../../matlab/tests');addpath(genpath('${IDYNTREE_INTERNAL_MOXUNIT_PATH}'));success=moxunit_runtests('${CMAKE_CURRENT_SOURCE_DIR}/../../matlab/tests','-verbose');exit(~success);")
