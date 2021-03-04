# Call moxunit_runtests in ${CMAKE_CURRENT_SOURCE_DIR}
# Furthermore add to the pass the direcotry of iDynTreeMEX.mex* ,
# the location of +iDynTree and the location of MOxUnit
add_custom_target(copy_matlab_model_in_build ALL)
add_custom_command(TARGET copy_matlab_model_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                            ${CMAKE_CURRENT_SOURCE_DIR}/model.urdf ${CMAKE_CURRENT_BINARY_DIR}/model.urdf)

option(IDYNTREE_DISABLE_MATLAB_TESTS "If ON, disable MATLAB tests enve if IDYNTREE_USES_MATLAB and the option to compile tests are both on" OFF)
if (IDYNTREE_USES_MATLAB AND NOT IDYNTREE_DISABLE_MATLAB_TESTS)
  find_package(Matlab REQUIRED
               COMPONENTS MAIN_PROGRAM)
  add_test(NAME matlab_idyntree_tests
           COMMAND ${Matlab_MAIN_PROGRAM} -nodisplay -nodesktop -nojvm -r "addpath('$<TARGET_FILE_DIR:${mexname}>');addpath('${MEX_BINDINGS_SOURCE_DIR}');addpath('${MATLAB_WRAPPERS_BINDINGS_SOURCE_DIR}');addpath('${CMAKE_CURRENT_SOURCE_DIR}/');addpath(genpath('${IDYNTREE_INTERNAL_MOXUNIT_PATH}'));success=moxunit_runtests('${CMAKE_CURRENT_SOURCE_DIR}','-verbose');exit(~success);")
endif()

if (IDYNTREE_USES_OCTAVE)
  add_test(NAME octave_idyntree_tests
           COMMAND ${OCTAVE_EXECUTABLE} --no-gui --quiet --eval "addpath('$<TARGET_FILE_DIR:idyntreeOctaveMex>');addpath('${MEX_BINDINGS_SOURCE_DIR}');addpath('${MATLAB_WRAPPERS_BINDINGS_SOURCE_DIR}');addpath('${CMAKE_CURRENT_SOURCE_DIR}/');addpath(genpath('${IDYNTREE_INTERNAL_MOXUNIT_PATH}'));success=moxunit_runtests('${CMAKE_CURRENT_SOURCE_DIR}','-verbose');exit(~success);")
endif()
