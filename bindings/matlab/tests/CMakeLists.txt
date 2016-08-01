# Call moxunit_runtests in ${CMAKE_CURRENT_SOURCE_DIR}
# Furthermore add to the pass the direcotry of iDynTreeMEX.mex* ,
# the location of +iDynTree and the location of MOxUnit 
add_test(NAME matlab_idyntree_tests
         COMMAND ${Matlab_MAIN_PROGRAM} -nodisplay -nodesktop -nojvm -r "addpath('$<TARGET_FILE_DIR:${mexname}>');addpath('${CMAKE_CURRENT_SOURCE_DIR}/../');addpath(genpath('${IDYNTREE_INTERNAL_MOXUNIT_PATH}'));success=moxunit_runtests('${CMAKE_CURRENT_SOURCE_DIR}','-verbose');exit(~success);")
