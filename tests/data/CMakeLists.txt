# This folder contains resources useful for testing (such as URDF files)
# that are copied in all the test directories

# if you add a file to this list, please
# remember to add it also in the
# testModels.h.in file
set(IDYNTREE_TESTS_URDFS icub.urdf
                         icub_model.urdf
                         icub_skin_frames.urdf
                         bigman.urdf
                         twoLinks.urdf
                         oneLink.urdf
                         icubTwoLinks.urdf)

# first method: copy the test urdf in the test directories
get_property(IDYNTREE_BINARY_TEST_DIRS GLOBAL PROPERTY IDYNTREE_BINARY_TEST_DIRS)

foreach(IDYNTREE_TESTS_DIRECTORY ${IDYNTREE_BINARY_TEST_DIRS})
    foreach(URDF_TO_COPY ${IDYNTREE_TESTS_URDFS})
        message(STATUS "Copy ${CMAKE_CURRENT_SOURCE_DIR}/${URDF_TO_COPY} to ${IDYNTREE_TESTS_DIRECTORY}/${URDF_TO_COPY}")
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${URDF_TO_COPY}"
                       "${IDYNTREE_TESTS_DIRECTORY}/${URDF_TO_COPY}")
    endforeach()
endforeach()

# second method: configure an header containing the path
# where the test files are stored and the list of files
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/testModels.h.in" "${CMAKE_CURRENT_BINARY_DIR}/testModels.h" @ONLY)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
