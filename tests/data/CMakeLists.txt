# This folder contains resources useful for testing (such as URDF files)
# that are copied in all the test directories

set(IDYNTREE_TESTS_URDFS icub.urdf
                        icub_model.urdf
                        icub_skin_frames.urdf
                        bigman.urdf
                        twoLinks.urdf)

get_property(IDYNTREE_BINARY_TEST_DIRS GLOBAL PROPERTY IDYNTREE_BINARY_TEST_DIRS)

foreach(IDYNTREE_TESTS_DIRECTORY ${IDYNTREE_BINARY_TEST_DIRS})
    foreach(URDF_TO_COPY ${IDYNTREE_TESTS_URDFS})
        message(STATUS "Copy ${CMAKE_CURRENT_SOURCE_DIR}/${URDF_TO_COPY} to ${IDYNTREE_TESTS_DIRECTORY}/${URDF_TO_COPY}")
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${URDF_TO_COPY}"
                       "${IDYNTREE_TESTS_DIRECTORY}/${URDF_TO_COPY}")
    endforeach()
endforeach()