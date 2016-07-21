# This folder contains resources useful for testing (such as URDF files)
# that are copied in all the test directories

# if you add a file to this list, please
# remember to add it also in the
# testModels.h.in file
set(IDYNTREE_TESTS_URDFS icub.urdf
                         icub2BB5Sea.urdf
                         icub_model.urdf
                         icub_skin_frames.urdf
                         bigman.urdf
                         twoLinks.urdf
                         threeLinks.urdf
                         twoLinksFixed.urdf
                         oneLink.urdf
                         icubTwoLinks.urdf
                         robotModelTestTwoLinks.urdf
                         iCubGenova02.urdf
                         iCubDarmstadt01.urdf
                         twoLinksWithoutBaseSensors.urdf)


# configure an header containing the path
# where the test files are stored and the list of files
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/testModels.h.in" "${CMAKE_CURRENT_BINARY_DIR}/testModels.h" @ONLY)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}")