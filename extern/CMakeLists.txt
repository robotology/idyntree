#  iDynTree
#  Copyright: (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Silvio Traversaro <silvio.traversaro@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

# MOxUnit ( https://github.com/MOxUnit/MOxUnit ) is an xUnit testing
# framework for Matlab and Octave, it is only included if testing
# is enabled in Matlab or Octave
if( (IDYNTREE_USES_OCTAVE OR IDYNTREE_USES_MATLAB OR IDYNTREE_GENERATE_MATLAB) AND
    IDYNTREE_COMPILE_TESTS )
    # We save the location of MOxUnit in a variable that is then accessible to the test
    set(IDYNTREE_INTERNAL_MOXUNIT_PATH "${CMAKE_CURRENT_LIST_DIR}/MOxUnit/MOxUnit" PARENT_SCOPE)
endif()

# fpconv ( https://github.com/night-shift/fpconv ) is a small implementation of a shortest exact representation of double to string
# as of C++14, it has no equivalent in the C++ standaard, as using stringstream with setprecision(17) result
# in printing long numbers (such as 1.000000000000000000000) even when there is a short equivalent representation (1.0)
# This can be removed once std::to_chars from the charconv header introduced in C++17 becomes widely available
# We imported the MIT-licensed source code from https://github.com/night-shift/fpconv/tree/4a087d1b2df765baa409536931916a2c082cdda4
add_library(idyntree-private-fpconv OBJECT fpconv/fpconv.c)
target_include_directories(idyntree-private-fpconv PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/fpconv)


# mesh2tri ( https://it.mathworks.com/matlabcentral/fileexchange/28327-mesh2tri ) is a matlab function that allows converting a mesh format into a triangulation format.
# It is only installed if Matlab is enabled, not tested in Octave yet.
if (IDYNTREE_USES_MATLAB OR IDYNTREE_GENERATE_MATLAB)
    # We save the location of mesh2tri in a variable that is then accessible to the test
    set(IDYNTREE_INTERNAL_MESH2TRI_PATH "${CMAKE_CURRENT_LIST_DIR}/mesh2tri" PARENT_SCOPE)
endif()
