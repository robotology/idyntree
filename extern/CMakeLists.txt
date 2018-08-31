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
