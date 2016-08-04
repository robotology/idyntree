#  iDynTree
#  Copyright: (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Silvio Traversaro <silvio.traversaro@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

# An external TinyXML is compulsory if compiling
# with KDL support enabled, because KDL uses urdfdom
# that include TinyXML in its public headers.
# If KDL is not enabled, we can use also the internal
# copy of TinyXML, given that we don't include it in
# our public headers
if(NOT IDYNTREE_USES_KDL)
    # If no TinyXML is found in the system, use the internal one
    if(NOT TinyXML_FOUND)
        # TinyXML
        add_subdirectory(tinyxml)
        set(TinyXML_INCLUDE_DIRS ${TinyXML_INCLUDE_DIRS} PARENT_SCOPE)
        set(TinyXML_LIBRARIES ${TinyXML_LIBRARIES} PARENT_SCOPE)
        set(TinyXML_DEFINITIONS ${TinyXML_DEFINITIONS} PARENT_SCOPE)
    endif()
endif()

# MOxUnit ( https://github.com/MOxUnit/MOxUnit ) is an xUnit testing
# framework for Matlab and Octave, it is only included if testing
# is enabled in Matlab or Octave
if( (IDYNTREE_USES_OCTAVE OR IDYNTREE_USES_MATLAB OR IDYNTREE_GENERATE_MATLAB) AND
    IDYNTREE_COMPILE_TESTS )
    # We save the location of MOxUnit in a variable that is then accessible to the test
    set(IDYNTREE_INTERNAL_MOXUNIT_PATH "${CMAKE_CURRENT_LIST_DIR}/MOxUnit/MOxUnit" PARENT_SCOPE)
endif()




