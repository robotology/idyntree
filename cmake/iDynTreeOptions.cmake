


#########################################################################
# Control whether libraries are shared or static.
option(IDYNTREE_SHARED_LIBRARY "Compile iDynTree as a shared library" TRUE)
set(BUILD_SHARED_LIBS ${IDYNTREE_SHARED_LIBRARY})


#########################################################################
# Use position indipendent code
set (CMAKE_POSITION_INDEPENDENT_CODE TRUE)

#########################################################################
# Turn on testing.
option(IDYNTREE_COMPILE_TESTS "Compile iDynTree tests" FALSE)
if(IDYNTREE_COMPILE_TESTS)
   enable_testing()
endif()

#########################################################################
# Deal with RPATH
option(IDYNTREE_ENABLE_RPATH "Enable RPATH for the library" TRUE)
mark_as_advanced(IDYNTREE_ENABLE_RPATH)

#########################################################################
# Enable/disable dependencies

option(IDYNTREE_USE_ICUB_MAIN  "Compiled iDynTree with icub-main dependencies" TRUE)
