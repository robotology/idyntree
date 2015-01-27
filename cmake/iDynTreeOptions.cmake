


#########################################################################
# Control whether libraries are shared or static.

option(IDYNTREE_SHARED_LIBRARY "Compile iDynTree as a shared library" TRUE)


#########################################################################
# Turn on testing.
option(IDYNTREE_COMPILE_TESTS "Compile iDynTree tests" TRUE)
if(IDYNTREE_COMPILE_TESTS)
   enable_testing()
endif()

#########################################################################
# Deal with RPATH
option(IDYNTREE_ENABLE_RPATH "Enable RPATH for the library" TRUE)
mark_as_advanced(IDYNTREE_ENABLE_RPATH)