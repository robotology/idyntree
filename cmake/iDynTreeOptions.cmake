


#########################################################################
# Control whether libraries are shared or static.
if( MSVC )
option(IDYNTREE_SHARED_LIBRARY "Compile iDynTree as a shared library" FALSE)
else()
option(IDYNTREE_SHARED_LIBRARY "Compile iDynTree as a shared library" TRUE)
endif()
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
# Turn on bindings compilation.
option(IDYNTREE_COMPILE_BINDINGS "Compile iDynTree bindings" FALSE)

#########################################################################
# Turn on compilation of geometrical relations semantics check.
option(IDYNTREE_USES_SEMANTICS "Compile iDynTree semantics check" FALSE)

#########################################################################
# Deal with RPATH
option(IDYNTREE_ENABLE_RPATH "Enable RPATH for the library" TRUE)
mark_as_advanced(IDYNTREE_ENABLE_RPATH)

#########################################################################
# Enable/disable dependencies

option(IDYNTREE_ENABLE_SYMORO_PAR "Enable support for SyMoRo par format" TRUE)
option(IDYNTREE_USES_ICUB_MAIN  "Compiled iDynTree with icub-main dependencies (for iKin and skinDynLib helper functions and tools)" TRUE)
if( MSVC )
option(IDYNTREE_USES_INTERNAL_URDFDOM "Compile iDynTree with an internal copy of urdfdom patched to avoid Boost dependencies" TRUE)
else()
option(IDYNTREE_USES_INTERNAL_URDFDOM "Compile iDynTree with an internal copy of urdfdom patched to avoid Boost dependencies" FALSE)
endif()

if(IDYNTREE_USES_INTERNAL_URDFDOM)
    add_definitions(-DURDF_USE_PLAIN_POINTERS)
    add_definitions(-DIDYNTREE_USE_INTERNAL_URDFDOM)
endif()

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

# Compile flags definitions for Semantic checks
if(IDYNTREE_COMPILE_BINDINGS)
    add_definitions(-DIDYNTREE_COMPILE_BINDINGS)
endif(IDYNTREE_COMPILE_BINDINGS)

if(IDYNTREE_USES_SEMANTICS)
    add_definitions(-DIDYNTREE_USES_SEMANTICS)
endif(IDYNTREE_USES_SEMANTICS)
