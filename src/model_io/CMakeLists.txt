add_subdirectory(urdf)

if(IDYNTREE_USES_KDL)
    add_subdirectory(urdf-kdl)
endif()

option(IDYNTREE_ENABLE_SYMORO_PAR "Enable support for SyMoRo par format" TRUE)
if(IDYNTREE_ENABLE_SYMORO_PAR AND IDYNTREE_USES_KDL)
    add_subdirectory(symoro)
endif()

if(IDYNTREE_USES_ICUB_MAIN AND IDYNTREE_USES_KDL)
    add_subdirectory(iKin)
endif()

if(IDYNTREE_COMPILE_TESTS AND IDYNTREE_USES_KDL)
    add_subdirectory(tests)
endif()