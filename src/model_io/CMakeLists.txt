add_subdirectory(urdf)

if(IDYNTREE_ENABLE_SYMORO_PAR)
    add_subdirectory(symoro)
endif()

if(IDYNTREE_USES_ICUB_MAIN)
    add_subdirectory(iKin)
endif()

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()