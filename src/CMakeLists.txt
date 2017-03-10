# include test data before everything,
# to be sure that is available to test
# of all components
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests/data)
endif()

add_subdirectory(core)
add_subdirectory(model)
add_subdirectory(sensors)
add_subdirectory(model_io)
add_subdirectory(estimation)
add_subdirectory(high-level)

if (${IDYNTREE_USES_IPOPT})
  add_subdirectory(inverse-kinematics)
endif()

if(IDYNTREE_USES_YARP)
    add_subdirectory(yarp)
endif()

if(IDYNTREE_USES_ICUB_MAIN)
    add_subdirectory(icub)
endif()

if(IDYNTREE_USES_KDL)
    add_subdirectory(kdl)
    add_subdirectory(regressors)
    if(IDYNTREE_USES_YARP)
        add_subdirectory(yarp-kdl)
        add_subdirectory(estimation-kdl)
        if(IDYNTREE_USES_ICUB_MAIN)
            add_subdirectory(icub-kdl)
        endif()
    endif()
    add_subdirectory(high-level-kdl)
endif()

add_subdirectory(visualization)

# compile simple command line tools
add_subdirectory(tools)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()