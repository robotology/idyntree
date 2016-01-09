# add integration tests, that
# test interaction between components
add_subdirectory(integration)

if(IDYNTREE_USES_KDL)
    # Integration tests of old kdl_codyco project
    add_subdirectory(kdl_tests)

    # Benchmarks against old RNEA & CRBA based on kdl
    add_subdirectory(benchmark)

    if(IDYNTREE_USES_ICUB_MAIN)
        # Add benchmark test against yarp/icub functions
        add_subdirectory(yarp_benchmark)

        # Add a subdirectory of all the consistency tests between the old
        # iCub-dependent external force/internal torque estimation
        #  implementations and the new native iDynTree implementations
        add_subdirectory(icub_consistency)
    endif()

endif()