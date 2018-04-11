# add integration tests, that
# test interaction between components
add_subdirectory(integration)

if(IDYNTREE_USES_KDL)
    # Integration tests of old kdl_codyco project
    add_subdirectory(kdl_tests)

    # Consistency tests with kdl stuff
    add_subdirectory(kdl_consistency)

    # Benchmarks against old RNEA & CRBA based on kdl
    add_subdirectory(benchmark)

    # Comparative tests of implementations of similar methods in iDynTree, Eigen, KDL and YARP
    # if( IDYNTREE_USES_YARP )
    #     add_subdirectory(yarp_kdl_consistency)
    # endif()
endif()

if(IDYNTREE_USES_ICUB_MAIN)
    # Add benchmark test against yarp/icub functions
    add_subdirectory(yarp_benchmark)

    # Add a subdirectory of all the consistency tests between the old
    # iCub-dependent external force/internal torque estimation
    # implementations and the new native iDynTree implementations
    add_subdirectory(icub_consistency)
endif()
