# add integration tests, that
# test interaction between components
add_subdirectory(integration)


# add regression tests, to
# verify that solved bug are not
# reintroduced
add_subdirectory(regression)

if(IDYNTREE_USES_KDL)
    # Integration tests of old kdl_codyco project
    add_subdirectory(kdl_tests)

    # Benchmarks against old RNEA & CRBA based on kdl
    add_subdirectory(benchmark)

    # Add a subdirectory of all the consistency tests between the old
    # "kdl_codyco" implementations and the new native iDynTree implementations
    add_subdirectory(kdl_consistency)

endif()
