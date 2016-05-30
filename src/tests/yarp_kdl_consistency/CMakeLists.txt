get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

add_executable(RPYConsistencyTest RPYConsistency.cpp)
target_include_directories(RPYConsistencyTest PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(RPYConsistencyTest idyntree-modelio-urdf-kdl idyntree-yarp-kdl idyntree-icub idyntree-core idyntree-kdl idyntree-model)
add_test(NAME RPYConsistencyTest COMMAND RPYConsistencyTest)

add_executable(QuaternionConsistencyTest QuaternionConsistency.cpp)
target_include_directories(QuaternionConsistencyTest PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(QuaternionConsistencyTest idyntree-modelio-urdf-kdl idyntree-yarp-kdl idyntree-icub idyntree-core idyntree-kdl idyntree-model)
add_test(NAME QuaternionConsistencyTest COMMAND QuaternionConsistencyTest)