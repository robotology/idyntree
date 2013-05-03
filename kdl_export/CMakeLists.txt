include_directories(include)

add_library(kdl_export SHARED src/kdl_export.cpp)
target_link_libraries(kdl_export ${tinyxml_libraries} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})

# --------------------------------

#add_executable(check_urdf src/check_urdf.cpp)
#target_link_libraries(check_urdf urdfdom_model urdfdom_world)

INSTALL(TARGETS kdl_export DESTINATION lib/)
INSTALL(DIRECTORY include/kdl_export DESTINATION include)
