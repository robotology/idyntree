include_directories(include)

add_library(kdl_import SHARED src/kdl_import.cpp)
target_link_libraries(kdl_import ${urdfdom_libraries} ${tinyxml_libraries} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})

# --------------------------------

#add_executable(check_urdf src/check_urdf.cpp)
#target_link_libraries(check_urdf urdfdom_model urdfdom_world)

INSTALL(TARGETS kdl_import DESTINATION lib/)
INSTALL(DIRECTORY include/kdl_import DESTINATION include)
