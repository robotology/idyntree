macro(add_modelio_urdf_unit_test classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-modelio-urdf)
    add_test(NAME ${testname} COMMAND ${testbinary})
endmacro()

# Copy the tests URDF also in this directory
set_property(GLOBAL APPEND PROPERTY IDYNTREE_BINARY_TEST_DIRS "${CMAKE_CURRENT_BINARY_DIR}")


add_modelio_urdf_unit_test(URDFModelImport)