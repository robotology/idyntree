# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
FIND_PACKAGE(Boost)
include_directories(${Boost_INCLUDE_DIR})

add_executable(check_urdf_import_export check_urdf_import_export.cpp)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/urdf/black_icub.urdf ${CMAKE_CURRENT_BINARY_DIR}/black_icub.urdf)
target_link_libraries(check_urdf_import_export idyntree-kdl idyntree-modelio-urdf-kdl ${orocos_kdl_LIBRARIES} ${urdfdom_LIBRARIES} idyntree-testmodels)
add_test(NAME test_urdf_import_export COMMAND check_urdf_import_export black_icub.urdf)

if (IDYNTREE_USES_SYMORO_PAR)
    add_executable(check_symoro_par_import_fixed_chain_regressor check_symoro_par_import_fixed_chain_regressor.cpp)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/symoro_par/fake_puma.par ${CMAKE_CURRENT_BINARY_DIR}/fake_puma.par)
    target_link_libraries(check_symoro_par_import_fixed_chain_regressor idyntree-kdl idyntree-modelio-symoro ${orocos_kdl_LIBRARIES})
    add_test(NAME test_par_import_fixed_chain_regressor COMMAND check_symoro_par_import_fixed_chain_regressor fake_puma.par)
endif()

if(IDYNTREE_USES_ICUB_MAIN)
    add_executable(check_iKin_export_random_chain check_iKin_export_random_chain.cpp)
    target_link_libraries(check_iKin_export_random_chain idyntree-core idyntree-modelio-urdf-kdl idyntree-modelio-ikin-kdl YARP::YARP_os YARP::YARP_math iKin)
    add_test(NAME test_check_iKin_export_random_chain COMMAND check_iKin_export_random_chain)

endif()

#check iKin Denavit Hartenberg parameters export
#add_executable(check_iKin_export_random_chain check_iKin_export_random_chain.cpp)
#target_link_libraries(check_iKin_export_random_chain ${kdl_codyco_LIBRARIES} YARP::YARP_os YARP::YARP_math ${ICUB_LIBRARIES} kdl-format-io)


#In Release mode, don't compile the HRP2 Regressor model because it takes a lot of time
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    add_executable(check_symoro_par_import_floating_tree_regressor check_symoro_par_import_floating_tree_base_regressor.cpp)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/symoro_par/HRP2JRL_IMU.par ${CMAKE_CURRENT_BINARY_DIR}/HRP2JRL_IMU.par)
    target_link_libraries(check_symoro_par_import_floating_tree_regressor idyntree-core idyntree-regressors idyntree-modelio-symoro ${orocos_kdl_LIBRARIES})
    add_test(NAME test_par_import_floating_tree_regressor COMMAND check_symoro_par_import_floating_tree_regressor HRP2JRL_IMU.par)
ENDIF()
