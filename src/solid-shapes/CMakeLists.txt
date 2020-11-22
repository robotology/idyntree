set(libraryname idyntree-solid-shapes)

set(IDYNTREE_SOLID_SHAPES_SOURCES src/InertialParametersSolidShapesHelpers.cpp)
set(IDYNTREE_SOLID_SHAPES_HEADERS include/iDynTree/InertialParametersSolidShapesHelpers.h)

add_library(${libraryname} ${IDYNTREE_SOLID_SHAPES_HEADERS} ${IDYNTREE_SOLID_SHAPES_SOURCES})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")


target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-model
                                     PRIVATE Eigen3::Eigen)

if (IDYNTREE_USES_ASSIMP)
  target_compile_definitions(${libraryname} PRIVATE IDYNTREE_USES_ASSIMP)
  # Depending on the assimp version, the way of consuming assimp changes
  # In assimp 5.0, only the imported assimp::assimp target is defined, in earlier
  # versions (that do no export the version to CMake) the ASSIMP_* variables are defined
  # We assume that on WIN32 we always have a valid imported target as a workaround for 
  # https://github.com/microsoft/vcpkg/issues/14255
  if((assimp_VERSION AND assimp_VERSION VERSION_GREATER_EQUAL 5.0) OR WIN32)
    target_link_libraries(${libraryname} PRIVATE assimp::assimp)
  else()
    target_include_directories(${libraryname} PRIVATE "${ASSIMP_INCLUDE_DIRS}")
    link_directories("${ASSIMP_LIBRARY_DIRS}")
    target_link_libraries(${libraryname} PRIVATE "${ASSIMP_LIBRARIES}")
  endif()
endif()

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_SOLID_SHAPES_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(IDYNTREE_COMPILE_TESTS AND IDYNTREE_USES_ASSIMP)
    add_subdirectory(tests)
endif()
