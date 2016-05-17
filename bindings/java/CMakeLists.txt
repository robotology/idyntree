set(CMAKE_SWIG_FLAGS "-Wextra;-module;iDynTree;-package;iDynTree")

find_package(Java REQUIRED COMPONENTS Runtime Development)
find_package(JNI  REQUIRED)

include(UseJava)

# CMake structure inspired by http://stackoverflow.com/a/31750207/1379427

# Specify where to place generated *.java files
SET(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_BINARY_DIR}/java")

# Add module
set(target_name iDynTree_java)
set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)
swig_add_module(${target_name} java ../iDynTree.i)
include_directories(SYSTEM ${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})

# Create directory for the *class files
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/classes")

# Create JAR including compiled .class file 
add_custom_command(TARGET ${target_name} POST_BUILD
                   COMMAND "${Java_JAVAC_EXECUTABLE}" -d classes java/*.java
                   COMMAND "${Java_JAR_EXECUTABLE}" -cfM iDynTree.jar -C classes .)

# Store Path to iDynTree JAR in variable:
SET(IDYNTREE_JAR "${CMAKE_CURRENT_BINARY_DIR}/iDynTree.jar")


# if compile tests execute also java tests
# if(IDYNTREE_COMPILE_TESTS)
#    add_subdirectory(tests)
# endif()
