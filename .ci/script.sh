#!/bin/sh
set -e

# Install source deps

# Build  YARP
git clone https://github.com/robotology/yarp
cd yarp
git checkout devel
mkdir build
cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" -DCREATE_LIB_MATH:BOOL=ON -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} ..
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install
cd ../..

# Build ICUB
git clone https://github.com/robotology/icub-main
cd icub-main
git checkout devel
mkdir build
cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} ..
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install
cd ../..

# Build, test and install iDynTree
cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} -DIDYNTREE_COMPILE_TESTS:BOOL=ON -DIDYNTREE_RUN_VALGRIND_TESTS:BOOL=${VALGRIND_TESTS} -DCODYCO_TRAVIS_CI:BOOL=ON -DIDYNTREE_USES_KDL:BOOL=${FULL_DEPS} -DIDYNTREE_USES_YARP:BOOL=${FULL_DEPS} -DIDYNTREE_USES_ICUB_MAIN:BOOL=${FULL_DEPS}  -DIDYNTREE_USES_PYTHON:BOOL=${COMPILE_BINDINGS} -DIDYNTREE_USES_LUA:BOOL=${COMPILE_BINDINGS} -DIDYNTREE_USES_OCTAVE:BOOL=${COMPILE_BINDINGS} ..
cmake --build . --config $TRAVIS_BUILD_TYPE
ctest --output-on-failure --build-config ${TRAVIS_BUILD_TYPE}
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install

# Build iDynTree examples
cd ../examples
mkdir build
cd build
cmake -G"${TRAVIS_CMAKE_GENERATOR}" -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} ..
cmake --build . --config ${TRAVIS_BUILD_TYPE}

