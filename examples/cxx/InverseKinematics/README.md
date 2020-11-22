# iDynTree InverseKinematics Example

This is an example of use of the iDynTree InverseKinematics

## Compilation

Make sure that you installed iDynTree with the `IDYNTREE_USES_IPOPT` CMake option selected, and your iDynTree installation
can be found by CMake's [`find_package`](https://cmake.org/cmake/help/latest/command/find_package.html), either because it is installed in a system
location or because its installation prefix is present in [`CMAKE_PREFIX_PATH`](https://cmake.org/cmake/help/latest/variable/CMAKE_PREFIX_PATH.html).

After that, the example can be compiled as any CMake project, for example on *nix systems:
~~~bash
mkdir build
cd build
cmake ..
make
~~~

For how to compile using IDEs, see https://cgold.readthedocs.io/en/latest/first-step.html .


No installation target is provided, as the example is meant to be run from the build directory.

## Run the example
To run the example, go to the `build` directory and execute the `iDynTreeExampleInverseKinematics` executable.
The executable assumes that a file `kr16_2.urdf` is available in the same directory where the executable as launched,
so you need to launch the executable in its directory. The file `kr16_2.urdf` is downloaded by CMake during configuration.


For a detailed overview of the example, check its source code.

