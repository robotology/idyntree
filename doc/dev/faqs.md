## How to run the tests 
iDynTree uses tests to prevent regressions caused by modification in the source code of the library. 
To compile and run the test locally, enable the `IDYNTREE_COMPILE_TESTS` CMake option and then compile
iDynTree as usual. Once you have compiled the library, run the `ctest` program from the build directory 
to run all the tests. If you enabled the compilation of MATLAB bindings using the `IDYNTREE_USES_MATLAB` options, 
test validating the functionality of the MATLAB interface will be automatically run. 

## How to run the Valgrind-based tests 
[Valgrind MemCheck](http://valgrind.org/) is a tool to identify memory related software bugs (use of initialize memory, memory leaks, ...). To automatically run the iDynTree test suite under Valgrind, just enabled the `IDYNTREE_RUN_VALGRIND_TESTS` CMake option (together with the `IDYNTREE_COMPILE_TESTS` option). 

## How to get notified if my code fails to compile or create a failure in some tests 
iDynTree uses [Travis](https://travis-ci.org/robotology/idyntree) and [AppVeyor](https://ci.appveyor.com/project/robotology/idyntree) Continous Integration (CI) services
to make sure that all code commited to the repository. To be notified if some of your commits
failed to pass tests, just sign in [Travis](https://travis-ci.org/) and [AppVeyor](https://ci.appveyor.com/login)
with your GitHub account, and you will automatically receive notifications if your commits cause a failure. 

## What are the things that needs to be done before merging a pull request?
* Check if Travis and AppVeyor compiler and run the test without any failure.  

## How to add wrap a new class or function with SWIG 
* Include the new header in [bindings/iDynTree.i](bindings/iDynTree.i) . 
  * Notice that the headers should be included two times, take inspiration from the headers already present to get the idea.
  * Remember to include the headers following the inheritance order of the classes. 
* If you add a instantiated template, remember to add it to swig with the `%template` SWIG command. 
* Regenerate the MATLAB bindings, following the instructions in https://github.com/robotology/idyntree#matlab-bindings-modifications . 
* (Optional) Add a new matlab test testing the new code in `bindings/matlab/tests`. 
  * The structure of tests in `bindings/matlab/tests` uses [MOxUnit](https://github.com/MOxUnit/MOxUnit) a unit test framework for Matlab and Octave. Please see the MOxUnit documentation and existing tests for understanding the structure of the tests. As the tests are run by `Travis` also on Octave, please make sure that your tests do not use Matlab-specific features.
* Run matlab tests with `ctest -VV -R matlab` in the build directory, and check that they are running file. 
* Commit the updated version of the bindings by adding everything inside the `bindings/matlab` directory, with commit message `[bindings] update matlab bindings`. 

## How to add a class or a function to a component documentation 
* For each component (i.e. "part" of iDynTree, such as Core, Model, ...) a [Doxygen group](https://www.stack.nl/~dimitri/doxygen/manual/grouping.html) is defined in [doc/main.dox](../../doc/main.dox), usually following the `iDynTree${ComponentName}` structure ( `iDynTreeCore`, `iDynTreeModel`). 
* To add a class or a function to a group, just add `\ingroup iDynTree${ComponentName}` to the doxygen documentation of the class/function. Search for [`\ingroup` in the repo](https://github.com/robotology/idyntree/search?q=\ingroup)  to see how this is done for existing classes.

## How do I generated Doxygen documentation locally on my computer? 
* If you have [Doxygen](www.doxygen.org) installed on your machine, you can simply generate the iDynTree Doxygen documentation by building the `dox` target, for example using `make` you just need to type:
~~~
make dox
~~~ 
in the iDynTree build, and the Doxygen documentation in HTML format will be generated in `build/doc/html/index.html`. 
* To generate the call/caller graphs in the Doxygen documentation to simplify code inspection, just enable the ` IDYNTREE_DOXYGEN_VERBOSE` advanced CMake option. This are disabled by default because they render the documentation quite unreadable for the simple users.
