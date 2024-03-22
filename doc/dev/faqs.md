## What are best entry points to understand iDynTree structure? 
The best way to understand iDynTree is probably to read directly the source code. 

Some classes extremly used are the following:

| Class name | Description |  Doxygen docs | Headers (`.h`) | Source Code (`.cpp`) | Relevant Tests | 
|:----------:|:-----------:|:------------:|:--------------:|:-----------:|:-----:|
| `iDynTree::Transform` | Generic transform between two different frames. | [link](http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1Transform.html) | [ src/core/include/iDynTree/Transform.h](../../src/core/include/iDynTree/Transform.h) | [ src/core/src/Transform.cpp](../../src/core/src/Transform.cpp) | [src/core/tests](../../src/core/tests) | 
| `iDynTree::Model` | Generic undirected (i.e. with no "base") multibody model. | [link](http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1Model.html) | [ src/model/include/iDynTree/Model.h](../../src/model/include/iDynTree/Model.h) | [ src/model/src/Model.cpp](../../src/model/src/Model.cpp) | [src/model/tests/ModelUnitTest.cpp](../../src/model/tests/ModelUnitTest.cpp) | 
| `iDynTree::Traversal` | Class describing a order in which link are visited once a "base link" is choosen |  [link](http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1Traversal.html) | [ src/model/include/iDynTree/Traversal.h](../../src/model/include/iDynTree/Traversal.h) | [ src/model/src/Traversal.cpp](../../src/model/src/Traversal.cpp) | [src/model/tests/ModelUnitTest.cpp](../../src/model/tests/ModelUnitTest.cpp) | 
 

If you need any additional doxygen comments, please [open an issue](https://github.com/robotology/idyntree/issues/new) detailing which point of the code you find difficul to understand. 

Internally iDynTree uses a lot the [`Eigen` C++ template vector library](https://eigen.tuxfamily.org/dox/index.html), and in particularly the [`Eigen::Map`](https://eigen.tuxfamily.org/dox/group__TutorialMapClass.html)
class is used a lot. Please check [Eigen documentation](https://eigen.tuxfamily.org/dox/index.html) for more details. 

## How to run the tests 
iDynTree uses tests to prevent regressions caused by modification in the source code of the library. 
To compile and run the test locally, enable the `BUILD_TESTING` CMake option and then compile
iDynTree as usual. Once you have compiled the library, run the `ctest` program from the build directory 
to run all the tests. If you enabled the compilation of MATLAB bindings using the `IDYNTREE_USES_MATLAB` options, 
test validating the functionality of the MATLAB interface will be automatically run. 

## How to run the Valgrind-based tests 
[Valgrind MemCheck](http://valgrind.org/) is a tool to identify memory related software bugs (use of initialize memory, memory leaks, ...). To automatically run the iDynTree test suite under Valgrind, just enabled the `IDYNTREE_RUN_VALGRIND_TESTS` CMake option (together with the `BUILD_TESTING` option). 

## What are the things that needs to be done before merging a pull request?
* Check if the GitHub Actions jobs compiles and run the test without any failure.  

## How to add wrap a new class or function with SWIG 
* Include the new header in [bindings/iDynTree.i](bindings/iDynTree.i) .  For a little more detailed overview, check [doc/generating-idyntree-matlab-bindings.md](../../doc/generating-idyntree-matlab-bindings.md) . 
  * Notice that the headers should be included two times, take inspiration from the headers already present to get the idea.
  * Remember to include the headers following the inheritance order of the classes. 
* If you add a instantiated template, remember to add it to swig with the `%template` SWIG command. 
* Regenerate the MATLAB bindings, following the instructions in https://github.com/robotology/idyntree#matlaboctave-bindings-modifications .  
* (Optional) Add a new matlab test testing the new code in `bindings/matlab/tests`. 
  * The structure of tests in `bindings/matlab/tests` uses [MOxUnit](https://github.com/MOxUnit/MOxUnit) a unit test framework for Matlab and Octave. Please see the MOxUnit documentation and existing tests for understanding the structure of the tests. As the tests are run by `GitHub Actions` also on Octave, please make sure that your tests do not use Matlab-specific features.
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
