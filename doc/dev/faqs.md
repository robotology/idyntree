## How to add wrap a new class or function with SWIG 
* Include the new header in [bindings/iDynTree.i](bindings/iDynTree.i) . 
  * Notice that the headers should be included two times, take inspiration from the headers already present to get the idea.
  * Remember to include the headers following the inheritance order of the classes. 
* If you add a instantiated template, remember to add it to swig with the `%template` SWIG command. 
* Regenerate the MATLAB bindings, following the instructions in https://github.com/robotology/idyntree#matlab-bindings-modifications . 
* (Optional) Add a new matlab test testing the new code in `bindings/matlab/tests`. 
  * You should add a script named `testNameOfTheTest.m` and then add it to the `runiDynTreeTests.m` script. 
* Run matlab tests with `ctest -VV -R matlab` in the build directory, and check that they are running file. 
* Commit the updated version of the bindings by adding everything inside the `bindings/matlab` directory, with commit message `[bindings] update matlab bindings`. 

## How to add a class or a function to a component documentation 
* For each component (i.e. "part" of iDynTree, such as Core, Model, ...) a [Doxygen group](https://www.stack.nl/~dimitri/doxygen/manual/grouping.html) is defined in [doc/main.dox](../../doc/main.dox), usually following the `iDynTree${ComponentName}` structure ( `iDynTreeCore`, `iDynTreeModel`). 
* To add a class or a function to a group, just add `\ingroup iDynTree${ComponentName}` to the doxygen documentation of the class/function. Search for [`\ingroup` in the repo](https://github.com/robotology/idyntree/search?q=\ingroup)  to see how this is done for existing classes.
