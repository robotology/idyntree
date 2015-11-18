## How to add wrap a new class or function with SWIG 
* Include the new header in `bindings/iDynTree.i` . 
  * Notice that the headers should be included two times, take inspiration from the headers already present to get the idea.
  * Remember to include the headers following the inheritance order of the classes. 
* If you add a instantiated template, remember to add it to swig with the `%template` SWIG command. 
* Regenerate the MATLAB bindings, following the instructions in https://github.com/robotology/idyntree#matlab-bindings-modifications . 
* (Optional) Add a new matlab test testing the new code in `bindings/matlab/tests`. 
  * You should add a script named `testNameOfTheTest.m` and then add it to the `runiDynTreeTests.m` script. 
* Run matlab tests with `ctest -VV -R matlab` in the build directory, and check that they are running file. 
* Commit the updated version of the bindings by adding everything inside the `bindings/matlab` directory, with commit message `[bindings] update matlab bindings`. 
