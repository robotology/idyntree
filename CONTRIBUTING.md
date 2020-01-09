This page is used as a starting point for contributing to the development of the iDynTree library.

If you would like to contribute to the development of iDynTree, please get in contact with the development team using [GitHub issues](https://github.com/robotology/idyntree/issues).

The easiest way to contribute is to pick an issue and try to solve it. If you want to get an easy issue to solve for a beginner to the library, just ask to the development team .

If you need a new feature or to fix a bug, please [fill a GitHub issue](https://github.com/robotology/gazebo-yarp-plugins/issues/new) describing the feature or the bugfix you would like to implement.

## Code style
We are not currently enforcing a specific style for the code in iDynTree, but if you modify 
a file, please be consistent with the style of the file. 

## Patches and features contribution
The contribution follows the classical GitHub stages:
* open an issue (so we can discuss the problem and possible solutions);
* develop the changes;
* fork the project and submit the pull request.

## Repository structure and releases management.
Development of new features follows the [GitHub Flow](https://guides.github.com/introduction/flow/index.html) ( also know as [Feature Branch Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow) ).

In a nutshell, development of new features/bugfixes happens in separated branch. When you believe your contribution is stable you can open a pull request against master, where your code will be review by other contributors prior to merging it.  

## Continuous Integration 
We use [https://en.wikipedia.org/wiki/Continuous_integration](Continuous Integration) to make 
sure that proposed changes in the library do not cause regression. In particular we use 
[GitHub Actions](https://travis-ci.org/robotology/idyntree) for Continuous Integration on Linux, macOS and Windows. 

In all this build systems the compilation of the tests is enabled by setting to true 
the `IDYNTREE_COMPILE_TESTS` CMake flag. 

Almost all the dependencies are enabled in the Linux build. 
Furthermore on Linux builds the `IDYNTREE_RUN_VALGRIND_TESTS` flag is enabled, to 
run all the tests also under the [`valgrind`](http://valgrind.org/) tool, to spot 
any memory-related error, suck of use of initialized memory or memory leak. 

Some dependencies or options are disabled on Windows and macOS, and you can find the specific iDynTree configuration used in the GitHub Actions build in the [`.github/workflows/ci.yml`](https://github.com/robotology/idyntree/blob/master/.github/workflows/ci.yml) file. 


