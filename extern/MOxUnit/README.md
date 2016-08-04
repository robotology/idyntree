# MOxUnit [![Build Status](https://travis-ci.org/nno/MOxUnit.svg?branch=master)](https://travis-ci.org/MOxUnit/MOxUnit) [![Coverage Status](https://coveralls.io/repos/github/MOxUnit/MOxUnit/badge.svg?branch=master)](https://coveralls.io/github/MOxUnit/MOxUnit?branch=master)

MOxUnit is a lightweight unit test framework for Matlab and GNU Octave.

### Features

- Runs on both the [Matlab] and [GNU Octave] platforms.
- Uses object oriented TestCase, TestSuite and TestResult classes, allowing for user-defined extensions.
- Can be used directly with continuous integration services, such as [Travis-ci] and [Shippable].
- Supports JUnit-like XML output for use with Shippable and other test results visualization approaches.
- Supports the generation of code coverage reports using [MOCov]
- Provides compatibility with Steve Eddin's [Matlab xUnit test framework].
- Distributed under the MIT license, a permissive free software license.


### Installation

- Using the shell (requires a Unix-like operating system such as GNU/Linux or Apple OSX):

    ```bash
    git clone https://github.com/MOxUnit/MOxUnit.git
    cd MOxUnit
    make install
    ```
    This will add the MOxUnit directory to the Matlab and/or GNU Octave searchpath. If both Matlab and GNU Octave are available on your machine, it will install MOxUnit for both.

- Manual installation:

    + Download the zip archive from the [MOxUnit] website.
    + Start Matlab or GNU Octave.
    + On the Matlab or GNU Octave prompt, `cd` to the `MOxUnit` root directory, then run:

        ```matlab
        cd MOxUnit          % cd to MOxUnit subdirectory
        moxunit_set_path()  % add the current directory to the Matlab/GNU Octave path
        savepath            % save the path
        ```

### Running MOxUnit tests

- `cd` to the directory where the unit tests reside. For MOxUnit itself, the unit tests are in the directory `tests`.
- run the tests using `moxunit_runtests`. For example, running `moxunit_runtests` from MOxUnit's `tests` directory should give the following output:

  ```
  suite: 31 tests
  ...............................
  --------------------------------------------------

  OK
  ans =

       1
  ```

- `moxunit_runtests`, by default, gives non-verbose output and runs all tests in the current directory. This can be changed using the following arguments:
  - `-verbose`: show verbose output.
  - `directory`: run unit tests in directory `directory`.
  - `file.m`: run unit tests in file `file.m`.
  - `-recursive`: add files from directories recursively.
  - `-logfile logfile.txt`: store the output in file `logfile.txt`.
  - `-junit_xml_file xmlfile`: store JUnit-like XML output in file `xmlfile`.

- To test MOxUnit itself using a shell, run:

    ```
    make test
    ```

### Use with travis-ci and Shippable
MOxUnit uses the [Travis-ci] service for continuous integration testing. This is achieved by setting up a [.travis.yml configuration file](.travis.yml). This file is also used by [Shippable].
As a result, the test suite is run automatically on both [Travis-ci] and [Shippable] every time it is pushed to the github repository, or when a pull request is made. If a test fails, or if all tests pass after a test failed before, the developers are notified by email.

### Defining MOxUnit tests

To define unit tests, write a function with the following header:

```matlab
function test_suite=my_test_of_abs
    initTestSuite;
```

*Important*: it is crucial that the output of the main function is called `test_suite`.

Then, define subfunctions whose name start with `test_` or end with `_test`. These functions can use the following `assert*` functions:
- `assertTrue(a)`: assert that `a` is true.
- `assertFalse(a)`: assert that `a` is false.
- `assertEqual(a,b)`: assert that `a` and `b` are equal.
- `assertElementsAlmostEqual(a,b)`: assert that the floating point arrays `a` and `b` have the same size, and that corresponding elements are equal within some numeric tolerance.
- `assertVectorsAlmostEqual(a,b)`: assert that floating point vectors `a` and `b` have the same size, and are equal within some numeric tolerance based on their vector norm.
- `assertExceptionThrown(f,id)`: assert that calling `f()` throws an exception with identifier `id`. (To deal with cases where Matlab and GNU Octave throw errors with different identifiers, use `moxunit_util_platform_is_octave`).

As a special case, `moxunit_throw_test_skipped_exception('reason')` throws an exception that is caught when running the test; `moxunit_run_tests` will report that the test is skipped for reason `reason`.

For example, the following function defines three unit tests that tests some possible inputs from the builtin `abs` function:
```matlab
function test_suite=my_test_of_abs
    initTestSuite

function test_abs_scalar
    assertTrue(abs(-1)==1)
    assertEqual(abs(-NaN),NaN);
    assertEqual(abs(-Inf),Inf);
    assertEqual(abs(0),0)
    assertElementsAlmostEqual(abs(-1e-13),0)

function test_abs_vector
    assertEqual(abs([-1 1 -3]),[1 1 3]);

function test_abs_exceptions
    % GNU Octave and Matlab use different error identifiers
    if moxunit_util_platform_is_octave()
        assertExceptionThrown(@()abs(struct),'');
    else
        assertExceptionThrown(@()abs(struct),...
                             'MATLAB:UndefinedFunction');
    end
```

Examples of unit tests are in MOxUnit's `tests` directory, which test some of MOxUnit's functions itself.

### Compatibility notes
- Because GNU Octave 3.8 does not support `classdef` syntax, 'old-style' object-oriented syntax is used for the class definitions. For similar reasons, MOxUnit uses the `lasterror` function, even though its use in Matlab is discouraged.


### Acknowledgements
- The object-oriented class structure was inspired by the [Python unit test] framework.
- The `assert*` function signatures are aimed to be compatible with Steve Eddin's [Matlab xUnit test framework].


### Limitations
Currently MOxUnit does not support:
- Documentation tests (these would require `evalc`, which is not available on `GNU Octave` as of January 2014).
- Support for setup and teardown functions in `TestCase` classes.


### Contact
Nikolaas N. Oosterhof, nikolaas dot oosterhof at unitn dot it


### Contributions
- Thanks to Scott Lowe, Thomas Feher and Joel LeBlanc for contributions.


### License

(The MIT License)

Copyright (c) 2015 Nikolaas N. Oosterhof

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.



[GNU Octave]: http://www.gnu.org/software/octave/
[Matlab]: http://www.mathworks.com/products/matlab/
[Matlab xUnit test framework]: http://it.mathworks.com/matlabcentral/fileexchange/22846-matlab-xunit-test-framework
[MOxUnit]: https://github.com/MOxUnit/MOxUnit
[MOcov]: https://github.com/MOcov/MOcov
[Python unit test]: https://docs.python.org/2.6/library/unittest.html
[Travis-ci]: https://travis-ci.org
[Shippable]: https://app.shippable.com/



