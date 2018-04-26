## Writing unit tests with the Catch unit-test framework (1.x branch)

Read the [tutorial](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/tutorial.md). More [documentation](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/Readme.md) is available from the [home page](https://github.com/catchorg/Catch2/tree/Catch1.x). Read it too.

### Summary of usage

1. **In general**&emsp;Write a unit-test master file as follow. Let's call it `unit-tests.cpp`.

    ```cpp
    #define CATCH_CONFIG_MAIN
    #include <catch.hpp>
    ```

    The preprocessor definition tells Catch to provide a `main()` function, and also makes the following include directive include Catch's implementation in addition to its headers. Catch is indeed a header-only library, its implementation is bundled with its headers in the single-include file `catch.hpp`, but it is protected from inclusion by `#ifdef`s, so that `#include <catch.hpp>` only includes the necessary headers unless `CATCH_CONFIG_MAIN` is defined.

    Only define `CATCH_CONFIG_MAIN` in **one** CPP file. There is no need to compile the whole of Catch into every translation unit that defines unit tests. This is useless and slow. See [this documentation page](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/slow-compiles.md) for more information.

    **In the CDFF**&emsp;The unit-test master file is `Tests/UnitTests/UnitTests.cpp`.

2. **In general**&emsp;Write your unit tests in other CPP files (or HPP+CPP files), let's say `factorial.cpp` and `square.cpp`:

    ```cpp
    #include <catch.hpp>

    unsigned int Factorial(unsigned int number) {
        return number <= 1 ? number : Factorial(number-1)*number;
        // bug: should be number > 1 ? Factorial(number-1)*number : 1
    }

    TEST_CASE( "Factorials are computed", "[factorial]" ) {
        REQUIRE( Factorial(0) == 1 );
        REQUIRE( Factorial(1) == 1 );
        REQUIRE( Factorial(2) == 2 );
        REQUIRE( Factorial(3) == 6 );
        REQUIRE( Factorial(10) == 3628800 );
    }
    ```

    ```cpp
    #include <catch.hpp>

    unsigned int Square(unsigned int number) {
        return number*number;
    }

    TEST_CASE( "Squares are computed", "[square]" ) {
        REQUIRE( Square(1) == 1 );
        REQUIRE( Square(2) == 4 );
    }
    ```

    These code snippets obviously assume that the Catch single-include header is available unqualified on the include directory path (i.e. in system include directories or in directories specified by `-I` to the compiler), but you may need to prefix it with a directory name if necessary.

    **In the CDFF**&emsp;This is the part that you have to do. Put your unit test files in the subdirectories of `Tests/UnitTests/`. The CMake build system is parameterized so that the Catch header file is available unqualified: just use `#include <catch.hpp>`.

3. **In general**&emsp;Separately compile all source files to object files, then link them together into a unique test executable, let's say `unit-tests`:

    ```shell
    $ g++ -c unit-tests.cpp factorial.cpp square.cpp
    $ g++ -o unit-tests unit-tests.o factorial.o square.o
    ```

    **In the CDFF**&emsp;This task is performed by CMake. Follow the instructions in `Tests/UnitTests/CMakeLists.txt`.

4. **In general**&emsp;Unit tests are ready to be run with:

    ```shell
    $ ./unit-tests
    ```

    **In the CDFF**&emsp;The unit test executable is `Tests/UnitTests/cdff-unit-tests`. The CMake build system is parameterized so that it is also available through CTest (see ctest(1)), but by default CTest reports less.

    ```shell
    /path/to/CDFF/build/Tests/UnitTests:~$ ./cdff-unit-tests

    /path/to/CDFF/build:~$ ctest [--verbose]
    /path/to/CDFF/build:~$ make test # same as ctest --force-new-ctest-process
    ```

    Use options `-?, -h, --help` with `cdff-unit-tests`, and [this documentation page](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/command-line.md), for usage information.

    **Warning:** the test executable can only be run from its own directory; it fails if you run `/path/to/cdff-unit-tests`. This is a bug that needs fixing.

### How to write tests? What are the available assertion macros?

See the [reference documentation](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/Readme.md), in particular the pages about:

* [Assertion macros](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/assertions.md)
* [Test cases and sections](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/test-cases-and-sections.md)
* [Matchers](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/matchers.md)

### Usage with CMake

Integration of Catch with CMake has been sorted out. You shouldn't have to change anything. See [this documentation page](https://github.com/catchorg/Catch2/blob/Catch1.x/docs/build-systems.md) for what was done.

## Catch2

The aforementioned links point to files in the 1.x branch (C++03) of Catch's code repository. The [master branch](https://github.com/catchorg/Catch2) is for version 2.x (C++11, C++14, C++17) and has documentation in similar files. We are using version 1.x at the moment.
