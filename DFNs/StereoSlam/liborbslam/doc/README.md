# How to contribute to the documentation
We use [Doxygen](http://www.stack.nl/~dimitri/doxygen/index.html) for documentation. You can add code-level documentation directly inside the source files, and any other information in separate pages. 

## Generate the documentation
Edit your documentation in the code and in separate `doxy-<page>.h` files as you like. 
Then, in the root directory of your project: 
``` 
mkdir build
cd build
cmake ..
make doc 
``` 
Finally, open the documentation page in your browser at `{PROJECT_ROOT}/doc/html/index.html`.

Check out the generated documentation of the CMakeTemplate project for a complete example. 

## Documentation Rules
* All direct source code documentation should be organized within Doxygen Modules, using the `\defgroup`, `\addtogroup` and `\ingroup` commands.
* All other pages (main page, install, tutorials, manuals, etc.), declared with the `\page` command, must be a `\subpage` declared inside the `\mainpage`. This makes sure everything is easily accessible to users, and preserves the project structure if your project is one day included in another as a subproject. 
* Follow the default convention :
    * Put your `\mainpage` in the file `doxy-main.h`
    * For other pages, one `\page` per file, with the name `doxy-<name>.h`
    * Declare your groups in `doxy-groups.h`
* For your `\mainpage`, make sure you keep the `if...else` macro at the beginning of the `doxy-main.h` file. This enables recursive generation of the documentation of your submodules. 
* As much as possible, use unique names in your sections and pages e.g. `\section \cmaketemplate_HowToBegin` instead of just `\section \HowToBegin`. This will help preserve documentation structure and avoid name clashes if you add submodules to your project.

## Implementation details
To generate the documentation, Doxygen needs a Doxyfile. In our projects built on CMakeTemplate, we can add multiple levels of independent documentations for each submodules in the `modules` folder. 
The documentation is generated recursively for each submodule by the top-level project when we call `make doc`. 
In order to tell Doxygen which documentation is the main project and organize the lower levels, we use the CMakeLists.txt to automatically generate a Doxyfile (from the Doxyfile.in file). CMake configure the Doxyfile by replacing the configuration variable `ENABLED_SECTIONS  = @ROOT_PROJECT@` in Doxyfile.in with the name of the main project. This works together with the `\if...else` conditions we put around the `\mainpage` commands to create a tree-like structure with the submodules. 

## Resources
* [Doxygen Documentation](http://www.stack.nl/~dimitri/doxygen/manual/index.html) : How to use Doxygen.

