## Externals (aka 3rd_party libraries)
The third_party dependencies have been reworked.

Submodules are no more a part of this repository.

In order to install the required dependencies:
Run fetch_compile_install_dependencies.sh.
This will install all the required dependencies into /External/install.
This will *not* override your system libraries.  

To only install a specific dependency run
fetch_compile_install_dependencies.sh -s XXXX specific library



### Warning !
For the moment, in the CMakeLists.txt, the include(FindPkgConfig) has been disabled, hence, only libraries under /Externals will be searched for and used in INFUSE.

If your Distribution comes already with some dependencies, they will not be used.
Mixed Configuration is not encouraged, but if you like fire, uncomment the lines and start having fun with "dll hell".

## Current dependencies :
- Boost 1.61.0
- eigen 3.3.4
- flann 1.9.1
- opencv 3.4.0
- pcl 1.8.1
- qhull (master branch)
- tinyxml2 6.0.0
- vtk 8.1.0
- yaml-cpp 0.5.3

## Adding new Dependencies
You need to add a script with .sh extension in the /installers folder.
The script needs to contain a function with the signature install4infuse_XXXX where XXXX is your new dependency

## Removing old submodules from your repository
1. Delete the relevant line from the .gitmodules file.
2. Delete the relevant section from .git/config.
3. Run git rm --cached path_to_submodule (no trailing slash).
4. Commit and delete the now untracked submodule files.
