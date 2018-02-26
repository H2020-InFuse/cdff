## Externals (aka 3rd_party libraries)
In order to install the required dependencies:
Run fetch_compile_install_dependencies.sh. from the /External folder, 
This will install all the required dependencies into /External/install.
This will *not* override your system libraries.  

To only install a specific dependency run
fetch_compile_install_dependencies.sh -s XXXX specific library

### Warning !
The root CMakeLists.txt uses FindPkgConfig to find current dependencies. 
If your distribution already provides system libraries, FindPkgConfig might pick them instead of the libraries under /Externals. 

In a close future, an environnement variable will be used to correct this on mixed systems.   

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
The third_party dependencies have been reworked.
Submodules are no more a part of this repository. Should you need to clean your old local repository here is the procedure : 

1. Delete the relevant line from the .gitmodules file.
2. Delete the relevant section from .git/config.
3. Run git rm --cached path_to_submodule (no trailing slash).
4. Commit and delete the now untracked submodule files.
