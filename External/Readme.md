## Externals (aka 3rd_party libraries)
### Recommendation
It is recommended you install "checkinstall". This helps keeping track of manually installed dependencies.

### Automatic script

In order to install the required dependencies run
```
$ CDFF/External/fetch_compile_install_dependencies.sh
```
This will install all the required dependencies into /External/install.
This will *not* override your system libraries.  
The script fetches the sources, compiles and installs them for each dependency neede by Infuse

To only install a specific dependency run :
```
$ CDFF/External/fetch_compile_install_dependencies.sh -s pcl
Found infuse installers for : eigen tinyxml2 cmake pcl opencv vtk qhull yaml-cpp flann boost
Dependencies that will be BUILT : pcl
build directory    = PATH_TO_CDFF/External/build
install directory  = PATH_TO_CDFF/External/install
Packages directory = PATH_TO_CDFF/External/package
```
fetch_compile_install_dependencies.sh ? provides help.

### Warning !
The root CMakeLists.txt uses FindPkgConfig to find current dependencies.
If your distribution already provides system libraries, FindPkgConfig might pick them instead of the libraries under /Externals.

In a close future, an environnement variable will be used to correct this on mixed systems.   

## Current dependencies :
- Boost 1.66.0
- eigen 3.3.4
- flann 1.9.1
- opencv 3.4.0
- pcl 1.8.1
- qhull (master branch)
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
