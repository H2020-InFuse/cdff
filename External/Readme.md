# Dependencies (third-party libraries)

The CDFF currently requires the following libraries (**direct dependencies**):

* yaml-cpp 0.5.3
* Boost 1.66.0
* Eigen 3.3.4
* FLANN 1.9.1
* QHull (master branch)
* VTK 8.1.0
* PCL 1.8.1
* OpenCV 3.4.0

These libraries in turn have their own dependencies (the CDFF's **recurse dependencies**). List under construction:

* Boost:
  - The C/C++ standard libraries (`apt:libc6-dev` `apt:libstdc++-5-dev`)
  - The GCC support library (`apt:libgcc-5-dev`)
  - For certain Boost.* libraries: OpenSSL, OpenGL, Python v2.2+, zlib (`apt:zlib1g-dev`), libbzip2 (`apt:libbz2-dev`), liblzma (`apt:liblzma-dev`), ICU (`apt:libicu-dev`)
* Eigen:
  - Only the C++ standard library
* PCL:
  - Boost, Eigen, FLANN, QHull, VTK
* The others:
  - Check their online documentation and report here

Also note that a working basic development environment is required for building the CDFF of course:

* The GNU Toolchain
  - GNU Compiler Collection (`apt:gcc`, `apt:g++`) and GNU Make (`apt:make`) can be installed as dependencies of the `build-essential` metapackage (`apt:build-essential`)
  - You may be interested in the GNU Debugger too (`apt:gdb`), however the GNU Build Tools aka automake (`apt:automake`, `apt:autoconf`, `apt:libtool`) aren't necessary as we use CMake instead
* CMake (`apt:cmake`)
  - Only CMake 3.5.1 provided by Ubuntu 16.04 has been tested, but previous versions may very well work, within a limit. To try building the CDFF with a previous version of CMake, change the requested version number on the first line of the top-level `CMakeLists.txt` file. Report here whether it works or not.
* The C and C++ standard libraries (*a C standard library is installed by `apt:build-essential` anyway but do we actually depend on it? I don't think so, but we probably recurse-depends on it though*), for instance:
  - The GNU C Library aka glibc provides an implementation of the C standard library and the standard math library (`apt:libc6`, `apt:libc6-dev`)
  - The GNU Standard C++ Library aka libstdc++ provides an implementation of the C++ Standard Library and the Standard Template Library (STL) (`apt:libstdc++6`, `apt:libstdc++-5-dev`)

The dependency installation script (see next subsection) additionally requires `apt:git`, `apt:wget`, `apt:curl`, `apt:bash`, `apt:tar`, `apt:cmake` at the moment.

## Install the CDFF's dependencies on your computer

A helper script is provided in `External` to automatically download the sources of all the CDFF's direct dependencies, build them, and install the result under `External/install` by default, subdirectories `bin`, `include`, `lib`, `share`. You can do the same manually of course and get the same result.

The script doesn't check whether the CDFF's recurse dependencies are available, and it is your responsability to ensure that they are present on your computer. You can simply install them with your distribution's package manager. It is rarely clear, from the documentation of a library, whether it requires only the runtime libraries of its dependencies (usually `apt:<packagename><version>`) or also their headers (usually `apt:<packagename>-dev`), so perhaps install both to be in the clear.

**Warning:** the helper script has only been tested with `bash` on Ubuntu 16.04 for now.

### Step 1: optionally install CheckInstall

CheckInstall (`apt:checkinstall`) is a program that monitors the installation phase of a normal software build process (the `make install` phase), notes the files that are added to the system, builds a software package that contains these files (`.deb` archive), removes the files installed by the original installation run, and finally installs the created package using the system package management tools (`apt-get`, `dpkg`), so that the software package will be properly registered in the local installed packages database.

The main interest of CheckInstall is that it makes it possible to later remove software compiled from source by running `apt-get` or `dpkg` on the CheckInstall-created package. Without it, one would need to manually remove the installed software from where it has been copied in the filesystem hierarchy: this is easy enough if everything is in `/opt/softwarename`, but is tedious if the software's components have been divided into the `bin`, `include`, `lib`, `share` of `/usr/local`. Some software provides an `uninstall` target in their `Makefile` to make it easier, but that is not always the case.

The current default of the provided helper script is to install all compiled software into `External/install/{bin,include,lib,share}` in the CDFF's source tree. Nothing is installed in your system directories not even in the local hierarchy `/usr/local`. So CheckInstall isn't really necessary: if you don't have it, you can simply delete the whole `External/install` directory to uninstall all the CDFF's direct dependencies (if you have it you ought to use `apt-get` or `dpkg` though, to update your local package database).

However, if installing manually somewhere else, you may want to consider using CheckInstall. **Warning:** this has not been thoroughly tested yet, only installation into default `External/install` has been tested and is currently recommended.

### Step 2: download, build, and install the direct dependencies

Download, build, and install the CDFF's direct dependencies in `External/install/{bin,include,lib,share}` by default:

```
/path/to/CDFF/External$ ./fetch_compile_install_dependencies.sh
```

As explained in "Step 1" above, nothing is installed in your system directories not even in the local hierarchy `/usr/local` (where you may already have put the same libraries in different versions for your other work). If you have CheckInstall though, your system's package database is updated to include the CheckInstall-created packages.

Use option `-s` to only install a specific dependency (e.g. OpenCV but not the PCL), and use option `-h` for help.

## Or use a Docker image

The CDFF's direct dependencies are ready to use in the Docker image `nexus.spaceapplications.com/repository/infuse/cdff-ci:<version>` and in all the images built on top of it, in particular `h2020infuse/cdff-core:latest`. They have been compiled from source and installed in the image's `/usr/local` (which is otherwise empty) (**warning:** images currently being rebuild, don't use today).

The Docker images *should* also provide the CDFF's recurse dependencies but I am afraid we missed some of them. In particular, no widget toolkit is available in these images (e.g. `apt:libgtk-3-dev` or `apt:libgtk2.0-dev`) which means that OpenCV was compiled without any GUI features (e.g. without support for `cv::imshow()`). Until we fix that, whenever you span a container from a Docker image, you can workaround that missing recurse dependency by installing it yourself with `apt-get` and recompile OpenCV (sorry for the bother).

## How does the CDFF's build process find the CDFF's dependencies?

The project's top-level `CMakeLists.txt` file currently uses a combination of:

* CMake-provided find modules (`Find<package>.cmake`)
* Package-provided configuration files (`<package>Config.cmake`)
* Hard-coded paths (`/usr/local` by default, `External/install` if `USE_BUNDLED_DEPENDENCIES=ON`), *will be removed in a future version*

to search for what CMake calls "packages" and which are in our case libraries, those of the CDFF's dependencies. Depending on the value of the CMake cache variable `USE_BUNDLED_DEPENDENCIES`, by default `OFF`, the search paths are adjusted so that the libraries in the source tree under `External/install` are found after or before those in the system libraries in particular those in the local hierarchy `/usr/local`.

The supported use cases are:

* **building the CDFF inside a Docker container**:
  - libraries in `/usr/local`
  - build the CDFF with `cmake /path/to/top-level/directory`

* **building the CDFF without Docker**:
  - libraries in `External/install`
  - build the CDFF with `cmake -D USE_BUNDLED_DEPENDENCIES=ON /path/to/top-level/directory`

An example unsupported use case is:

* **unsupported and untested**:
  - libraries in `/usr` (installed using your distribution's package manager) or in `/usr/local` (installed from source, manually or by passing option `-i /usr/local` to the dependency installation script)
  - build the CDFF with `cmake /path/to/top-level/directory`

# Add a new CDFF dependency

At the start of the implementation activities, we made a list of the expected direct dependencies for the CDFF, so adding new dependencies should only be necessary if it turns out that we have forgotten a dependency that can't be otherwise satisfied (by a module of one of the existing dependencies, for instance).

To add a new dependency, you need to write a function called `install4infuse_<lib>()` in a bash-compatible script called `External/installers/<lib>.sh`, where `<lib>` is your new dependency. That function is an installation scripts that downloads, unpacks, builds, and install the new dependency. Have a look at the other files for templates.

**Extensively document your script** and please notify all the partners.

# Remove git submodules from your repository

The CDFF's dependencies used to be provided through git submodules. In case your repository still features a `third_party` directory and git submodules, you should remove them:

1. Delete the relevant line from the `.gitmodules` file.
2. Delete the relevant section from the `.git/config` file.
3. Run `git rm --cached path_to_submodule` (without any trailing `/`).
4. Commit to your local repository and delete the now untracked submodule files.

Or you can just clone a fresh copy of the repository from the `origin` remote, if you have pushed all your current work to it and haven't got any outstanding local branches (e.g. you've pushed them all to the `origin` remote, had them all merged into the `origin`'s master, and subsequently removed them from your local repository).
