## Dependencies (third-party libraries)

The following dependencies are already installed in the Docker images `h2020infuse/cdff:latest` and `nexus.spaceapplications.com/repository/infuse/cdff-ci:<version>`.

### Libraries

The CDFF currently requires the following libraries (**direct dependencies**):

* yaml-cpp 0.5.3
* Boost 1.66.0  
  Only a couple Boost.* libraries are currently required, see [`boost.sh`](/Tools/Docker/installers/boost.sh) for a list
* Eigen 3.3.4
* FLANN 1.9.1
* VTK 8.1.0  
  It's only required by `pcl_visualization` for 3D point cloud rendering and visualization, and we don't need that module on our target systems, so what can we do about it?
* PCL 1.8.1
* OpenCV 3.4.0
* Ceres 1.14.0

These libraries in turn have their own dependencies (the CDFF's **recurse dependencies**). List under construction:

* yaml-cpp
  - Certain Boost.* libraries, but precisely which ones is undocumented (assumedly, header-only ones)

* Boost
  - The C/C++ standard libraries (`apt: libc6-dev libstdc++-5-dev`)
  - The GCC support library (`apt: libgcc-5-dev`)
  - For the Boost.Iostream library (a dependency of PCL): zlib, libbzip2, liblzma (`apt: zlib1g-dev libbz2-dev liblzma-dev`)

* Eigen
  - Only the C++ standard library

* PCL
  - A few Boost.* libraries, see [`boost.sh`](/Tools/Docker/installers/boost.sh) for the list
  - Eigen, FLANN, VTK
  - QHull is an optional dependency of `pcl_surface` and `pcl_surface` isn't currently used by the CDFF

* OpenCV
  - FFmpeg development packages are required for video IO to work (`apt: libavcodec-dev libavformat-dev libswscale-dev`)
  - A widget toolkit, including headers, is required for GUI features to work: GTK+ 2 (`apt: libgtk2.0-dev`) or GTK+ 3 (`apt: libgtk-3-dev`) or Qt (`apt: qtbase5-dev`)
  - Git and pkg-config are required for some reason (`apt: git pkg-config`)
  - Python 2.6+ and NumPy 1.5+ with development packages are required but assumedly only for using OpenCV's Python API (`apt: python-dev python-numpy` or perhaps `apt: python3-dev python3-numpy`)
  - Build tools: GNU Compiler Collection 4.4+ and CMake 2.8.7+ (`apt: build-essential cmake`)
  - Optional dependencies, most of them we probably don't need: `apt: libdc1394-22-dev libv4l-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libxine2-dev` and probably others for video IO, `apt: libjpeg-dev libpng12-dev libtiff5-dev libjasper-dev libwebp-dev libopenexr-dev libgdal-dev zlib1g-dev` and probably others for media IO, `apt: libtbb2 libtbb-dev` for parallel programming but I'm pretty sure we don't want to allow that in the CDFF

* [Ceres](http://ceres-solver.org/installation.html)
  - Required: Eigen 3.1.0+, CMake 2.8.0+
  - Recommended: Eigen 3.2.2+, glog 0.3.1+ (`apt: libgoogle-glog-dev`), SuiteSparse (`apt: libsuitesparse-dev`)
  - Optional but required for SuiteSparse: BLAS and LAPACK, both provided by ATLAS for instance (`apt: libatlas-base-dev`)
  - Optional: gflags to build examples and tests (`apt: libgflags-dev`), Threading Building Blocks (`apt: libtbb-dev`) to have multithreading support provided by Threading Building Blocks instead of OpenMP or C++11 primitives (all multithreading is off by default)

* The others
  - Check their online documentation and `CMakeLists` files and report here

### Build tools

Also note that a working basic **development environment** is of course required for building the CDFF:

* The GNU Toolchain
  - GNU Compiler Collection (`apt: gcc g++`) and GNU Make (`apt: make`) can be installed as dependencies of the `build-essential` metapackage (`apt: build-essential`)
  - You may be interested in the GNU Debugger too (`apt: gdb`)
  - However the GNU Build Tools aka automake (`apt: automake autoconf libtool`) are not necessary as we use CMake instead

* CMake (`apt: cmake`)
  - Only CMake 3.5.1 provided by Ubuntu 16.04 has been tested, but previous versions may very well work, within a limit. To try building the CDFF with a previous version of CMake, change the requested version number on the first line of the top-level `CMakeLists.txt` file. Report here whether it works or not.

* The C and C++ standard libraries, for instance:
  - The GNU C Library aka glibc provides an implementation of the C standard library and the standard math library (`apt: libc6 libc6-dev`)
  - The GNU Standard C++ Library aka libstdc++ provides an implementation of the C++ Standard Library and the Standard Template Library (STL) (`apt: libstdc++6 libstdc++-5-dev`)

### Additional dependencies

Downloading the compiled ASN.1 data types from Space Applications' GitLab server requires cURL (`apt: curl`). Alternatively, compiling the ASN.1 data types requires the [ASN1SCC](https://github.com/ttsiodras/asn1scc) compiler and its dependencies, among which Mono.

The dependency installation script (see further) additionally requires Bash, GNU Core Utilities, Git, GNU Wget, cURL, GNU tar, CMake at the moment (`apt: bash coreutils git wget curl tar cmake`).

## Option 1: get the CDFF's dependencies packaged in a Docker image

### Available images

The CDFF's direct dependencies are ready to use in the Docker images `h2020infuse/cdff:latest` and `nexus.spaceapplications.com/repository/infuse/cdff-ci:<version>`. They have been compiled from source and installed in the images' `/usr/local` (which is otherwise empty).

* `h2020infuse/cdff:latest` is meant for helping you build and test the CDFF's core and support components on your computer. It includes a version of OpenCV compiled with GTK+ 2 support so that the GUI features of the `cv::highgui` module work. It can also be used for using the CDFF's dev component (as an alternative to installing it on your computer).

* `nexus.spaceapplications.com/repository/infuse/cdff-ci:<version>` is meant for the continuous integration pipeline on the GitLab server. GUI features are not available. In that sense, it is an environment that is closer to our target systems than `h2020infuse/cdff:latest` and our desktop computers are.

The recurse dependencies have been installed from Ubuntu 16.04's software package repositories using the system's package management tools (both images are build on Ubuntu 16.04), prior to compiling the direct dependencies of course. Please let us know if you spot a dependency we forgot. All the above-mentioned build tools are also available in the images.

### Usage

We have documentation about [using Docker and the InFuse Docker image](https://drive.google.com/open?id=1aW3_giavOZdvOljEEfun4W0Cq2tlnDvb8S3y2bysjpw). The shortest possible summary is:

1. Make sure you have the most up-to-date Docker image (list [here](https://hub.docker.com/r/h2020infuse/cdff/)):

    ```shell
    $ sudo -H docker pull h2020infuse/cdff:latest
    ```

2. Create a Docker container from that image, mounting your code repository inside it at the same time.

    It is worth defining an alias for this long command in one of your shell startup files, for instance `~/.bashrc` or `~/.bash_aliases` if you use `bash`. Have a look at the file [`Tools/Docker/docker_aliases.template`](/Tools/Docker/docker_aliases.template) for a suggested solution that you can copy-paste into one of your shell startup files.

    ```shell
    $ sudo -H docker run

      # Optional: your choice of identifiers
      --name=container-name --hostname=container-hostname \

      # Optional: if you need graphics display
      --env=DISPLAY --volume=/tmp/.X11-unix:/tmp/.X11-unix \

      # Optional: allow messages from inside the container to be logged outside
      --volume=/dev/log:/dev/log \

      # Optional: allow breakpoint debugging
      --cap-add=SYS_PTRACE \

      # Optional: open port for debugging from outside the container
      --publish=2159:2159 \

      # Recommended: be the same user inside your container as outside
      --volume=/etc/passwd:/etc/passwd:ro --volume=/etc/group:/etc/group:ro \
      --volume=/etc/shadow:/etc/shadow:ro --volume=/etc/gshadow:/etc/gshadow:ro \
      --user=$(id -u):$(id -g) \

      # Recommended: use an init process and delete your container when you exit it
      --init --rm \

      # Mandatory: mount your local CDFF-core+support repository inside your container
      --volume=/absolute/path/to/CDFF:/where/i/want/it/in/the/container \

      # Optional: also mount your local CDFF-dev repository if you want to use CDFF-dev
      --volume=/absolute/path/to/CDFF-dev:/where/i/want/it/in/the/container \

      # Mandatory: interactively use a terminal in your container
      --interactive --tty \

      # And finally: image to use and shell to start (default bash)
      h2020infuse/cdff:latest [bash]
    ```

    or with an adequately-defined alias:

    ```shell
    $ docker cdff [--name=container-name] [--hostname=container-hostname]
    ```

You can then build and test the CDFF. See the [main readme file](/README.md#download-and-compile) at the root of the repository for information on how to do that.

## Option 2: install the CDFF's dependencies yourself on your computer

### Guidelines

You can use a combination of your distribution's software package manager and/or compiling from source to install the CDFF's dependencies and the necessary build tools on your own computer.

* **Direct dependencies**

    It is currently **recommended** to compile the CDFF's direct dependencies from source, rather than relying on the precompiled versions available in your distribution's software package manager. The reason for that is that they are specialist libraries (mathematics, robotics, computer vision) that the CDFF relies heavily on, and relying on the compilation and packaging choices of a particular package maintainer can be uncertain: those choices are often different for different distributions, as is the version number too. Our current policy is to **use the official upstream sources in fixed versions and choose our own compilation options**.

    That said, you are free to install these dependencies from whatever source you prefer in whatever version you want, and link the CDFF against them. In that case, please make sure that the features you contribute to the CDFF also work as you intend in a CDFF linked against direct dependencies compiled from source in the versions listed above. You can use the `h2020infuse/cdff` Docker image for that: the whole point of that image is to provide a common reference environment. Please also be aware that the `nexus.spaceapplications.com/repository/infuse/cdff-ci:<version>` image used to run the continuous integration pipeline on the GitLab server also uses direct dependencies compiled from source in the versions listed above.

    A helper script is provided in `External/`. It downloads the sources of the CDFF's direct dependencies, builds them, and installs the result in `External/install/{bin,include,lib,share}` by default. Of course you can do all that manually if you prefer. In that case, check out our currently-selected build options in the various `External/installers/*.sh` scripts.

    **Warning:** the helper script requires `bash` and has only been tested on Ubuntu 16.04 for now. It very probably works on other GNU/Linux distributions, but would require some adaptation to work on macOS and BSD distributions.

* **Recurse dependencies**

    If you want to build the direct dependencies from source (whether manually or using the aforementioned helper script), you will first have to install their own dependencies. You can do that with your distribution's package manager. Since you are compiling, you will need the headers (usually `apt: <packagename>-dev`) in addition to the shared libraries (usually `apt: <packagename><version>`).

    If you install precompiled software packages for the direct dependencies, your distribution's package manager will also install their dependencies (just the shared libraries, not their headers).

* **Build tools and C/C++ standard libraries**

    Use your distribution's package management tools to get them from your distribution's software repositories.

### Installing the direct dependencies from source, step 1: optionally install CheckInstall

CheckInstall (`apt: checkinstall`) is a program that monitors the installation phase of a normal software build process (the `make install` phase), notes the files that are added to the system, builds a software package that contains these files (`.deb` archive), removes the files installed by the original installation run, and finally installs the created package using the system package management tools (`apt: dpkg`), so that the software package will be properly registered in the local database of installed packages.

The main advantage of CheckInstall is that using it makes it possible to later **uninstall locally-compiled software** by running `apt-get remove` or `dpkg --remove` with the name of the CheckInstall-created package as argument. In contrast, when locally-compiled software has been installed out of CheckInstall supervision, uninstalling that software means manually removing its files from where they have been copied. This is easy enough if everything is in `/opt/softwarename/`, but it can be tedious if the software's components have been divided into `/usr/local/{bin,include,lib,share}`. Some software provides an `uninstall` target in their `Makefile` to make it easier, but that is not always the case.

The current default of the aforementioned helper script is to install all the software it compiles into `External/install/{bin,include,lib,share}`. Nothing is installed in your system directories not even in the local hierarchy `/usr/local/`.

* In that default configuration, CheckInstall is convenient but not really necessary:
  - If you don't have it, you can simply delete the whole `External/install/` directory to uninstall all the CDFF's direct dependencies.
  - If you have it, you ought to use `apt-get remove` or `dpkg --remove`, so that the packages you remove are written as such in your system's package database.

* You should really consider using CheckInstall if you decide to not follow the default and install the locally-compiled direct dependencies in your system directories.  
  **Warning:** only `/usr/local/` has been tested as an alternative installation prefix for now.

**Warning:** since CheckInstall writes to your system's package database, it needs to be run as `root`: you can either run the whole helper script as `root`, or edit it to replace the line that states `checkinstall` by `sudo chekinstall` (but the script will stop to ask for your password, possibly on several occasions).

### Installing the direct dependencies from source, step 2: download, build, and install

Download, build, and install the CDFF's direct dependencies in `External/install/{bin,include,lib,share}` by default:

```shell
/path/to/CDFF/External$ [sudo] ./fetch_compile_install_dependencies.sh
```

As explained in "step 1" above, nothing is installed in your system directories not even in the local hierarchy `/usr/local/`, where you may already have put the same libraries in different versions for your other work. CheckInstall is used if it is available, in which case you need `sudo`, see previous **warning**.

Use option `-s <dependency-in-lower-case>` to only install a specific dependency (e.g. OpenCV but not the PCL). Use option `-h` for help.

## How does the CDFF's build process find the CDFF's dependencies?

The project's top-level `CMakeLists.txt` file currently uses a combination of:

* CMake-provided find modules (`Find<package>.cmake`)
* Library-provided configuration files (`<package>Config.cmake`)
* Hard-coded paths (`/usr/local/` by default, `External/install/` if `USE_BUNDLED_DEPENDENCIES=ON`), *will be removed in a future version*

to search for what CMake calls "packages" and which are in our case libraries, those of the CDFF's dependencies. Depending on the value of the CMake cache variable `USE_BUNDLED_DEPENDENCIES`, by default `OFF`, the search paths are adjusted so that the libraries in the source tree under `External/install/` are found after or before those in the system directories in particular those in the local hierarchy `/usr/local/`. The tested and supported usage is currently:

* **Build the CDFF inside a Docker container**
  - Libraries in `/usr/local/`
  - Build the CDFF with `cmake /path/to/CDFF/`

* **Build the CDFF without using Docker**
  - Libraries in `External/install/`
  - Build the CDFF with `cmake -D USE_BUNDLED_DEPENDENCIES=ON /path/to/CDFF/`

See the [main readme file](/README.md) at the root of the repository for more information.

## How to add a new CDFF dependency?

At the start of the implementation activities, we made a list of the expected direct dependencies for the CDFF, so adding a new dependency should only be necessary if it turns out that we have forgotten it.

To add a new dependency, you need to write a function called `install4infuse_<lib>()` in a Bash-compatible script called `External/installers/<lib>.sh`, where `<lib>` is your new dependency. That function is an installation script that downloads, unpacks, builds, and installs the new dependency. Have a look at the other files for templates.

**Extensively document your script** and please notify all the partners.

## Remove Git submodules from your repository

The CDFF's dependencies used to be provided through Git submodules. In case your local repository still features a `third_party` directory and Git submodules, you should remove them:

1. Delete the relevant line from the `.gitmodules` file.
2. Delete the relevant section from the `.git/config` file.
3. Run `git rm --cached path_to_submodule` (without any trailing `/`).
4. Commit to your local repository and delete the now untracked submodule files.

Or you can just clone a fresh copy of the repository from the `origin` remote, if you have pushed all your current work to it and haven't got any local branches you'd like to keep working on (for instance if you've pushed all your branches to the `origin` remote and had them all merged into the `origin`'s master).
