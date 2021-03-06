[![build status](
https://gitlab.spaceapplications.com/InFuse/CDFF/badges/master/build.svg)](
https://gitlab.spaceapplications.com/InFuse/CDFF)

# CDFF

This is the code repository for the Core and Support components of the Common Data Fusion Framework (CDFF) developed by the InFuse consortium. [Another repository](https://gitlab.spaceapplications.com/InFuse/CDFF_dev), named `CDFF_dev`, contains the Dev component.

* The Core and Support components are those that must be deployed in your final robotic target system. It is released under the terms of the [2-Clause BSD license](https://opensource.org/licenses/BSD-2-Clause).

* The Dev component is a development environment containing tools to develop and test data fusion solutions, and visualize and analyze data fusion products. It is released under the terms of the [GNU General Public License version 3](https://opensource.org/licenses/GPL-3.0) or any later version.

## Building the CDFF (Core and Support)

### Download and compile

Download the code for the CDFF's Core and Support components, for instance:

```shell
~/InFuse$ git clone git@gitlab.spaceapplications.com:InFuse/CDFF.git
```

To develop a Data Fusion Node (DFN) or a Data Fusion Processing Compound (DFPC), or more generally to use the tools provided by the CDFF's Dev component, you should also get that component:

```shell
~/InFuse$ git clone git@gitlab.spaceapplications.com:InFuse/CDFF_dev.git
```

There are two ways you can build the CDFF (Core and Support components):

* You can compile it using CMake and a C++ compiler **in a Docker container**.

    We provide a Docker image based on Ubuntu 16.04 where build tools and the CDFF's dependencies are pre-installed. You can spawn a container from that image, mount the CDFF's code inside it, and compile. That image, and the libraries it contains, make up a common **reference environment** for the CDFF.

    We have documentation about [using Docker and the InFuse Docker image](https://drive.google.com/open?id=1aW3_giavOZdvOljEEfun4W0Cq2tlnDvb8S3y2bysjpw), however perhaps start with [this short section](/External/README.md#option-1-get-the-cdffs-dependencies-packaged-in-a-docker-image) in the documentation about the dependencies of the CDFF.

    The TL;DR is (seriously, read the documentation though, not all these options are necessary all the time):

    ```shell
    $ docker pull h2020infuse/cdff:latest
    $ docker run \
      --name=cdff --hostname=cdff \
      --env=DISPLAY --volume=/tmp/.X11-unix:/tmp/.X11-unix \
      --volume=/dev/log:/dev/log \
      --cap-add=SYS_PTRACE --publish=2159:2159 \
      --volume=/etc/passwd:/etc/passwd:ro --volume=/etc/group:/etc/group:ro \
      --volume=/etc/shadow:/etc/shadow:ro --volume=/etc/gshadow:/etc/gshadow:ro \
      --user=$(id -u):$(id -g) \
      --init --rm \
      --volume=/absolute/path/to/CDFF:/where/i/want/it/in/the/container \
      --volume=/absolute/path/to/CDFF-dev:/where/i/want/it/in/the/container \
      --interactive --tty \
      h2020infuse/cdff:latest [bash]
    ```

    Once you have mounted the directory containing the CDFF's source code inside your Docker container, you can build the CDFF (Core and Support):

    ```shell
    /path/to/CDFF/build$ cmake [-D CMAKE_INSTALL_PREFIX=/path/to/CDFF/install/] /path/to/CDFF/
    /path/to/CDFF/build$ make
    ```

* You can compile it using CMake and a C++ compiler **in a directory on your computer**.

    In that case, you must have installed the **dependencies** of the CDFF yourself on your computer, and the dependencies of these dependencies, and the necessary build tools. To do so, you can use your distribution's software package manager and/or compile from source. The dependencies you compile from source do not need to be installed in your system's local hierarchy (`/usr/local/`): you may want to install them there, or you may prefer to put them in a dedicated directory next to the CDFF's source (`External/install/`), which is the **recommended** location for now as it makes uninstalling them easy.

    Downloading the first-level dependencies is easy using the included `get-cdff-dependencies.sh` script. You must first install a handful of recurse dependencies through the package manager.

    ```shell
    $ sudo apt install build-essential cmake wget curl \
                       liblzma-dev libbz2-dev zlib1g-dev \
                       libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev \
                       libavcodec-dev libavformat-dev libswscale-dev libjpeg-dev \
                       libpng++-dev libjasper-dev libtiff5-dev libv4l-dev freeglut3-dev \
                       libxt-dev #  for VTK
    $ /path/to/CDFF/External/get-cdff-dependencies.sh
    ```

    This might take up to an hour. If you choose this route remember to pass `-D USE_BUNDLED_DEPENDENCIES=ON` to CMake when building.

    The documentation about the [dependencies of the CDFF](/External/README.md) describes this topic in more detail.

    Once you have installed all the dependencies and build tools, you can build the CDFF (Core and Support):

    ```shell
    /path/to/CDFF/build$ cmake                           \
        [-D USE_BUNDLED_DEPENDENCIES=ON]                 \
        [-D COMPILE_ASN1=OFF]                            \
        [-D CMAKE_INSTALL_PREFIX=/path/to/CDFF/install/] \
        /path/to/CDFF/
    /path/to/CDFF/build$ make
    ```

Paths and options:

* `/path/to/CDFF/`: Directory where the CDFF's top-level `CMakeLists.txt` file is located.

* `/path/to/CDFF/build/`: Directory where you would like to build the CDFF.
  - For instance: a subdirectory named `build` in `/path/to/CDFF/` results in the commonplace `cmake ../` invocation.
  - Default: this is always the current directory.

* `CMAKE_INSTALL_PREFIX`: Directory where you would like to optionally install the CDFF (in `{bin,lib}` subdirectories).
  - **Warning** don't even try to install for now, our current `make install` rule is very incomplete, sorry. Only building works, not installing. Instructions in this bullet point are provided for future usage only.
  - For instance: `$HOME/.local/` is specified by [systemd's file-hierarchy specification](https://www.freedesktop.org/software/systemd/man/file-hierarchy.html), an extension of the [XDG Base Directory specification](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html), so `$HOME/.local/bin` is normally on the `PATH` of all systemd-compatible distributions.
  - Default: `/usr/local/`, and `/usr/local/bin` is on the `PATH` in all known distributions.
  - **Not recommended** to use the default for now, because I'm not sure we have a `make uninstall` rule.
  - **Recommended** (if you install at all) to install somewhere that you can easily `rm -r`, for instance a ad-hoc `/path/to/CDFF/install/`.

* `USE_BUNDLED_DEPENDENCIES`: Whether or not to look for the CDFF's direct dependencies (Boost, Eigen, PCL, OpenCV...) in `External/install/{bin,include,lib,share}` before looking for them in `/usr/local/{bin,include,lib,share}`.
  - Default: `OFF`, look directly in `/usr/local/`.
  - **Recommended** to use the default value when building inside an InFuse Docker container.
  - **Recommended** to install those dependencies in `External/install/` if you're not using Docker, and therefore recommended to use `ON` for that option.

* `COMPILE_ASN1`: Compile the ASN.1 data types on your machine (requires a working installation of Mono, with the Core and Numerics libraries), or download precompiled data types (compiled by the CI server).
  - Default: `ON`, compile them locally.
  - **Recommended** to use the default value when building inside an InFuse Docker container.

CMake variables given using the `-D` option are written to the CMake cache (`/path/to/CDFF/build/CMakeCache.txt`) and therefore don't need to be given on subsequent CMake runs. CMake will read their value in the cache. If you want to change a CMake cache entry, it is safer to delete the cache in addition to giving the new value on the command line. This makes sure that all entries whose value depends on the changed entry are regenerated, instead of being read from the cache.

### ASN.1 data types

CMake checks whether there is a directory called `Common/Types/C/`, meant to contain `.h` and `.c` files generated ("compiled") from the ASN.1 data types present in `Common/Types/ASN.1/`.

If this directory isn't present, rather than compiling the ASN.1 data types locally, CMake runs the script `Tools/ASN.1/FetcherScript.sh` to download precompiled types from Space Apps' GitLab server. This is so as to avoid the user to compile the data types themselves, as the ASN.1 compiler has a heavy dependency on Mono and isn't typically run more than once.

You may still compile the data types yourself if you wish, and you should do so if:

* The download fails (**warning:** currently happens if running CMake in a new branch *and* the types haven't been downloaded before; fixing in progress; a workaround is to run CMake once in the `master` branch before, or compile the types yourself).
* You modify or create a new data type in `Common/Types/ASN.1/`.

See the [ASN.1 documentation](/Tools/ASN.1/README.md) for more information. An overview of the data types, wrappers, sizes, etc. can be found in [this table](https://drive.google.com/open?id=0B2f4AImIv45fRUFkeldjdTVtSUE).

### Test and (optionally) install

Once the CDFF (Core and Support) is built you can run the unit tests with:

```shell
/path/to/CDFF/build$ make test
```

**Warning** don't even try to install for now, our current `make install` rule is very incomplete, sorry. Only building works, not installing. This is because we are focusing on developing the CDFF at the moment, not on installing (or uninstalling) it. In the future, one will be able to install the CDFF (Core and Support components) in their selected `CMAKE_INSTALL_PREFIX` (in `{bin,lib}` subdirectories) with:

```shell
/path/to/CDFF/build$ [sudo] make install
```

### Linked documentation

* [Dependencies of the CDFF](/External/README.md)
* [Using Docker and the InFuse Docker image](https://drive.google.com/open?id=1aW3_giavOZdvOljEEfun4W0Cq2tlnDvb8S3y2bysjpw)
