This is the code repository for the Core and Support components of the Common Data Fusion Framework (CDFF) developed by the InFuse consortium. [Another repository](https://gitlab.spaceapplications.com/InFuse/CDFF_dev), named `CDFF_dev`, contains the Dev component.

* The Core and Support components are those that must be deployed in your final robotic target system.

* The Dev component is a development environment which provides many utilities that can be used to efficiently implement reusable data fusion solutions. In fact, they **ought** to be used: using the CDFF's development environment is the natural way to develop the CDFF.

To develop a Data Fusion Node (DFN) or a Data Fusion Processing Compound (DFPC), you should therefore not just clone the `CDFF` repository but also the `CDFF_dev` repository. Put the resulting local repositories next to each other, for instance.

## Building the CDFF (Core and Support)

### Download and compile

Let's assume you've downloaded the CDFF's code (Core and Support components), for instance by cloning the `CDFF` repository somewhere on your computer (if you leave the path out in the following command line, a `CDFF` directory will be created in your current directory):

```
$ git clone git@gitlab.spaceapplications.com:InFuse/CDFF.git [/path/to/where/I/want/the/CDFF]
```

There are two ways you can build the CDFF (Core and Support components):

* You can compile it using CMake and a C++ compiler **in a Docker container**.

    We provide a Docker image based on Ubuntu 16.04 where build tools and the CDFF's dependencies are already installed (**warning:** except those we forgot: please let us know if you spot one). You can span a container from that image, mount the CDFF's code inside it, and compile. That image, and the libraries it contains, make up a common **reference environment** for the CDFF.

    We have documentation about [using Docker and the InFuse Docker image](https://drive.google.com/open?id=1aW3_giavOZdvOljEEfun4W0Cq2tlnDvb8S3y2bysjpw), please see also [this short section](/External/Readme.md#option-1-get-the-cdffs-dependencies-packaged-in-a-docker-image) in the documentation about the dependencies of the CDFF.

    Once you have mounted the directory containing the CDFF's source code inside your Docker container, you can build the CDFF (Core and Support):

    ```
    /path/to/CDFF/build$ cmake [-D CMAKE_INSTALL_PREFIX=/path/to/CDFF/install/] /path/to/CDFF/
    /path/to/CDFF/build$ make
    ```

* You can compile it using CMake and a C++ compiler **in a directory on your computer**.

    In that case, you will have installed the **dependencies** of the CDFF yourself on your computer, and the dependencies of these dependencies, and the necessary build tools. To do so, you can use a combination of your distribution's software package manager and/or compiling from source. The dependencies you compile from source do not need to be installed in your system's local hierarchy (`/usr/local/`): you may want to put them there, or you may prefer to put them in a dedicated directory next to the CDFF's source (`External/install/`), which is the **recommended** location for now as it makes uninstalling them easy.

    The documentation about the [dependencies of the CDFF](/External/Readme.md) describes this topic in detail.

    Once you have installed all the dependencies and build tools, you can build the CDFF (Core and Support):

    ```
    /path/to/CDFF/build$ cmake [-D USE_BUNDLED_DEPENDENCIES=ON] [-D CMAKE_INSTALL_PREFIX=/path/to/CDFF/install/] /path/to/CDFF/
    /path/to/CDFF/build$ make
    ```

Paths and options:

* `/path/to/CDFF/`: Directory where the CDFF's top-level `CMakeLists.txt` file is located.

* `/path/to/CDFF/build/`: Directory where you would like to build the CDFF.
  - For instance: a subdirectory named `build` in `/path/to/CDFF/` results in the commonplace `cmake ../` invocation.
  - Default: this is always the current directory.

* `CMAKE_INSTALL_PREFIX`: Directory where you would like to optionally install the CDFF (in `{bin,lib}` subdirectories).
  - For instance: `$HOME/.local/` is specified by [systemd's file-hierarchy specification](https://www.freedesktop.org/software/systemd/man/file-hierarchy.html), an extension of the [XDG Base Directory specification](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html), so `$HOME/.local/bin` is normally on the `PATH` of all systemd-compatible distributions.
  - Default: `/usr/local/`, and `/usr/local/bin` is on the `PATH` in all known distributions.
  - **Not recommended** to use the default for now, because I'm not sure we have a `make uninstall` rule.
  - **Recommended** (if you install at all) to install somewhere that you can easily `rm -r`, for instance a ad-hoc `/path/to/CDFF/install/`.

* `USE_BUNDLED_DEPENDENCIES`: Whether or not to look for the CDFF's direct dependencies (Boost, Eigen, PCL, OpenCV...) in `External/install/{bin,include,lib,share}` before looking for them in `/usr/local/{bin,include,lib,share}`.
  - Default: `OFF`, look directly in `/usr/local/`.
  - **Recommended** to use default when building inside an InFuse Docker container.
  - **Recommended** to install those dependencies in `External/install/` if you're not using Docker, and therefore recommended to use `ON` for that option.

CMake variables given using the `-D` option are written to the CMake cache (`/path/to/CDFF/build/CMakeCache.txt`) and therefore don't need to be given on subsequent CMake runs. CMake will read their value in the cache. If you want to change a CMake cache entry, it is safer to delete the cache in addition to giving the new value on the command line. This makes sure that all entries whose value depends on the changed entry are regenerated, instead of being read from the cache.

### ASN.1 data types

CMake checks whether there is a directory called `Common/Types/C/`, meant to contain `.h` and `.c` files generated ("compiled") from the ASN.1 data types present in `Common/Types/ASN.1/`.

If this directory isn't present, rather than compiling the ASN.1 data types locally, CMake runs the script `Tools/ASNToC/FetcherScript.sh` to download precompiled types from Space Apps' GitLab server. This is so as to avoid the user to compile the data types themselves, as the ASN.1 compiler has a heavy dependency on Mono and isn't typically run more than once.

You may still compile the data types yourself if you wish, and you should do so if:

* The download fails (**warning:** currently happens if running CMake in a new branch *and* the types haven't been downloaded before; fixing in progress; a workaround is to run CMake once in the `master` branch before, or compile the types yourself).
* You modify or create a new data type in `Common/Types/ASN.1/`.

See the [ASNtoC documentation](/Tools/ASNtoC/Readme.md) for more information.

### Test and (optionally) install

Once the CDFF is built you can run the unit tests and/or install the CDFF in your selected `CMAKE_INSTALL_PREFIX` with, respectively:

```
/path/to/CDFF/build$ make test
/path/to/CDFF/build$ [sudo] make install
```

### Linked documentation

* [Dependencies of the CDFF](/External/Readme.md)
* [Using Docker and the InFuse Docker image](https://drive.google.com/open?id=1aW3_giavOZdvOljEEfun4W0Cq2tlnDvb8S3y2bysjpw)

## Contributing to the CDFF

### Workflow

We use the popular Git **feature branch workflow** with **merge requests**.

In short, developpers develop new features and bug fixes in **feature branches** named anything but `master` in their **local repositories** cloned from the one on the `origin` **remote**, which is Space Apps' server. When they feel that a branch is ready to be shared with the other developers, they can **push** it to the `origin` remote (they mustn't push their branch before it is at least somewhat ready to be shared). When they feel that the branch is ready to be integrated into the main development history, they must request that it is merged into the `master` branch of the `origin` remote.

The **merge request** is easiest initiated through GitLab's web interface. The developer shall write a detailed description of the changes introduced by their branch, and assign the merge request to the most adequate reviewer. Everyone though, not just the reviewer, is encouraged to review and comment on the merge request.

* Once enough positive feedback is obtained, and all objections have been addressed, the branch can be **merged** into `master` on `origin`. The original developer must of course never merge their branch themselves (accept their own merge request) before they have left enough time for their request to be reviewed and gather enough positive feedback.
* If the merge request is rejected, or if the original developer cancels it, the request must be **closed** without the branch being merged. This is the case, for instance, of an idea that needs substantial further work.

In the case of an accepted merge request, the original developper, and everyone else, can finally **pull** (fetch and merge) the updated `master` into their own `master` branch in their repositories. Last but not least, the original developper must also **delete the merged branch** not just from their own local repository but also from the `origin` remote, and more generally from every remote where they have pushed it.

In the case of a rejected or canceled merge request, the original developper must also **delete the unmerged branch** from the `origin` remote, however they might want to keep it in their local repository if they intend to rework it. Note that one must also **delete their abandonned branches** (abandonned ideas) from the `origin` remote if they pushed them there.

Git will never delete a branch by itself, hence why we have to do it ourselves, otherwise they'll linger around. In particular, Git never automatically deletes a branch after merging it into another, contrary to what the term "merge" can imply.

### Documentation about contributing to the CDFF

* [Dependencies of the CDFF (extended version)](https://drive.google.com/open?id=1Lv1ryzOCpTKXPyYZ77M07PNrthuIvzkSY2At1ib6FX8)
* [Description of the repository filesystem hierarchy](https://drive.google.com/open?id=1ppECSp_fz4f23C0t9v5XJkrxQOHpKud0CJxJu4E5LpI)
* [Help with Git and its jargon](https://drive.google.com/open?id=1b9SNJDLAeYy8wc-1ryeyGpYzZKl0vcmHSz0IqaAfmLI)
* [Coding standard for the CDFF](https://drive.google.com/open?id=1jQ8I3lRKLel6BT5Fac5twtjzZ0SiQrc9rK23v-3NOLM)
* [Tutorial on writing DFNs](https://drive.google.com/open?id=1hFTRKgJNN3n_brT3aajMA03AR_jQ2eCo-ZM33ggY5cE)
* [Tutorial on writing DFPCs](https://drive.google.com/open?id=1ZUhZPnedd1mO42y-q4N7USltOnKeZzbyyZz_yzpLsmk)
