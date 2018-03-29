# CDFF

Welcome to the code repository for the Common Data Fusion Framework developed by the InFuse consortium (Core and Support components).

## The repositories

The `CDFF` repository contains the Core and Support components of the CDFF, whereas the `CDFF_dev` repository contains its Dev component.

  The Core and Support components are those that must be deployed on your final robotic target system.

  The [Dev component](https://gitlab.spaceapplications.com/InFuse/CDFF_dev) is a development environment for the CDFF, which provides many utilities that can be used to efficiently implement reusable data fusion solutions. In fact, they *ought* to be used: using the CDFF's development environment is the natural way to develop the CDFF.

So to develop a Data Fusion Node (DFN) or a Data Fusion Processing Compound (DFPC), you should not just clone the `CDFF` repository but also the `CDFF_dev` repository. Put your resulting local repositories next to each other, for instance.

## Documentation

Documentation about building the CDFF:

* [Dependencies of the CDFF](/External/Readme.md)
* [Building the CDFF inside a Docker container (and more generally, using Docker)](https://drive.google.com/open?id=1aW3_giavOZdvOljEEfun4W0Cq2tlnDvb8S3y2bysjpw)
* This file

## How to build the CDFF

First clone the `CDFF` repository and make sure that you have all the CDFF's direct and recurse dependencies. The documentation about the [dependencies of the CDFF](/External/Readme.md) describes this topic in detail and you **really** should read it. In short, if you aren't using a Docker image where everything is already installed (but **warning** see caveats and WIPs in linked documentation), you should first install all of the CDFF's recurse dependencies using your system's package manager, and then install all of the CDFF's direct dependencies from source into `External/install` (default):

```
$ git clone git@gitlab.spaceapplications.com:InFuse/CDFF.git
$ cd CDFF/External
$ ./fetch_compile_install_dependencies.sh  # warning: tested on bash/Ubuntu only
```

Once this is done (VTK and the PCL take forever to compile), you can build the CDFF with e.g.:

```
/path/to/CDFF/build$ cmake -D USE_BUNDLED_DEPENDENCIES=ON -D CMAKE_INSTALL_PREFIX=./ ..
/path/to/CDFF/build$ make
```

Once the CDFF is built you can install it and/or run the tests with, respectively:

```
/path/to/CDFF/build$ make install
/path/to/CDFF/build$ make tests
```

## ASN.1 datatypes

On the first run of the first build process, CMake will run `/Tools/ASNToC/FetcherScript.sh` which will download ASN.1 datatype generated files from Space Apps's servers and put them into your local repository. This won't happen again unless you clone the `CDFF` repository anew of course. If you want to develop new datatypes, you will need to compile them: see the [ASNtoC documentation](/Tools/ASNToC/Readme.md).

# Contributing to the CDFF

We use the popular Git **feature branch workflow** with **merge requests**.

In short, developpers develop bug fixes and new features in *feature branches* named anything but `master` in their *local repositories* cloned from the one on the `origin` *remote*, which is Space Apps's server. When they feel that a branch is ready to be publicized (made available) to the other developers, they may *push* it to the `origin` remote (they mustn't push it before it is at least somewhat ready to be shared). When they feel that the branch is ready to be integrated into the main development history, they must request that it is merged into the `master` branch of the `origin` remote.

The *merge request* is easiest initiated through GitLab's web interface. The developer shall write a detailed description of the changes introduced by their branch, and assign the merge request to the most adequate reviewer. Everyone though, not just the reviewer, is encouraged to review and comment on the merge request. Once enough positive feedback is obtained, and all objections have been addressed, the branch can be *merged* into `master` on `origin`. However, if the merge request is rejected, or if the original developer cancels it, the request must be *closed* without the branch being merged (this is the case, for instance, of an idea that needs substantial further work). Note that the original developer must of course **never** merge their branch themselves (accept their own merge request) until they have left enough time for the request to be reviewed and gather enough positive feedback!

In the case of an accepted merge request, the original developper, and everyone else, can finally *pull* (fetch and merge) the updated `master` into their own `master` branch in their repositories. Last but not least, the original developper must also *delete the merged branch* not just from their own local repository but also from the `origin` remote, and more generally from every other remote where they may have pushed it.

In the case of a rejected or canceled merge request, the original developper must also *delete the unmerged branch* from the `origin` remote, however they might want to keep it in their local repository if they intend to rework it. Note that one must also *delete their abandonned branches* (abandonned ideas) from the `origin` remote if they had pushed them here.

Git will never delete a branch by itself, hence why we have to do it ourselves, otherwise they'll linger around. In particular, branches are not deleted after they are merged into another.

## Documentation

Documentation about contributing to the CDFF:

* [Dependencies of the CDFF (extended version)](https://drive.google.com/open?id=1Lv1ryzOCpTKXPyYZ77M07PNrthuIvzkSY2At1ib6FX8)
* [Description of this Git repository](https://drive.google.com/open?id=1ppECSp_fz4f23C0t9v5XJkrxQOHpKud0CJxJu4E5LpI)
* [Help and guidelines for not misusing Git](https://drive.google.com/open?id=1b9SNJDLAeYy8wc-1ryeyGpYzZKl0vcmHSz0IqaAfmLI) and the Git workflow described above
* [Coding standard](https://drive.google.com/open?id=1jQ8I3lRKLel6BT5Fac5twtjzZ0SiQrc9rK23v-3NOLM)
* [Tutorial on writing DFNs](https://drive.google.com/open?id=1hFTRKgJNN3n_brT3aajMA03AR_jQ2eCo-ZM33ggY5cE)
* [Tutorial on writing DFPCs](https://drive.google.com/open?id=1ZUhZPnedd1mO42y-q4N7USltOnKeZzbyyZz_yzpLsmk)
