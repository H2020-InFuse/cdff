FROM ubuntu:16.04

LABEL version="1.6.5"
LABEL maintainer="xma@spaceapplications.com,romain.michalec@strath.ac.uk"
LABEL description="Image providing the CDFF's dependencies compiled from source and installed in /CDFF/External/install"

# Install the basic tools that are not already available in the base image
RUN apt-get update && apt-get install -y \
  build-essential \
  apt-utils \
  cmake \
  git \
  cppcheck \
  mono-devel \
  wget \
  curl \
  unzip \
  checkinstall

# The binaries that are going to be compiled must be found first on the PATH
ENV PATH="/CDFF/External/install/bin:${PATH}"

# Add the scripts that download, build, and install the CDFF's dependencies
# (main script added here, component scripts added next)
COPY installers/infuse_environnement.env CDFF/Tools/Docker/installers/
COPY fetch_compile_install_dependencies.sh CDFF/Tools/Docker/

# Download, build, and install the CDFF's dependencies
RUN apt-get update && apt-get install -y libbz2-dev python-dev
COPY installers/boost.sh CDFF/Tools/Docker/installers/
RUN /bin/bash CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -s boost

COPY installers/eigen.sh CDFF/Tools/Docker/installers/
RUN /bin/bash CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -s eigen

COPY installers/flann.sh CDFF/Tools/Docker/installers/
RUN /bin/bash CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -s flann

COPY installers/qhull.sh CDFF/Tools/Docker/installers/
RUN /bin/bash CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -s qhull

COPY installers/yamlcpp.sh CDFF/Tools/Docker/installers/
RUN /bin/bash CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -s yaml-cpp

RUN apt-get update && apt-get install -y freeglut3-dev
COPY installers/opencv.sh CDFF/Tools/Docker/installers/
RUN /bin/bash CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -s opencv

COPY installers/vtk.sh CDFF/Tools/Docker/installers/
RUN /bin/bash CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -s vtk

COPY installers/pcl.sh CDFF/Tools/Docker/installers/
RUN /bin/bash CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -s pcl
