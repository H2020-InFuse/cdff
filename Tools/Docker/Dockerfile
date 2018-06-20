FROM ubuntu:16.04

# Install the basic tools that are not already available in the base image
RUN apt-get update && apt-get install -y \
  build-essential \
  apt-utils \
  cmake \
  git \
  cppcheck \
  mono-runtime libmono-system-core4.0-cil libmono-system-numerics4.0-cil \
  wget \
  curl \
  unzip

# Add the scripts that download, build, and install the CDFF's dependencies
# (main script added here, component scripts added next)
COPY installers/infuse_environnement.env /tmp/CDFF/Tools/Docker/installers/
COPY fetch_compile_install_dependencies.sh /tmp/CDFF/Tools/Docker/

# Download, build, and install the CDFF's dependencies (and their dependencies)
RUN apt-get update && apt-get install -y liblzma-dev libbz2-dev zlib1g-dev
COPY installers/boost.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s boost

COPY installers/eigen.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s eigen

RUN apt-get update && apt-get install -y libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
COPY installers/ceres.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s ceres

COPY installers/flann.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s flann

COPY installers/qhull.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s qhull

COPY installers/yamlcpp.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s yaml-cpp

RUN apt-get update && apt-get install -y \
  libavcodec-dev libavformat-dev libswscale-dev \
  libjpeg-dev libpng++-dev libdc1394-22-dev libjasper-dev libtiff5-dev \
  libv4l-dev \
  freeglut3-dev
COPY installers/opencv.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s opencv

COPY installers/vtk.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s vtk

COPY installers/pcl.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s pcl

LABEL version="1.8"
LABEL maintainer="Xavier Martinez <xma@spaceapplications.com>, Romain Michalec <romain.michalec@strath.ac.uk>"
LABEL description="Image providing the dependencies of CDFF-core and CDFF-support, compiled from source, and installed in /usr/local"
