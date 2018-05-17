FROM ubuntu:16.04


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

# Add the scripts that download, build, and install the CDFF's dependencies
# (main script added here, component scripts added next)
COPY installers/infuse_environnement.env /tmp/CDFF/Tools/Docker/installers/
COPY fetch_compile_install_dependencies.sh /tmp/CDFF/Tools/Docker/

# Download, build, and install the CDFF's dependencies
RUN apt-get update && apt-get install -y libbz2-dev python-dev
COPY installers/boost.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s boost

COPY installers/eigen.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s eigen

COPY installers/flann.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s flann

COPY installers/qhull.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s qhull

COPY installers/yamlcpp.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s yaml-cpp

RUN apt-get update && apt-get install -y freeglut3-dev
COPY installers/opencv.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s opencv

COPY installers/vtk.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s vtk

COPY installers/pcl.sh /tmp/CDFF/Tools/Docker/installers/
RUN /bin/bash /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s pcl

LABEL version="1.7.1"
LABEL maintainer="xma@spaceapplications.com,romain.michalec@strath.ac.uk"
LABEL description="Image providing the CDFF's dependencies compiled from source and installed in /usr/local"
