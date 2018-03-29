FROM ubuntu:latest

LABEL version="1.6.5"
LABEL maintainer "xma@spaceapplications.com"
LABEL description="This container compiles dependencies from CDFF\external\ \
It does not rely anymore on the ubuntu packages."

RUN apt-get update && apt-get install -y
  build-essential \
  apt-utils \
  cmake \
  git \
  cppcheck \
  mono-devel \
  wget \
  unzip \
  checkinstall

# add install directory
#RUN echo "PATH=$PATH:/CDFF/External/install/bin" >> /root/.bashrc
#RUN echo "export PATH" >> /root/.bashrc
ENV PATH="/CDFF/External/install/bin:${PATH}"

# master
ADD installers/infuse_environnement.env CDFF/External/installers/
ADD fetch_compile_install_dependencies.sh CDFF/External

# dependencies
RUN apt-get install -y libbz2-dev python-dev
COPY installers/boost.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s boost

COPY installers/eigen.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s eigen

COPY installers/flann.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s flann

COPY installers/qhull.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s qhull

COPY installers/tinyxml2.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s tinyxml2

COPY installers/yamlcpp.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s yaml-cpp

RUN apt-get install -y freeglut3-dev
COPY installers/opencv.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s opencv

COPY installers/vtk.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s vtk
#
COPY installers/pcl.sh CDFF/External/installers
RUN /bin/bash CDFF/External/fetch_compile_install_dependencies.sh -s pcl
