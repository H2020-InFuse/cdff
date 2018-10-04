FROM ubuntu:16.04

## Install basic tools that are not already available in the base image

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --yes --quiet apt-utils && \
    apt-get install --yes --quiet \
      build-essential \
      cmake \
      curl \
      git \
      mono-runtime libmono-system-core4.0-cil libmono-system-numerics4.0-cil \
      unzip \
      wget

## Download, build, and install the CDFF's dependencies (and their dependencies)

SHELL ["/bin/bash", "-c"]

COPY installers/infuse_set_pkg_config_path.sh /tmp/CDFF/Tools/Docker/installers/
COPY fetch_compile_install_dependencies.sh /tmp/CDFF/Tools/Docker/

RUN apt-get update && apt-get install --yes --quiet liblzma-dev libbz2-dev zlib1g-dev
COPY installers/boost.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s boost

COPY installers/eigen.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s eigen

RUN apt-get update && apt-get install --yes --quiet libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
COPY installers/ceres.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s ceres

COPY installers/flann.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s flann

COPY installers/qhull.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s qhull

COPY installers/yamlcpp.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s yaml-cpp

RUN apt-get update && apt-get install --yes --quiet \
      libavcodec-dev libavformat-dev libswscale-dev \
      libjpeg-dev libpng++-dev libdc1394-22-dev libjasper-dev libtiff5-dev \
      libv4l-dev \
      freeglut3-dev
COPY installers/opencv.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s opencv

COPY installers/vtk.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s vtk

COPY installers/pcl.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s pcl

## For EnviRe

RUN apt-get update && apt-get install --yes --quiet pkg-config
COPY installers/base_cmake.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s base_cmake

COPY installers/base_logging.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s base_logging

COPY installers/sisl.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s sisl

COPY installers/base_types.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s base_types

COPY installers/base_numeric.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s base_numeric

COPY installers/base_boost_serialization.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s base_boost_serialization

COPY installers/console_bridge.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s console_bridge

COPY installers/poco.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s poco

COPY installers/poco_vendor.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s poco_vendor

COPY installers/class_loader.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s class_loader

RUN apt-get update && apt-get install --yes --quiet libtinyxml-dev libgoogle-glog-dev
COPY installers/tools_plugin_manager.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s tools_plugin_manager

COPY installers/envire_envire_core.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s envire_envire_core

## Image information

LABEL version="1.9"
LABEL maintainer="Xavier Martinez <xma@spaceapplications.com>, Romain Michalec <romain.michalec@strath.ac.uk>"
LABEL description="Image providing the dependencies of CDFF-core and CDFF-support, compiled from source, and installed in /usr/local"
