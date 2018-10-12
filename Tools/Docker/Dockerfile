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

COPY fetch_compile_install_dependencies.sh /tmp/CDFF/Tools/Docker/
COPY installers/ /tmp/CDFF/Tools/Docker/installers/

RUN apt-get update && apt-get install --yes --quiet liblzma-dev libbz2-dev zlib1g-dev && \
    /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s boost
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s eigen
RUN apt-get update && apt-get install --yes --quiet libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev && \
    /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s ceres
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s flann
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s qhull
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s yaml-cpp
RUN apt-get update && apt-get install --yes --quiet \
      libavcodec-dev libavformat-dev libswscale-dev \
      libjpeg-dev libpng++-dev libdc1394-22-dev libjasper-dev libtiff5-dev \
      libv4l-dev \
      freeglut3-dev && \
    /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s opencv
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s vtk
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s pcl

COPY installers/edreswrapper.sh /tmp/CDFF/Tools/Docker/installers/
RUN /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s edres-wrapper

## Download, build, and install EnviRe and its dependencies

RUN apt-get update && apt-get install --yes --quiet pkg-config libtinyxml-dev libgoogle-glog-dev && \
    /tmp/CDFF/Tools/Docker/fetch_compile_install_dependencies.sh -i /usr/local -s base_cmake -s base_logging -s sisl -s base_types -s base_numeric -s base_boost_serialization -s console_bridge -s poco -s poco_vendor -s class_loader -s tools_plugin_manager -s envire_envire_core

## Image information

LABEL version="1.9"
LABEL maintainer="Xavier Martinez <xma@spaceapplications.com>, Romain Michalec <romain.michalec@strath.ac.uk>"
LABEL description="Image providing the dependencies of CDFF-core and CDFF-support, compiled from source, and installed in /usr/local"
