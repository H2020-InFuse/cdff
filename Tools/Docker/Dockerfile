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

COPY get-cdff-dependencies.sh /tmp/CDFF/Tools/Docker/
COPY installers/ /tmp/CDFF/Tools/Docker/installers/
COPY patches/ /tmp/CDFF/Tools/Docker/patches/

RUN apt-get update && apt-get install --yes --quiet liblzma-dev libbz2-dev zlib1g-dev && \
    /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d boost
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d yaml-cpp
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d eigen
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d cloudcompare-core
RUN apt-get update && apt-get install --yes --quiet libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev && \
    /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d ceres
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d nabo
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d pointmatcher
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d flann
RUN apt-get update && apt-get install --yes --quiet \
      libavcodec-dev libavformat-dev libswscale-dev \
      libjpeg-dev libpng++-dev libdc1394-22-dev libjasper-dev libtiff5-dev \
      libv4l-dev \
      freeglut3-dev && \
    /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d opencv
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d vtk
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d pcl
RUN /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d edres-wrapper

## Download, build, and install EnviRe and its dependencies

RUN apt-get update && apt-get install --yes --quiet pkg-config libtinyxml-dev libgoogle-glog-dev && \
    /tmp/CDFF/Tools/Docker/get-cdff-dependencies.sh -i /usr/local -d none -e

## Image information

LABEL version="2.0.3"
LABEL maintainer="Xavier Martinez <xma@spaceapplications.com>, Romain Michalec <romain.michalec@strath.ac.uk>"
LABEL description="Image providing the dependencies of CDFF-core and CDFF-support, compiled from source, and installed in /usr/local"
