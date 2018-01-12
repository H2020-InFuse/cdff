FROM ubuntu:latest

LABEL version="1.5"
LABEL maintainer "xma@spaceapplications.com"
LABEL description="This container compiles dependencies from CDFF\external\ \
It does not rely anymore on the ubuntu packages."

RUN apt-get update && apt-get install -y build-essential \
apt-utils \
git \
cppcheck \
mono-devel \
wget \
unzip \
checkinstall

ADD installers/infuse_environnement.env CDFF/External/installers/
ADD fetch_compile_install_dependencies.sh CDFF/External
RUN chmod +rwx CDFF/External/fetch_compile_install_dependencies.sh

COPY installers/cmake.sh CDFF/External/installers
RUN chmod +rwx CDFF/External/installers/cmake.sh
RUN CDFF/External/fetch_compile_install_dependencies.sh -s cmake

ADD installers/eigen.sh CDFF/External/installers
RUN chmod +rwx CDFF/External/installers/eigen.sh
RUN CDFF/External/fetch_compile_install_dependencies.sh -s eigen

ADD installers/flann.sh CDFF/External/installers
RUN chmod +rwx CDFF/External/installers/flann.sh
RUN CDFF/External/fetch_compile_install_dependencies.sh -s flann

ADD installers/qhull.sh CDFF/External/installers
RUN chmod +rwx CDFF/External/installers/qhull.sh
RUN CDFF/External/fetch_compile_install_dependencies.sh -s qhull

RUN apt-get install -y libbz2-dev python-dev

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s boost
#ENV BOOST_ROOT="/CDFF/External/install"



#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s tinyxml2

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s yamlcpp

#RUN apt-get install -y freeglut3-dev

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s opencv
#ENV OPENCV_ROOT="/CDFF/External/install"

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s vtk
#ENV VTK_ROOT="/CDFF/External/install"

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s pcl
