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

ENV PATH="/CDFF/External/install/bin:${PATH}"
ENV PKG_CONFIG_PATH="/CDFF/External/install/lib/pkgconfig:${PKG_CONFIG_PATH}"
RUN echo "export PATH=$PATH" >> /root/.profile 
RUN echo "export PKG_CONFIG_PATH=$PKG_CONFIG_PATH" >> /root/.profile 

RUN mkdir -p CDFF/External
ADD fetch_compile_install_dependencies.sh CDFF/External
RUN chmod +rwx CDFF/External/fetch_compile_install_dependencies.sh

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s cmake

RUN apt-get install -y libbz2-dev python-dev 

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s boost
#ENV BOOST_ROOT="/CDFF/External/install"

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s eigen
#ENV EIGEN_ROOT="/CDFF/External/install"

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s flann
#ENV FLANN_ROOT="/CDFF/External/install"

#RUN cd CDFF/External \
#&& ./fetch_compile_install_dependencies.sh -s qhull
#ENV QHULL_ROOT="/CDFF/External/install"

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
