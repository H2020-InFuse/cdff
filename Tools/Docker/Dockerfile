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

RUN git config --global http.sslVerify false
#RUN git clone --depth 1 https://gitlab-ci-token:pVUF6xEhoz2kgWAUyyCr@10.2.0.137/InFuse/CDFF.git -b feature/Repository_Structure  

RUN git clone --depth 1 https://gitlab-ci-token:pVUF6xEhoz2kgWAUyyCr@gitlab.spaceapplications.com/InFuse/CDFF.git -b feature/Repository_Structure 

ADD fetch_compile_install_dependencies.sh CDFF/External
RUN chmod +rwx CDFF/External/fetch_compile_install_dependencies.sh

#ADD build path
RUN echo "PATH=$PATH:/CDFF/External/install/bin" >> /root/.bashrc 
RUN echo "export PATH" >> /root/.bashrc 
ENV PATH="/CDFF/External/install/bin:${PATH}"

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s cmake

RUN apt-get install -y libbz2-dev python-dev 

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s boost

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s eigen

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s flann

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s qhull

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s tinyxml2

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s yamlcpp

RUN apt-get install -y freeglut3-dev

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s vtk

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s opencv

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh -s pcl
