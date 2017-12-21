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
unzip 

RUN apt-get install -y net-tools \
iputils-ping

# add ssh key for git 
#ADD /.ssh/id_rsa /root/.ssh/id_rsa
#ADD do-ssh.sh CDFF/do-ssh.sh
#RUN /CDFF/do-ssh.sh

#RUN cp /etc/resolv.conf /etc/resolv.conf.bak 
#ll /roo		
#CMD cp /etc/resolv.conf.bak /etc/resolv.conf

RUN git config --global http.sslVerify false
RUN git clone --depth 1 https://gitlab-ci-token:pVUF6xEhoz2kgWAUyyCr@10.2.0.137/InFuse/CDFF.git -b feature/Repository_Structure  

#RUN git clone --depth 1 https://gitlab-ci-token:pVUF6xEhoz2kgWAUyyCr@gitlab.spaceapplications.com/InFuse/CDFF.git -b feature/Repository_Structure  

RUN cd CDFF/ \ 
&& git submodule update --init --recursive #--depth 1

ADD fetch_compile_install_dependencies.sh CDFF/External
RUN chmod +rwx CDFF/External/fetch_compile_install_dependencies.sh

RUN apt-get install -y checkinstall

#ADD new path
RUN echo "PATH=$PATH:/CDFF/External/install/bin" >> ~/.bashrc \
&& echo "export PATH" >> ~/.bashrc \
&& source ~/.bashrc

RUN cd CDFF/External \
&& ./fetch_compile_install_dependencies.sh \
&& ldconfig





