FROM ubuntu:latest

LABEL version="1.4"
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

# install all dependencies from repo
ADD ~/.ssh/id_rsa ~/.ssh/id_rsa \
&& eval "$(ssh-agent)" && ssh-agent -s \
&& chmod 0600 ~/.ssh/id_rsa \
&& ssh-add ~/.ssh/id_rsa 

# # Git protocol replacement
#RUN git config --global url."https://".insteadOf git://

#RUN git clone --depth 1 https://gitlab-ci-token:pVUF6xEhoz2kgWAUyyCr@gitlab.spaceapplications.com:InFuse/Infuse/CDFF.git

#RUN cd Infuse/CDFF/External \
#&& fetch_compile_install_dependencies.sh \
#&& ldconfig




