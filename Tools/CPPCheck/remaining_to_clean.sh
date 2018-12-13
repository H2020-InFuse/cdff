#!/bin/bash
#1. run cpp check inside a docker mounted on current directory

current=$(pwd)
echo You need to be at the ROOT of CDFF, you currently are at : $current
sudo docker run -v $current:$current nexus.spaceapplications.com/repository/infuse/docker-cppcheck:1.85 /bin/bash -c $current"/Tools/CPPCheck/runcppcheck.sh"
