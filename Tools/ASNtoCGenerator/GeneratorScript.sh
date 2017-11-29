#!/bin/bash
#This file calls the ASN to C compiler binary
#It needs to be modified whenever a new ASN type is added

#exit immediately if a simple command exits with a nonzero exit value.
set -e

# Clean the output directory #TBD
rm -rf ../../Common/Types/C/*
mkdir -p ../../Common/Types/C

# Calling the compiler, add new .asn files here
./bin/asn1c ../../Common/Types/ASN.1/*.asn 

# copy stuff
cp -r *.h *.c ../../Common/Types/C/
rm -rf *.h *.c

#Removing this generated file we do not want to use
rm -rf Makefile.am.sample
