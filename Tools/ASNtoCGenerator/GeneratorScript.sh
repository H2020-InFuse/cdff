#!/bin/bash
#This file calls the ASN to C compiler binary
#It needs to be modified whenever a new ASN type is added

#exit immediately if a simple command exits with a nonzero exit value.
set -e
dir="../../Common/Types"
ASN1SCC=asn1scc

# Clean the output directory #TBD
rm -rf $dir/C/*
mkdir -p $dir/C

# binary can be compiled from https://github.com/ttsiodras/asn1scc
# or downloaded from https://download.tuxfamily.org/taste/ASN1SCC/
if [ ! -d "$ASN1SCC" ]; then
	wget https://download.tuxfamily.org/taste/ASN1SCC/ASN1SCC-latest.tgz
	tar -xvf ASN1SCC-latest.tgz
	rm ASN1SCC-latest.tgz
fi

#Compile ASN files To C
# has to be a oneliner, else the compiler misses definitions
asnFiles=`find $dir/ASN.1 -name '*.asn'`
#echo $asnFiles
mono ./asn1scc/asn1.exe $asnFiles -c -o $dir/C/