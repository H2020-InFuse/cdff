#!/bin/bash
#xma@spaceapplications.com
#This file calls the ASN to C compiler binary
#It needs to be called whenever a new ASN type is added to /Common/Types/Asn

#exit immediately if a simple command exits with a nonzero exit value.
set -e

#Get working directory and script containing directory
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

#output directory where the files will be uncompressed
BASE_DIR=$DIR"/../../Common/Types"

if [ $# -eq 0 ]
  then
    echo "No arguments supplied, base output directory will be :$BASE_DIR"
  else
    BASE_DIR=$1
fi

#set ASN and C diretories
ASN_DIR=$BASE_DIR/ASN.1
C_DIR=$BASE_DIR/C

ASN1SCC=asn1scc
# Clean the output directory #TBD
echo "Cleaning old directory:$C_DIR"
rm -rf $C_DIR/*
mkdir -p $C_DIR
echo "Done."

# binary can be compiled from https://github.com/ttsiodras/asn1scc
# or downloaded from https://download.tuxfamily.org/taste/ASN1SCC/
if [ ! -d "$ASN1SCC" ]; then
	echo "Downloading compiler"
	wget https://download.tuxfamily.org/taste/ASN1SCC/ASN1SCC-latest.tgz
	tar -xf ASN1SCC-latest.tgz
	rm ASN1SCC-latest.tgz
	echo "Done."
else
	echo "Reusing compiler in folder $ASN1SCC"
fi


#Compile ASN files To C
# has to be a oneliner, else the compiler misses definitions
echo "Compiling $ASN_DIR to $C_DIR"
asnFiles=`find $ASN_DIR -name '*.asn'`
#echo $asnFiles
mono ./asn1scc/asn1.exe $asnFiles -c -uPER -o $C_DIR
echo "Done."

