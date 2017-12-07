#!/bin/bash
#xma@spaceapplications.com
#This file fetches the latest generated files directly from the build server

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
OUTPUT_DIR=$DIR"/../../Common/Types/C"

if [ $# -eq 0 ]
  then
    echo "No arguments supplied, output directory will be :$OUTPUT_DIR"
  else
    OUTPUT_DIR=$1
fi

#get latest files
echo "Fetching latest Artifacts containing C generated files from ASN description."  
curl -o generatedFiles.gz -LOk -X GET --header "PRIVATE-TOKEN: X-2vkSkeB7zGCCT7pz8V" https://gitlab.spaceapplications.com/InFuse/CDFF/-/jobs/artifacts/master/download?job=autogeneration
echo "Done." 

echo "Unzipping to $OUTPUT_DIR" 
unzip -joq generatedFiles.gz -d $OUTPUT_DIR
rm generatedFiles.gz 
echo "Done." 
