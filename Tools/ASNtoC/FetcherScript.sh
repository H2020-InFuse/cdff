#!/bin/bash
#xma@spaceapplications.com
#This file fetches the latest generated files directly from the build server

#exit immediately if a simple command exits with a nonzero exit value.
set -e

function get_source_function(){
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
}

function show_error_exit {
  echo "Could not retreive correct Artifacts for you."
  echo "Please run GeneratorScript.sh instead, or switch to a branch having successfully been build on the server (eg master)."
  exit -1
}

function test_present_or_exit {
  if (! command -v $1); then
    echo "$1 Not Found."
    echo "Please Install it first before running this scrip."
    exit -1
  fi
}

function getbranch_function(){
  branch_name=$(git symbolic-ref -q HEAD)
  branch_name=${branch_name##refs/heads/}
  branch_name=${branch_name:-HEAD}

  if [[ $branch_name == *"unnamed branch"* ]]; then
  echo "You are in detached HEAD mode."
    show_error_exit
  fi
}

function download_artifact_function(){
  test_present_or_exit wget
  test_present_or_exit curl

  # Note: The wget command may fail when run in docker. This seems to be linked
  # to the MTU (Maximum Transmission Unit). Fixing it requires setting the --mtu
  # option in the docker startup script (/lib/systemd/docker.service).
  # However, if you are running from the docker image you're better off
  # just compiling the ASN.1 types locally with the GeneratorScript.
  if [[ `wget -S --spider $1  2>&1 | grep 'HTTP/1.1 200 OK'` ]]; then
  echo "Fetching latest Artifacts for branch $branch_name."
  curl -o generatedFiles.gz -LOk -X GET --header "PRIVATE-TOKEN: pVUF6xEhoz2kgWAUyyCr" https://gitlab.spaceapplications.com/InFuse/CDFF/-/jobs/artifacts/$branch_name/download?job=autogeneration
else
  show_error_exit
  fi
}

function unzip_function(){
  test_present_or_exit unzip
  echo "Unzipping to $OUTPUT_DIR"
  unzip -joq generatedFiles.gz -d $OUTPUT_DIR
  rm generatedFiles.gz
  echo "Done."
}

# Main
get_source_function
getbranch_function
download_artifact_function https://gitlab.spaceapplications.com/InFuse/CDFF/-/jobs/artifacts/$branch_name/download?job=autogeneration
unzip_function
