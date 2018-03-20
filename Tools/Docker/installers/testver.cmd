#!/bin/bash

# NAME: testver
# PATH: /usr/local/bin
# DESC: Test a program's version number >= to passed version number
# DATE: May 21, 2017.

# CALL: testver Program Version

# PARM: 1. Program - validated to be a command
#       2. Version - validated to be numeric

# NOTE: Extracting version number perl one-liner found here:
#       http://stackoverflow.com/questions/16817646/extract-version-number-from-a-string

#       Comparing two version numbers written by Dell employee for DKMS application:
#       http://lists.us.dell.com/pipermail/dkms-devel/2004-July/000142.html

# Map parameters to coder-friendly names.
Program="$1"
Version="$2"

# Program name must be a valid command.
command -v $Program >/dev/null 2>&1 || { echo "Command: $Program not found. Check spelling."; exit 99; }

# Passed version number must be valid format.
if ! [[ $Version =~ ^([0-9]+\.?)+$ ]]; then
  echo "Version number: $Version has invalid format. Aborting.";
  exit 99
fi

# Get current version number of installed program
InstalledVersion=$( "$Program" --version | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/' )

# Sanity check
if ! [[ $InstalledVersion =~ ^([0-9]+\.?)+$ ]]; then
  echo "Invalid version number: $InstalledVersion found for command: $Program"
  exit 99
fi

version_checker() {
  local ver1=$InstalledVersion
  while [ `echo $ver1 | egrep -c [^0123456789.]` -gt 0 ]; do
    char=`echo $ver1 | sed 's/.*\([^0123456789.]\).*/\1/'`
    char_dec=`echo -n "$char" | od -b | head -1 | awk {'print $2'}`
    ver1=`echo $ver1 | sed "s/$char/.$char_dec/g"`
  done
  local ver2=$Version
  while [ `echo $ver2 | egrep -c [^0123456789.]` -gt 0 ]; do
    char=`echo $ver2 | sed 's/.*\([^0123456789.]\).*/\1/'`
    char_dec=`echo -n "$char" | od -b | head -1 | awk {'print $2'}`
    ver2=`echo $ver2 | sed "s/$char/.$char_dec/g"`
  done

  ver1=`echo $ver1 | sed 's/\.\./.0/g'`
  ver2=`echo $ver2 | sed 's/\.\./.0/g'`

  do_version_check "$ver1" "$ver2"
}

do_version_check() {

  [ "$1" == "$2" ] && return 10

  ver1front=`echo $1 | cut -d "." -f -1`
  ver1back=`echo $1 | cut -d "." -f 2-`
  ver2front=`echo $2 | cut -d "." -f -1`
  ver2back=`echo $2 | cut -d "." -f 2-`

  if [ "$ver1front" != "$1" ] || [ "$ver2front" != "$2" ]; then
    [ "$ver1front" -gt "$ver2front" ] && return 11
    [ "$ver1front" -lt "$ver2front" ] && return 9

    [ "$ver1front" == "$1" ] || [ -z "$ver1back" ] && ver1back=0
    [ "$ver2front" == "$2" ] || [ -z "$ver2back" ] && ver2back=0
    do_version_check "$ver1back" "$ver2back"
    return $?
  else
    [ "$1" -gt "$2" ] && return 11 || return 9
  fi
}

version_checker "$InstalledVersion" "$Version"
TestResults=$?
#echo "11= greater, 10= same and 9= less: $TestResults"

[[ $TestResults -eq 9 ]] && exit 1 ;
exit 0
