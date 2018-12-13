#!/usr/bin/env bash

# script in 4 steps.

#1. create build database
#2. run ccpcheck on current working directory
#3. generates a changed_file containing all files present in cppcheck_all.log
#4. dumps in file author, csv style format, who needs to work on what file.

#1.
DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
cd $DIR/../../
mkdir -p build_cppcheck
cd build_cppcheck
cmake with -D CMAKE_EXPORT_COMPILE_COMMANDS=ON -D CMAKE_INSTALL_PREFIX=../install/ ..

#2.
../Tools/CPPCheck/full-cppcheck.sh compile_commands.json

#3.
apt install -y xmlstarlet
xmlstarlet sel -t -v "/results/errors/error/location/@file" cppcheck/cppcheck.xml > changed_files

#4.
IFS=$'\r\n' GLOBIGNORE='*' command eval  'file=($(cat changed_files))'

declare -A users
users=( ["author Alessandro Bianco"]="@alessandro.bianco"
        ["author xma"]="@xma"
        ["author Thibaud Chupin"]="@tch"
        ["author Romain Michalec"]="@romain.michalec"
        ["author Vincent Bissonnette"]="@vincent.bissonnette"
        ["author Irene"]="@isn"
        ["author iya"]="@iya"
        ["author Bilal Wehbe"]="@Bilal.Wehbe"
        ["author Raul Dominguez"]="@raul.dominguez"
        ["author Raul.Dominguez"]="@raul.dominguez"
        ["author Steffen Planthaber"]="@steffen.planthaber"
        ["author Nassir Oumer"]="@nassir.oumer"
        ["author Unknown"]=""
        ["author Your Name"]=""
        ["author Xavier Martinez"]="@xma"
)

for i in "${file[@]}"
do
   echo "$i"
   line=$(git blame $i --porcelain | grep  "^author " | sort | uniq)
   for user in "${!users[@]}";
	do
	old=$user
	new=${users[$user]}";"
	line="${line/$old/$new}"
   done
   split=$(echo $line | tr ";" "\n")
   for author in $split
	do
    	echo "* [ ] $author: ; - $i" >> authors
   done
done


