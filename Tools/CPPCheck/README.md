## Memo for cppcheck maintainers

Two checkers can be run
partial-cppcheck and full-cppcheck.

## full-cppcheck
Will run a full check, using all normaly compiled files with all possible flags ( eg debug=ON & debug=OFF ).
The resulst will be readable in HTML format under cppcheck/public.
full-cppcheck Requires as argument a compilation database (the list of files to be compiled, with the linked libraries and files.

## partial-cppcheck
Will run a partial check on the c++ files that differ between current branch and master.
The results will be log files under cppcheck/

## how to run

```
host# cd /pathTo/CDFF
host# mkdir -p build
host# cd build

 generates the file compile_commands.json used by cppcheck.
host# cmake with -D CMAKE_EXPORT_COMPILE_COMMANDS=ON -D CMAKE_INSTALL_PREFIX=/pathTo/CDFF/install/ ..
host# sudo docker run -it --volume=/pathTo/CDFF:/pathTo/CDFF nexus.spaceapplications.com/repository/infuse/docker-cppcheck:1.85

docker# cd /pathTo/CDFF

docker# Tools/CPPCheck/full-cppcheck.sh build/compile_commands.json

or

docker# Tools/CPPCheck/partial-cppcheck.sh

```


    echo "No compilation database provided.
    run  cmake with -D CMAKE_EXPORT_COMPILE_COMMANDS=ON to create a database file named compile_commands.json.
    Then : #analysis-cppcheck.sh compile_commands.json"

partial-cppcheck can and should be triggered by every user before push.

to test :

sudo docker run -it --volume=/home/xma/dev/CDFF:/home/xma/dev/CDFF nexus.spaceapplications.com/repository/infuse/docker-cppcheck:1.85
