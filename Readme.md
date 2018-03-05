## Synopsis

Welcome to the Infuse project repository. 

Enjoy it ! 

## Structure
See https://docs.google.com/document/d/1ppECSp_fz4f23C0t9v5XJkrxQOHpKud0CJxJu4E5LpI/edit?usp=sharing

## Coding Standard

See https://docs.google.com/document/d/1jQ8I3lRKLel6BT5Fac5twtjzZ0SiQrc9rK23v-3NOLM

## DFN Coding Guidelines

https://docs.google.com/document/d/1hFTRKgJNN3n_brT3aajMA03AR_jQ2eCo-ZM33ggY5cE/edit

## DFPC Coding Guidelines

https://docs.google.com/document/d/1ZUhZPnedd1mO42y-q4N7USltOnKeZzbyyZz_yzpLsmk/edit

## Externals (aka 3rd party dependencies)
See [Third party readme] (/External/Readme.md)

## Fetching sources and building 
    git clone https://gitlab.spaceapplications.com/InFuse/CDFF.git

### Get all dependencies first: 
    cd CDFF/External/
    ./fetch_compile_install_dependencies.sh
  
### Build CDFF with
    $ mkdir CDFF/build && cd CDFF/build
    $ cmake -DCMAKE_INSTALL_PREFIX=./ ..
    $ make install

### Now you can run the tests
    CDFF/build$ make test

### ASN.1 Datatypes
On first build, Cmake will run /Tools/ASNToC/FetcherScript.sh. 
This will download generated files from the server.  

If you want to develop new Types, you will need to compile them. See /Tools/ASNToC/Readme.md

## Contributing

Excellent code depends on rigorous review. In Infuse, every change is reviewed using this flow:

A developer makes a change in their feature branch and tests it. When they're happy they push, and make a merge request.
The developer assigns the merge request to a reviewer, who looks at it and makes line and design level comments as appropriate. When the reviewer is finished, they assign it back to the author. 
The author addresses the comments. This stage can go around for a while, but once both are happy, one assigns to a final reviewer who can merge(a maintainer).
The final reviewer follows the same process again. The author again addresses any comments, either by changing the code or by responding with their own comments.
Once the final reviewer is happy and the build is green, they will merge.
