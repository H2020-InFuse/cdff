## Synopsis

Welcome to the Infuse project repository. 

Enjoy it ! 

## Structure
See https://docs.google.com/document/d/1ppECSp_fz4f23C0t9v5XJkrxQOHpKud0CJxJu4E5LpI/edit?usp=sharing

## Externals 
Two options : 
1. You grab the dependencies for your platform (eg apt-get install opencv) 
and the build system will use those.

2. You fetch the git repositories present in /External, configure, make make install for each of them and the build system will use those. 

The External folder is composed by submodules, 
On first project pull, 

use "git submodule update --depth 1" or you might die from openCV overwhelmingly big repository. 