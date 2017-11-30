## Synopsis

Welcome to the Infuse project repository. 

Enjoy it ! 

## Structure
See https://docs.google.com/document/d/1ppECSp_fz4f23C0t9v5XJkrxQOHpKud0CJxJu4E5LpI/edit?usp=sharing

## Coding Standard

See https://docs.google.com/document/d/1jQ8I3lRKLel6BT5Fac5twtjzZ0SiQrc9rK23v-3NOLM

## Externals 
Two options : 
1. You grab the dependencies for your platform (eg apt-get install opencv) 
and the build system will use those.

2. You fetch the git repositories present in /External, configure, make make install for each of them and the build system will use those. 

The External folder is composed by submodules, 
On first project pull, 

use "git submodule update --depth 1" or you might die from openCV overwhelmingly big repository. 

## Contributing

Excellent code depends on rigorous review. In Infuse, every change is reviewed using this flow:

A developer makes a change in their feature branch and tests it. When they're happy they push, and make a merge request.
The developer assigns the merge request to a reviewer, who looks at it and makes line and design level comments as appropriate. When the reviewer is finished, they assign it back to the author. 
The author addresses the comments. This stage can go around for a while, but once both are happy, one assigns to a final reviewer who can merge(a maintainer).
The final reviewer follows the same process again. The author again addresses any comments, either by changing the code or by responding with their own comments.
Once the final reviewer is happy and the build is green, they will merge.
