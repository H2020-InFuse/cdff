/*
- This file contains the main page of the Doxygen documentation for this project.
- It will be shown as the Welcome Page in the documentation structure.
- For simplicity, all other doxygen pages should be placed inside a separate file within this directory, with the naming scheme: doxy-<pagename>.h .
- If your project has submodules, do not forget to add them as a subpage of the mainpage to preserve the project structure in the documentation.
 - Keep the conditional "if else" macro at the beginning of the next section in order to support recursive documentation if you are ever included as a submodule in another project.
 - To generate the documentation: run "cmake .." first, then run "make doc"
*/

/**
\if LIBORBSLAM_ISMAIN
\mainpage Welcome to the ORB-SLAM library 
\else
\page yarphelper_main Welcome to the ORB-SLAM library
\endif

See the main ORB-SLAM documentation. 

*/
