#===========================================================================================================================================
#
#                            CMAKE TEMPLATE
#
#===========================================================================================================================================
#
# Component : CMakeLists.txt of modules
#
# Author : Vincent Bissonnette
# Company : Magellium
#
# Description : Main CMakeLists.txt for submodules of a project. 
#
ADD_MODULE(DBoW2 DBoW2)
EXPORT_EXTERN_MODULE(DBoW2 "${CMAKE_CURRENT_SOURCE_DIR}/DBoW2" "DBoW2" "" "")

ADD_MODULE(g2o g2o)
EXPORT_EXTERN_MODULE(g2o "${CMAKE_CURRENT_SOURCE_DIR}/g2o" "g2o" "" "")

