#===========================================================================================================================================
#
#                            CMAKE TEMPLATE
#
#===========================================================================================================================================
#
# Composant : tools
#
# Auteur : Fabrice SOUVANNAVONG
# Societe : Magellium
#
# Langage : CMake
# Type : script
#
# Descriptif : Ensemble d'outils macro CMAKE 
#
#-------------------------------------------------------------------------------------------------------------------------------------------
# HISTORIQUE
# VERSION : 17/02/2016 - FSG - Creation du fichier
# VERSION : 20/05/2016 - EBN - Mise en forme
# FIN-HISTORIQUE
#-------------------------------------------------------------------------------------------------------------------------------------------
IF( NOT TOOLS_CMAKE_LOADED )
SET(TOOLS_CMAKE_LOADED TRUE)

#===========================================================================================================================================
# INIT_MODULE
# Detecte s'il s'agit du fichier principal et initialise chaque itération de CMake.
# - suppression des fichiers *Config.cmake du répertoire de build. 
# INPUT  : aucune
# OUTPUT : définition de ${PROJECT_NAME}_ISMAIN (TRUE s'il s'agit du fichier principal) 
#===========================================================================================================================================
MACRO(INIT_MODULE)
IF( ${PROJECT_NAME} STREQUAL ${CMAKE_PROJECT_NAME})
    SET(${PROJECT_NAME}_ISMAIN TRUE)

    # Il faut supprimer les fichiers de configuration locale à chaque itération.
    MESSAGE("REMOVING local Config.cmake files in ${CMAKE_BINARY_DIR}")
    FILE(GLOB_RECURSE CONFIGFILES "${CMAKE_BINARY_DIR}/*Config.cmake")
    FOREACH(CONFIGFILE ${CONFIGFILES})
        MESSAGE("REMOVE ${CONFIGFILE}")
        FILE(REMOVE ${CONFIGFILE})
    ENDFOREACH()
    
    INCLUDE(modules.cmake)
ENDIF()
SET(VERSION 1.0.0)
ENDMACRO()

#===========================================================================================================================================
# ADD_SUBMODULES
# Ajout du répertoire modules.
#===========================================================================================================================================
MACRO(ADD_SUBMODULES)
IF( EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/modules )
    MESSAGE("Adding ${CMAKE_CURRENT_SOURCE_DIR}/modules")
    ADD_SUBDIRECTORY(modules)
ENDIF()
ENDMACRO()

#===========================================================================================================================================
# ADD_LIBRARY_AUTO
# Définie la bibliothèque par défaut (nom du projet en minuscule).
# INPUT:
#   PROJECT_NAME                : Déclarer avec la commande PROJECT().
#   SRCFILES                    : liste des sources.
#   HDRFILES                    : liste des en-tête (optionnel).
#   ${PROJECT_NAME}_SHARED_LIBS : STATIC ou SHARED (optionel).
#===========================================================================================================================================
MACRO(ADD_LIBRARY_AUTO)
IF( SRCFILES )
    STRING(TOLOWER ${PROJECT_NAME} LIBNAME)
    ADD_LIBRARY(${LIBNAME} ${${PROJECT_NAME}_SHARED_LIBS} ${SRCFILES} ${HDRFILES})
    TARGET_LINK_LIBRARIES(${LIBNAME} ${LIBRARIES})
    # Ajout de la librairie à la liste des bibliothèques à exporter
    LIST(APPEND LIBRARIES ${LIBNAME})    
ENDIF()
ENDMACRO()

#===========================================================================================================================================
# ADD_MODULE
# Ajout d'un répertoire traité comme un module.
#===========================================================================================================================================
MACRO(ADD_MODULE MODNAME MODDIR)
#FIND_PACKAGE(${MODNAME} QUIET)
IF( NOT ${MODNAME}_DONOTLOAD )
    MESSAGE("Adding module ${MODNAME} from ${CMAKE_CURRENT_SOURCE_DIR}/${MODDIR}")
    ADD_SUBDIRECTORY(${MODDIR})
ELSE()
    IF(NOT WIN32)
		string(ASCII 27 Esc)
		set(Red "${Esc}[31m")
		set(ColourReset "${Esc}[m")
	ENDIF()
	MESSAGE(WARNING "${Red}Module ${MODNAME} already exists or has been disabled. ${CMAKE_CURRENT_SOURCE_DIR}/${MODDIR} IS NOT BUILD.${ColourReset}")
ENDIF()
ENDMACRO()

#===========================================================================================================================================
# ADD_GITMODULE
# Ajout d'un répertoire traité comme un module.
# Le répertoire doit être un sous-module git, il est initialisé et mis à jour.
# NOT TESTED
#===========================================================================================================================================
MACRO(ADD_GITMODULE MODNAME MODDIR)
FIND_PACKAGE(${MODNAME} QUIET)
IF( NOT ${MODNAME}_DONOTLOAD )
   MESSAGE("Adding git module ${MODNAME} from ${CMAKE_CURRENT_SOURCE_DIR}/${MODDIR}")
   execute_process(
        COMMAND git submodule update --init ${CMAKE_CURRENT_SOURCE_DIR}/${MODDIR}
    )
    ADD_SUBDIRECTORY(${MODDIR})
ENDIF()
ENDMACRO()

#===========================================================================================================================================
# EXPORT THE CONFIGURATION FOR OTHERS
# https://cmake.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file
#===========================================================================================================================================

#===========================================================================================================================================
# EXPORT_MODULE
# Export le module pour qu'il soit accessible via FIND_PACKAGE. Cette macro
# requière le fichier ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
# INPUT:
#   PROJECT_NAME        : This is declared by PROJECT().
#   INCLUDE_DIRS        : liste des chemins à inclure par les utilisateurs.
#   LIBRARIES           : liste des bibliothèques à inclure par les utilisateurs.
#   USE_${PROJECT_NAME} : TRUE si le projet est utilisé.
#   VERSION             : project version
# INPUT NON RECOMMANDE  :
#   CXX_FLAGS           : flags de compilation spécifiques.
#   LD_FLAGS            : flags de lien spécifiques.
#===========================================================================================================================================
MACRO(EXPORT_MODULE)
include(CMakePackageConfigHelpers)

SET(PRJNAME ${PROJECT_NAME})
# Variables pour le fichier de configuration
SET(CONF_INCLUDE_DIRS ${INCLUDE_DIRS})
SET(CONF_LIBRARIES    ${LIBRARIES})
SET(CONF_CXX_FLAGS    ${CXX_FLAGS})
SET(CONF_LD_FLAGS     ${LD_FLAGS})
SET(CONF_FOUND        ${USE_${PROJECT_NAME}})

# export configuration file for others
EXPORT(PACKAGE ${PRJNAME})

# Create the XXXConfig.cmake for the build tree
IF( EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in")
	MESSAGE(STATUS "Using local Config.cmake.in")
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
ELSE()
	MESSAGE(STATUS "Using root Config.cmake.in")
    CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
ENDIF()

WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${VERSION}
  COMPATIBILITY SameMajorVersion )
ENDMACRO()

#===========================================================================================================================================
# EXPORT_MODULE
# Export le module pour qu'il soit accessible via FIND_PACKAGE. Cette macro
# requière le fichier ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
# INPUT:
#   PRJNAME             : Project name.
#   INCLUDE_DIRS        : liste des chemins à inclure par les utilisateurs.
#   LIBRARIES           : liste des bibliothèques à inclure par les utilisateurs.
# INPUT NON RECOMMANDE  : mettre ""
#   CXX_FLAGS           : flags de compilation spécifiques.
#   LD_FLAGS            : flags de lien spécifiques.
#===========================================================================================================================================
MACRO(EXPORT_EXTERN_MODULE PRJNAME INCLUDE_DIRS LIBRARIES CXX_FLAGS LD_FLAGS)
include(CMakePackageConfigHelpers)

SET(PRJNAME ${PRJNAME})
# Variables pour le fichier de configuration
SET(CONF_INCLUDE_DIRS ${INCLUDE_DIRS})
SET(CONF_LIBRARIES    ${LIBRARIES})
SET(CONF_CXX_FLAGS    ${CXX_FLAGS})
SET(CONF_LD_FLAGS     ${LD_FLAGS})
SET(CONF_FOUND        TRUE)

# export configuration file for others
EXPORT(PACKAGE ${PRJNAME})

# Create the XXXConfig.cmake for the build tree
CONFIGURE_FILE(../Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PRJNAME}Config.cmake" @ONLY)
ENDMACRO()

ENDIF()
