# ======================================================================
#  FILE:  $URL$
# ======================================================================
#  PROJECT:             :  aruco_marker_detection
#  VERSION              :  @VERION@
#  LANGUAGE             :  CMake script
#  AUTHOR               :  @AUTHOR@
#  COPYRIGHT            :  @COPYRIGHT@
#  COMPILER             :  cmake version 3.0.2
#  CREATED              :  @CREATED@
#  CLASS                :  -
#  LAST MODIFIED        :  @LASTMODIFIED@
#  GENERATED FROM MODEL :  -
#  ORIGINAL MODEL AUTHOR:  -
# ----------------------------------------------------------------------
#  @DESCRIPTION@
# ----------------------------------------------------------------------
#  HISTORY
#  $History$
# ======================================================================

cmake_minimum_required(VERSION 3.2)

project(aruco_marker_detection)

include($ENV{ESROCOS_CMAKE})

#required function call to set up esrocos
esrocos_init()

########### FOR INTEGRATION PROJECTS ##############

#uncomment if you want to add dependencies for the build process (only required in integration projects)
esrocos_add_dependency(PARTITION test_arcuo_marker_detector
                       MODULES   aruco opencv base_support base-types eigen3)

########### FOR COMPONENT PROJECTS ################

#uncomment if you want to export a function
esrocos_export_function("aruco_marker_detector" "share/taste_components/aruco_marker_detection")

#uncomment to export PKG-CONFIG-file
esrocos_export_pkgconfig( VERSION 1.0
                          DESCRIPTION "none"
                          REQUIRES  opencv base_support base-types eigen3
                          LIBS ""
                          STATIC_LIBS "" 
                          C_FLAGS ""
                        ) 
