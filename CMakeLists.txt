#required the minimum version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 3.5.1 )

#project name
PROJECT( slam )

#c++11 or higher version
ADD_DEFINITIONS( -std=c++11 )

#option of turning on the log or not
OPTION( ONLOG "log on or off" OFF )

IF( ONLOG )
	ADD_DEFINITIONS( -DTERMINAL_LOG )
ENDIF( ONLOG )

#set the output directory of the libs
SET( LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib" )

#find required eigen
FIND_PACKAGE( Eigen3 )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

#find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

#directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

#add the source files
AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR}/src slam_src )

#headers of the source files
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include" )

#directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

#generate the lib files
ADD_LIBRARY( slam SHARED ${slam_src} )

ADD_LIBRARY( slam_static STATIC ${slam_src} )

#link libraries
TARGET_LINK_LIBRARIES( slam ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( slam_static ${OpenCV_LIBS} )

SET_TARGET_PROPERTIES( slam_static PROPERTIES OUTPUT_NAME "slam" )
