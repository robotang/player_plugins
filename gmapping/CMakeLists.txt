# ----------------------------------------------------------------------------
#  CMake file for gmapping
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.0)

SET(THE_TARGET "gmapping")
PROJECT(${THE_TARGET})
SET(CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules")
INCLUDE(UsePlayerPlugin)

SET(GMAPPING_DIR "${CMAKE_SOURCE_DIR}/gmapping/gmapping")

SET(GMAPPING_LIBS
	gridfastslam 
	sensor_odometry 
	sensor_range 
	utils 
	scanmatcher
)

INCLUDE_DIRECTORIES(
#	/usr/local/include/opencv/
	${GMAPPING_DIR}
	${CMAKE_SOURCE_DIR}
	${PLAYERCORE_EXTRA_INCLUDE_DIRS}
)

SET(THE_TARGET_SOURCES
	gmapping.cpp
	slam_gmapping.cpp
)

SET(THE_TARGET_HEADERS
	slam_gmapping.h
)

PLAYER_ADD_PLUGIN_DRIVER(${THE_TARGET} SOURCES ${THE_TARGET_SOURCES})
cmake_policy(SET CMP0002 OLD) #To suppress warning
LINK_DIRECTORIES(${GMAPPING_DIR}/lib)
ADD_LIBRARY(${THE_TARGET} SHARED ${THE_TARGET_SOURCES} ${THE_TARGET_HEADERS})
TARGET_LINK_LIBRARIES(${THE_TARGET} playercore ${GMAPPING_LIBS} pthread)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib${THE_TARGET}.so DESTINATION ${PLUGIN_PLAYERC_LIBRARY_DIRS}) 
