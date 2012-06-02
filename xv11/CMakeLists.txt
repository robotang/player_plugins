# ----------------------------------------------------------------------------
#  CMake file for xv11 rangefinder
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.0)

SET(THE_TARGET "xv11")
PROJECT(${THE_TARGET})
SET(CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules")
INCLUDE(UsePlayerPlugin)

INCLUDE_DIRECTORIES(
	${CMAKE_SOURCE_DIR}
	${PLAYERCORE_EXTRA_INCLUDE_DIRS}
)

SET(THE_TARGET_SOURCES
	xv11.cpp
	laser2d_filter.cpp
)

SET(THE_TARGET_HEADERS
	laser2d_filter.h
)

PLAYER_ADD_PLUGIN_DRIVER(${THE_TARGET} SOURCES ${THE_TARGET_SOURCES})
cmake_policy(SET CMP0002 OLD) #To suppress warning
ADD_LIBRARY(${THE_TARGET} SHARED ${THE_TARGET_SOURCES} ${THE_TARGET_HEADERS})
#ADD_DEPENDENCIES(${THE_TARGET} misc)
TARGET_LINK_LIBRARIES(${THE_TARGET} playercore)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib${THE_TARGET}.so DESTINATION ${PLUGIN_PLAYERC_LIBRARY_DIRS}) 
