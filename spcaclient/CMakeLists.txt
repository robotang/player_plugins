# ----------------------------------------------------------------------------
#  CMake file for spcaclient
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.0)

SET(THE_TARGET "spcaclient")
PROJECT(${THE_TARGET})
SET(CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules")
INCLUDE(UsePlayerPlugin)

INCLUDE_DIRECTORIES(
	/usr/local/include/opencv/
	${CMAKE_SOURCE_DIR}
	${PLAYERCORE_EXTRA_INCLUDE_DIRS}	
)

SET(THE_TARGET_SOURCES
	spcaclient.cpp
	
	misc.c
	avilib.c
	dpsh.c
	encoder.c
	huffman.c
	marker.c
	picture.c
	quant.c
	shc.c
	shclib.c
	tcputils.c
	utils.c
)

SET(THE_TARGET_HEADERS
	misc.h
	avilib.h
	dpsh.h
	encoder.h
	huffman.h
	jconfig.h
	jdatatype.h
	marker.h
	picture.h
	quant.h
	shc.h
	shclib.h
	spcaframe.h
	tcputils.h
	utils.h
)

PLAYER_ADD_PLUGIN_DRIVER(${THE_TARGET} SOURCES ${THE_TARGET_SOURCES})
cmake_policy(SET CMP0002 OLD) #To suppress warning
ADD_LIBRARY(${THE_TARGET} SHARED ${THE_TARGET_SOURCES} ${THE_TARGET_HEADERS})
#ADD_DEPENDENCIES(${THE_TARGET} misc)
TARGET_LINK_LIBRARIES(${THE_TARGET} playercore)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib${THE_TARGET}.so DESTINATION ${PLUGIN_PLAYERC_LIBRARY_DIRS}) 
