# ----------------------------------------------------------------------------
#  CMake file for devbot
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.0)

SET(THE_TARGET "devbot")
PROJECT(${THE_TARGET})
SET(CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules")
INCLUDE(UsePlayerPlugin)

INCLUDE_DIRECTORIES(
	/usr/local/include/opencv/
	${CMAKE_SOURCE_DIR}
	${PLAYERCORE_EXTRA_INCLUDE_DIRS}
)

SET(THE_TARGET_SOURCES
	devbot.cpp
	devbot_player_position2d.cpp
	devbot_player_imu.cpp
	devbot_player_power.cpp
	devbot_player_joystick.cpp
	devbot_protocol.c
	ring.c
)

SET(THE_TARGET_HEADERS
	devbot_player_position2d.h
	devbot_player_imu.h
	devbot_player_power.h
	devbot_player_joystick.h
	devbot_protocol.h
	ring.h
)

PLAYER_ADD_PLUGIN_DRIVER(${THE_TARGET} SOURCES ${THE_TARGET_SOURCES})
cmake_policy(SET CMP0002 OLD) #To suppress warning
ADD_LIBRARY(${THE_TARGET} SHARED ${THE_TARGET_SOURCES} ${THE_TARGET_HEADERS})
#ADD_DEPENDENCIES(${THE_TARGET} misc)
TARGET_LINK_LIBRARIES(${THE_TARGET} playercore)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib${THE_TARGET}.so DESTINATION ${PLUGIN_PLAYERC_LIBRARY_DIRS}) 
