cmake_minimum_required(VERSION 2.6)
SET(PROJECT_NAME run)
PROJECT(${PROJECT_NAME})

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/share/OpenCV")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)

find_package(OpenCV REQUIRED)
find_package(aruco  REQUIRED )
message(STATUS "    version: ${OpenCV_VERSION}")

set(ARUCO_PATH /usr/local)
SET(CMAKE_MODULE_PATH ${ARUCO_PATH}/lib/cmake )
SET(ARUCO_INCLUDE_DIRS ${ARUCO_PATH}/include/aruco )
SET(ARUCO_LIBS_DIRS ${ARUCO_PATH}/lib )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${ARUCO_INCLUDE_DIRS}
    "./src/include"
     "./build"
     "/usr/include/eigen3/"
     "/usr/local/include"
)

AUX_SOURCE_DIRECTORY(src DIR_SRCS)

ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS}   ${ARUCO_LIBS_DIRS})

target_link_libraries(${PROJECT_NAME} aruco serial wiringPi)
#target_link_libraries(${PROJECT_NAME} aruco )
#use lib
#target_link_libraries(${PROJECT_NAME}  /home/pi/QT/MF_C/liboldx_mapping.so)
#gen lig
#SET(LIB_SRC   ${DIR_SRCS1})
#ADD_LIBRARY(oldx_mapping SHARED ${LIB_SRC})





