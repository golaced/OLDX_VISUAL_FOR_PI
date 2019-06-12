cmake_minimum_required(VERSION 2.6)
SET(PROJECT_NAME run)
PROJECT(${PROJECT_NAME})

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/share/OpenCV")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)

find_package(OpenCV  REQUIRED)
message(STATUS "    version: ${OpenCV_VERSION}")

set(ARUCO_PATH /usr/local)
SET(CMAKE_MODULE_PATH ${ARUCO_PATH}/lib/cmake )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    "./src/include"
     "./build"
     "/usr/include/eigen3/"
     "/usr/local/include"
)

AUX_SOURCE_DIRECTORY(src DIR_SRCS)

ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})

target_link_libraries(${PROJECT_NAME}  serial wiringPi)






