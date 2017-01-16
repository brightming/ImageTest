cmake_minimum_required(VERSION 2.8.3)

project("ImageTest")


set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)


set(CMAKE_BUILD_TYPE "DEBUG")

#opencv
set(opencv_version 3)
if(${opencv_version} STREQUAL 3)
    set(OpenCV_DIR /home/gumh/OpenSource/opencv-3.2.0/build)
    set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -DOPENCV_VERSION3")
else()
    set(OpenCV_DIR /home/gumh/OpenSource/opencv-2.4.13/build)
endif()
#set(CMAKE_PREFIX_PATH /home/gumh/OpenSource/opencv-2.4.9/build)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND MY_LINKER_LIBS ${OpenCV_LIBRARIES})

# ---[ Google-glog
include("cmake/External/glog.cmake")
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
list(APPEND MY_LINKER_LIBS ${GLOG_LIBRARIES})


#--include ---##
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)


set(BUILD_SHARED_LIBS ON)
## --- srcs -----##
add_subdirectory(src)
message(STATUS "srcs----MY_LINKER_LIBS=====${MY_LINKER_LIBS}")
add_subdirectory(test)

