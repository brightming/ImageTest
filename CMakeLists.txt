cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

#opencv
set(OpenCV_DIR /home/gumh/OpenSource/opencv-2.4.9/build)
find_package(OpenCV REQUIRED)

list(APPEND MY_LINKER_LIBS ${OpenCV_LIBRARIES})

# ---[ Google-glog
include("cmake/External/glog.cmake")
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
list(APPEND MY_LINKER_LIBS ${GLOG_LIBRARIES})


include_directories(${OpenCV_INCLUDE_DIRS})
add_subdirectory(basic)
add_subdirectory(hist)
add_subdirectory(chapter2)
add_subdirectory(calibrate)
add_subdirectory(CHEVP)
