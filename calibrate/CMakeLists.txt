cmake_minimum_required(VERSION 2.8.3)
project(calibrate)


add_executable(calibrate_intrinsic calibrate_intrinsic.cpp)
target_link_libraries(calibrate_intrinsic ${MY_LINKER_LIBS})
