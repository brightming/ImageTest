cmake_minimum_required(VERSION 2.8.3)
project(test_hist)


add_executable(test_hist test_hist.cpp)
target_link_libraries(test_hist ${MY_LINKER_LIBS})
