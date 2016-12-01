cmake_minimum_required(VERSION 2.8.3)
project(morphological)


add_executable(test_watersheds test_watersheds.cpp)
target_link_libraries(test_watersheds ${MY_LINKER_LIBS})




