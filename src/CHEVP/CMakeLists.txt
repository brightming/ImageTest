cmake_minimum_required(VERSION 2.8.3)
project(chevp)

INCLUDE_DIRECTORIES(./)
add_executable(test_chevp test_chevp.cpp MSAC.cpp str_common.cpp)
target_link_libraries(test_chevp ${MY_LINKER_LIBS})
