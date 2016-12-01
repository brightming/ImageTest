cmake_minimum_required(VERSION 2.8.3)
project(distance)

aux_source_directory(./ src_lists)
add_library(${PROJECT_NAME} ${src_lists})
list(APPEND MY_LINKER_LIBS ${PROJECT_NAME})
set(MY_LINKER_LIBS ${MY_LINKER_LIBS} PARENT_SCOPE)




