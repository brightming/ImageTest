cmake_minimum_required(VERSION 2.8.3)
project(math)

aux_source_directory(./ src_lists)
add_library(${PROJECT_NAME} ${src_lists})
list(INSERT MY_LINKER_LIBS 0 ${PROJECT_NAME})
set(MY_LINKER_LIBS ${MY_LINKER_LIBS} PARENT_SCOPE)




