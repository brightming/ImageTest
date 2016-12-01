
#file(GLOB_RECURSE src_lists "${PROJECT_SOURCE_DIR}/test /*.cpp")
aux_source_directory(./ src_lists)
message(STATUS "-----test srcs:${src_lists}")

foreach(source_file  ${src_lists})

  message(STATUS "-----source_file srcs:${source_file}")
  # get file name
  get_filename_component(name ${source_file} NAME_WE)

  # get folder name
  get_filename_component(path ${source_file} PATH)
  get_filename_component(folder ${path} NAME_WE)

  add_executable(${name} ${source_file})
  target_link_libraries(${name} ${MY_LINKER_LIBS})
endforeach(source_file)
