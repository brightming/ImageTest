
#file(GLOB_RECURSE src_lists "${PROJECT_SOURCE_DIR}/test /*.cpp")
aux_source_directory(./ src_lists)
message(STATUS "-----test srcs:${src_lists}")

set(need_srcs "bbox_label_tool;test_digit;test_coord_transfer")
#list(APPEND need_srcs "test_lane_detect")
#list(APPEND need_srcs "test_nightlane_detect")
#list(APPEND need_srcs "test_CubicSplineInterpolation")



message(STATUS "need_srcs=${need_srcs}")
set(CMAKE_BUILD_TYPE DEBUG)
foreach(source_file  ${src_lists})

  message(STATUS "-----source_file srcs:${source_file}")
  # get file name
  get_filename_component(name ${source_file} NAME_WE)
  # get folder name
  get_filename_component(path ${source_file} PATH)
  get_filename_component(folder ${path} NAME_WE)
  list(LENGTH need_srcs len)
  if(${len} GREATER 0)
	  list(FIND need_srcs ${name} _idx)
	  if(${_idx} GREATER -1)
		  add_executable(${name} ${source_file})
		  target_link_libraries(${name} ${MY_LINKER_LIBS})
	  endif()
  else()
	  add_executable(${name} ${source_file})
	  target_link_libraries(${name} ${MY_LINKER_LIBS})
  endif()



endforeach(source_file)
