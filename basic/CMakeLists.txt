cmake_minimum_required(VERSION 2.8.3)
project(basic)


add_executable(test_shared_ptr  test_shared_ptr.cpp)
target_link_libraries(test_shared_ptr ${MY_LINKER_LIBS})

add_executable(test_poly_line  test_poly_line.cpp)
target_link_libraries(test_poly_line ${MY_LINKER_LIBS})

add_executable(test_hough_line  test_hough_line.cpp)
target_link_libraries(test_hough_line ${MY_LINKER_LIBS})

add_executable(convert_minst_to_jpg  convert_minst_to_jpg.cpp)
target_link_libraries(convert_minst_to_jpg ${MY_LINKER_LIBS})


