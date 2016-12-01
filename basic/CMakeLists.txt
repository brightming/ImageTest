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


add_executable(test_dft  test_dft.cpp)
target_link_libraries(test_dft ${MY_LINKER_LIBS})

add_executable(test_dft_rotation  test_dft_rotation.cpp)
target_link_libraries(test_dft_rotation ${MY_LINKER_LIBS})

add_executable(test_generate_black_white  test_generate_black_white.cpp)
target_link_libraries(test_generate_black_white ${MY_LINKER_LIBS})

add_executable(test_color_dist  test_color_dist.cpp)
target_link_libraries(test_color_dist ${MY_LINKER_LIBS})



