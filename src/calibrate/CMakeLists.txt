cmake_minimum_required(VERSION 2.8.3)
project(calibrate)


add_executable(calibrate_intrinsic calibrate_intrinsic.cpp)
target_link_libraries(calibrate_intrinsic ${MY_LINKER_LIBS})

#add_executable(stereo_calib_2_4 stereo_calib_2_4.cpp)
#target_link_libraries(stereo_calib_2_4 ${MY_LINKER_LIBS})


add_executable(stereo_calib_3_1 stereo_calib_3_1.cpp)
target_link_libraries(stereo_calib_3_1 ${MY_LINKER_LIBS})

add_executable(stereo_match_3_1 stereo_match_3_1.cpp)
target_link_libraries(stereo_match_3_1 ${MY_LINKER_LIBS})

add_executable(SBM_Sample_3_1 SBM_Sample_3_1.cpp)
target_link_libraries(SBM_Sample_3_1 ${MY_LINKER_LIBS})

add_executable(disparity_filtering disparity_filtering.cpp)
target_link_libraries(disparity_filtering ${MY_LINKER_LIBS})

add_executable(calibration_3_1 calibration_3_1.cpp)
target_link_libraries(calibration_3_1 ${MY_LINKER_LIBS})



