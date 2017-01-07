project(ImageTest)

add_subdirectory(common)
add_subdirectory(algorithm)
add_subdirectory(distance)
add_subdirectory(lanedetect)
add_subdirectory(lane)
add_subdirectory(ipm)
add_subdirectory(bspline)
#add_subdirectory(chapter2)
#add_subdirectory(CHEVP)

#add_subdirectory(calibrate)
#add_subdirectory(morphological)


#add_subdirectory(calibrate)


set(MY_LINKER_LIBS ${MY_LINKER_LIBS} PARENT_SCOPE)
