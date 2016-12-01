project(ImageTest)

add_subdirectory(common)
add_subdirectory(distance)
add_subdirectory(basic)
add_subdirectory(showgrayval)
add_subdirectory(hist)
add_subdirectory(chapter2)
add_subdirectory(CHEVP)

add_subdirectory(calibrate)
add_subdirectory(morphological)


#add_subdirectory(calibrate)


set(MY_LINKER_LIBS ${MY_LINKER_LIBS} PARENT_SCOPE)
