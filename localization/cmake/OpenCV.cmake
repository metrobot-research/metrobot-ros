find_package(OpenCV 3.2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBS})
MESSAGE("OpenCV_LIBS:")
MESSAGE(${OpenCV_LIBS})