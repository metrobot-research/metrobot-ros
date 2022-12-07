find_package(OpenCV 3.2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories("/usr/local/lib/")
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBS})
MESSAGE("OpenCV_LIBS:")
MESSAGE(${OpenCV_LIBS})