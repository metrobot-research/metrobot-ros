find_package(OpenCV 3.2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories("/usr/local/lib") # fixme: find these dir.s by "find /usr -name libopencv*" on Linux
link_directories("/usr/lib/x86_64-linux-gnu") # for Warren's computer and possibly most Linux Ubuntu system
link_directories("/usr/lib/aarch64-linux-gnu") # for JetsonNano
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBS})
MESSAGE("OpenCV_LIBS:")
MESSAGE(${OpenCV_LIBS})