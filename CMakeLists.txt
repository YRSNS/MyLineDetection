cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 14)

PROJECT(SDC_line)
find_package(Eigen3 REQUIRED)
MESSAGE( [Main] " EIGEN3_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}")
#EIGEN3_INCLUDE_DIRS = /usr/include/eigen3
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package( OpenCV REQUIRED ) #find_package( OpenCV 4 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS})

MACRO(add_example name)
  ADD_EXECUTABLE(${name} src/${name}.cpp src/cam.h src/functions.h src/laneDetection.h)
  TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBS})
ENDMACRO()

#file(GLOB H_FILES "src/cam/*.h" )
#add_executable(app src/main.cpp src/cam.h ${H_FILES})
add_example(main)
#add_example(basic_functions)
#add_example(array_types)
#add_example(split_channels)
#add_example(drawing_functions)
#add_example(simple_webcam)
#add_example(record_vid_cam)
#add_example(record_video)
#add_example(bitwise_ops)
#add_example(trackbar_wsum)
#add_example(mouse_events)
#add_example(mouse_userdata_buttons)
#add_example(inrange_hsv)
