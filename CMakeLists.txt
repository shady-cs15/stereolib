CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(StereoVisionLibrary)

FIND_PACKAGE(OpenCV REQUIRED)

IF(OpenCV_FOUND)
	MESSAGE(STATUS "OpenCV Found")
ELSE()
	MESSAGE(FATAL_ERROR "OpenCV Not FOUND")
ENDIF()

SET(SOURCE main.cpp ./calib/stereocalib.cpp ./rectify/stereorect.cpp)
SET(SOURCE_TEST main_test.cpp)
SET(SOURCE_MATCH main_corres.cpp ./corres/stereomatch.cpp)

IF(OpenCV_FOUND)
	ADD_EXECUTABLE(stereocalib ${SOURCE})
	TARGET_LINK_LIBRARIES(stereocalib ${OpenCV_LIBS})
	ADD_EXECUTABLE(stereotest ${SOURCE_TEST})
	TARGET_LINK_LIBRARIES(stereotest ${OpenCV_LIBS})
	ADD_EXECUTABLE(stereomatch ${SOURCE_MATCH})
	TARGET_LINK_LIBRARIES(stereomatch ${OpenCV_LIBS})
ENDIF()
