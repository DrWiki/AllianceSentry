cmake_minimum_required(VERSION 3.5)
project(Alliance2019_Sentry)
set(CMAKE_CXX_STANDARD 14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -O3 -pthread" )
set(OPENCV_VERSION 3.4.5)

#Forbid to overwrite the content behind without the permission of SongZiwu
####################################################################################
message("
       _    _ _  _     _    _ _ _
      / \\  (_) || |   / \\  | | (_) __ _ _ __   ___ ___
     / _ \\ | | || |_ / _ \\ | | | |/ _` | '_ \\ / __/ _ \\
    / ___ \\| |__   _/ ___ \\| | | | (_| | | | | (_|  __/
   /_/   \\_\\_|  |_|/_/   \\_\\_|_|_|\\__,_|_| |_|\\___\\___|

")
####################################################################################
###Compile Externally
#find_package(Qt5 5.5.1 REQUIRED QtCore QtGui QtXml)
#include(${QT_USE_FILE})
#add_executable(myexe main.cpp)
#target_link_libraries(myexe ${QT_LIBRARIES})


set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "FATAL: In-source builds are not allowed.
              You should create a separate directory for build files.")
else()
    message(STATUS "CMAKE_SOURCE_DIR : " ${CMAKE_SOURCE_DIR})
    message("   " "CMAKE_BINARY_DIR : " ${CMAKE_BINARY_DIR})
endif()
###OpenCV Module
find_package(/usr/local/opencv345/share/OpenCV/OpenCV  ${OPENCV_VERSION} REQUIRED)

#find_package(/usr/local/opencv401/lib/cmake/opencv4/OpenCV  ${OPENCV_VERSION} REQUIRED)
if (OpenCV_FOUND) # Find how to use NOT in CMakeLists.txt
    message(STATUS "OpenCV Version : " ${OPENCV_VERSION} " found successfully")
    message("   " OpenCV_INCLUDE_DIRS " : " ${OpenCV_INCLUDE_DIRS})
    message("   " OpenCV_LIBS " : " ${OpenCV_LIBS})
    include_directories(${OpenCV_INCLUDE_DIRS})
    #message(FATAL_ERROR "OpenCV Version : " ${OPENCV_VERSION} " has not been found!!!")
else()
    message(FATAL_ERROR "OpenCV Version : " ${OPENCV_VERSION} " has not been found!!!")
endif (OpenCV_FOUND)

###Eigen3 Module
find_package(Eigen3 REQUIRED NO_MODULE)
if(EIGEN3_FOUND)
    message(STATUS "Eigen3 : found successfully")
else()
    message(FATAL_ERROR "Eigen3 :  has not been found!!!")
endif(EIGEN3_FOUND)

###Include And Src Part
include_directories(include)
include_directories(include/IndustrialCamera)
link_libraries("/lib/libMVSDK.so")
add_executable(${PROJECT_NAME} src/main.cpp
        include/IndustrialCamera/CameraApi.h
        include/IndustrialCamera/CameraDefine.h
        include/IndustrialCamera/CameraStatus.h
#        src/libMVSDK.so
        include/AllianceTool_SZW.h
        include/LedBar.h                     src/LedBar.cpp
        include/Setter.h                     src/Setter.cpp
        include/RoboMaster2019.h             src/RoboMaster2019.cpp
        include/Calibrator.h                 src/Calibrator.cpp
        include/AllianceVideoCapture.h       src/AllianceVideoCapture.cpp
        include/MindVisionCamera.h           src/MindVisionCamera.cpp
        include/MicrosecondChronograph.h     src/MicrosecondChronograph.cpp
        include/Armor.h                      src/Armor.cpp
        include/PnP.h                        src/PnP.cpp
        include/SerialPort.h                 src/SerialPort.cpp
        include/TimerBase.h                  src/TimerBase.cpp
        include/TimerBaseClock.h             src/TimerBaseClock.cpp
        include/Analyze.h                    src/Analyze.cpp
        include/ArmorNumberRecgnation.h      src/ArmorNumberRecgnation.cpp
        unused/KalmanPredict_.h              unused/KalmanPredict_.cpp
        src/KalmanFilter.cpp
        include/KalmanFilter.hpp
        src/ArmorFind.cpp
        include/ArmorFind.h
        src/AnalyzeStatistics.cpp
        include/AnalyzeStatistics.h
        src/KalmanFilterAll.cpp
        include/KalmanFilterAll.h)

###Lib Part

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
####################################################################################
message("
   #                                                     #####    ###     #    #####
  # #   #      #      #   ##   #    #  ####  ######     #     #  #   #   ##   #     #
 #   #  #      #      #  #  #  ##   # #    # #                # #     # # #   #     #
#     # #      #      # #    # # #  # #      #####       #####  #     #   #    ######
####### #      #      # ###### #  # # #      #          #       #     #   #         #
#     # #      #      # #    # #   ## #    # #          #        #   #    #   #     #
#     # ###### ###### # #    # #    #  ####  ######     #######   ###   #####  #####
")
####################################################################################
