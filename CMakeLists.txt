cmake_minimum_required(VERSION 2.4.6)
project(SIM_ORB_SLAM)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
/usr/local/include/eigen3
/usr/local/include
)
link_directories(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib
)

add_executable(${PROJECT_NAME}
src/ORB_tracking.cpp
src/ORBextractor.cc
src/ORBmatcher.cc
src/Frame.cc
src/Initializer.cc
src/Map.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Optimizer.cc
src/Converter.cc
src/Tracking.cc


)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
DBoW2
g2o
)

