cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
/usr/local/include
)
link_directories(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib)

add_executable(${PROJECT_NAME}
src/ORBVO.cpp
src/ORBextractor.cc
src/ORBmatcher.cc
src/Frame.cc
src/Initializer.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
DBoW2
)

