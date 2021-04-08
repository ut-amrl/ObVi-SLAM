cmake_minimum_required(VERSION 3.1.0)
project(ut_vslam)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include)

set(LIBS glog gflags)

add_library(vslam_io src/vslam_io.cpp)
target_compile_features(vslam_io PRIVATE cxx_std_17)    # 17 required for file iteration 

add_executable(main src/main.cpp)
target_link_libraries(main vslam_io ${LIBS})