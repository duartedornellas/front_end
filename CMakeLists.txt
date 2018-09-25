cmake_minimum_required(VERSION 2.8)
project( front_end )
add_executable( front_end front_end.cpp )

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(OpenCV 3.3.1 PATHS /home/ddornellas/libs-ddornellas/OpenCV/installed)
find_package(Boost 1.40 COMPONENTS program_options filesystem system REQUIRED)

include_directories( ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR} "/home/ddornellas/libs-ddornellas/Eigen/installed_dir/")

target_link_libraries( front_end ${OpenCV_LIBS} ${Boost_LIBRARIES})

