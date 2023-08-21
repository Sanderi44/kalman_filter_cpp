cmake_minimum_required(VERSION 3.0.0)
project(kalman_filter VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
add_library(kalman_filter_lib kalman_filter.cpp)
add_library(car_lib car.cpp)

# populate matplotlib repository
include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

add_executable(kalman_filter main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
target_include_directories(kalman_filter PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})

target_link_libraries(
    kalman_filter
    kalman_filter_lib
    car_lib
    Eigen3::Eigen
    ${PYTHON_LIBRARIES}
    Python3::NumPy
    )

install(TARGETS kalman_filter DESTINATION bin)


