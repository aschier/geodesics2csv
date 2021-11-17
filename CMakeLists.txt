cmake_minimum_required(VERSION 3.1)
project(geodesics2csv)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
set(MSVC_RUNTIME "dynamic")

option(LIBIGL_WITH_OPENGL            "Use OpenGL"         OFF)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use IMGUI"          OFF)

find_package(Eigen3 REQUIRED QUIET)
find_package(LIBIGL REQUIRED QUIET)

add_executable(geodesics2csv
	geodesics2csv.cpp
)
target_link_libraries(geodesics2csv
	igl::core
)
install(TARGETS geodesics2csv RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
