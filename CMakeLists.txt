cmake_minimum_required(VERSION 3.6)

project(SOP_PointDeformByPrim)

list(APPEND CMAKE_PREFIX_PATH "$ENV{HOUDINITREE}/toolkit/cmake")

find_package(Houdini REQUIRED)

set(library_name SOP_PointDeformByPrim)

houdini_generate_proto_headers(FILES SOP_PointDeformByPrim.cpp)

add_library(${library_name} SHARED
    SOP_PointDeformByPrim.cpp
    SOP_PointDeformByPrim.h
    ThreadedPointDeform.cpp
    ThreadedPointDeform.h
    Timer.cpp
    Timer.h
    Utils.h
)

target_link_libraries(${library_name} Houdini)

target_include_directories(${library_name} PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)

houdini_configure_target(${library_name})