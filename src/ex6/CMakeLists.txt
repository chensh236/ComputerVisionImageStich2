cmake_minimum_required(VERSION 3.10)
project(panorama)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

link_directories($ENV{VLROOT}/bin/maci64/)


if (WIN32)
    set (CMAKE_CXX_FLAGS "-lgdi32 -I$ENV{VLROOT} -L$ENV{VLROOT}/bin/win64/ ")
    set (CMAKE_SHARED_LINKER_FLAGS "-lgdi32 ")
    add_executable(panorama
            CImg.h main.cpp ImageProcess.h ImageProcess.cpp Projection.h Projection.cpp equalization.cpp equalization.h)
    configure_file(vl.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vl.dll COPYONLY)
    target_link_libraries(panorama vl.lib)
endif(WIN32)

include_directories(.)
