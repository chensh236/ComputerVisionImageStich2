cmake_minimum_required(VERSION 3.10)
project(panorama)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

link_directories($ENV{VLROOT}/bin/maci64/)

if (APPLE)
    set (CMAKE_CXX_FLAGS "-lX11 -L/usr/X11/lib -I/usr/X11/include -I$ENV{VLROOT} -L$ENV{VLROOT}/bin/maci64/")
    add_executable(panorama
            CImg.h main.cpp ImageProcess.h ImageProcess.cpp Projection.h Projection.cpp)
    configure_file(libvl.dylib ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libvl.dylib COPYONLY)
    configure_file(libomp.dylib ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libomp.dylib COPYONLY)
    target_link_libraries(panorama-mac libvl.dylib)
endif (APPLE)

if(UNIX AND NOT APPLE)
    add_executable(panorama
            CImg.h main.cpp ImageProcess.h ImageProcess.cpp Projection.h Projection.cpp)
    set (CMAKE_CXX_FLAGS "-lX11 -pthread -I$ENV{VLROOT} -L$ENV{VLROOT}/bin/glnxa64/ -lvl")
    message(STATUS "Looking for X11...")
    find_package(X11 REQUIRED)
    include_directories(${X11_INCLUDE_DIR})
    configure_file(libvl.so ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libvl.so COPYONLY)
    target_link_libraries(panorama-linux libvl.so)
    target_link_libraries(panorama-linux ${X11_LIBRARIES})
endif(UNIX AND NOT APPLE)

if (WIN32)
    set (CMAKE_CXX_FLAGS "-lgdi32 -I$ENV{VLROOT} -L$ENV{VLROOT}/bin/win64/ ")
    set (CMAKE_SHARED_LINKER_FLAGS "-lgdi32 ")
    add_executable(panorama
            CImg.h main.cpp ImageProcess.h ImageProcess.cpp Projection.h Projection.cpp transfer.cpp transfer.h equalization.cpp equalization.h)
    configure_file(vl.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vl.dll COPYONLY)
    target_link_libraries(panorama vl.lib)
endif(WIN32)

include_directories(.)
