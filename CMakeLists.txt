cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "RELEASE")

set(CMAKE_C_FLAGS_DEBUG "-g -Wall ")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(/opt/vc/include)
include_directories(/opt/vc/include/interface/vcos/pthreads)
include_directories(/opt/vc/include/interface/vmcs_host)
include_directories(/opt/vc/include/interface/vmcs_host/linux)

include_directories(/home/pi/src/raspberrypi/userland)
include_directories(/opt/vc/src/hello_pi/libs/vgfont)

link_directories(/opt/vc/lib)
link_directories(/opt/vc/src/hello_pi/libs/vgfont)

add_executable(snoopmon RaspiCamControl.c snoopmon.c)

find_package( OpenCV REQUIRED )

target_link_libraries(snoopmon mmal_core mmal_util mmal_vc_client vcos bcm_host ${OpenCV_LIBS} vgfont openmaxil EGL)
