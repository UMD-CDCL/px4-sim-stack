cmake_minimum_required(VERSION 3.22)
project(gz_video_streamer CXX)

# Two programs: the encoder that turns Gazebo cameras into video streams, and
# the publisher that carries the emulated lens position to it.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Gazebo Harmonic component versions. Change these together with GZ_DISTRO.
#
# Do not add COMPONENTS here. The core library ships inside the main package and
# exports gz-transport13::core directly. Asking for COMPONENTS core makes CMake
# look for a separate gz-transport13-core package, which Harmonic does not ship,
# and the configure step fails.
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
)

add_executable(gz_video_streamer main.cc)

target_link_libraries(gz_video_streamer
  PRIVATE
    gz-transport13::core
    gz-msgs10::core
    PkgConfig::GST
)

# The other half of the simulated camera: the lens. The emulated SCF4
# controller has no Gazebo of its own, so it writes the field of view it is at
# and this puts it on the topic the streamer follows. Built here because it
# needs the same gz-transport and nothing else, and because one install step
# then covers both.
add_executable(gz_zoom_publisher zoom_publisher.cc)

target_link_libraries(gz_zoom_publisher
  PRIVATE
    gz-transport13::core
    gz-msgs10::core
)

install(TARGETS gz_video_streamer gz_zoom_publisher RUNTIME DESTINATION bin)
