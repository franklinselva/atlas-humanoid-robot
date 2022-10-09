cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(HumanoidRobotics)

# FIND PACKAGES
find_package(Bullet CONFIG REQUIRED)

include_directories(include)

file(GLOB_RECURSE SOURCES "src/*.cc")

add_executable(HumanoidRobotics src/main.cc ${SOURCES})

# add_library(HumanoidRoboticsLib SHARED ${SOURCES})

# install(TARGETS HumanoidRoboticsLib DESTINATION libs)

target_link_libraries(HumanoidRobotics PRIVATE LinearMath Bullet3Common BulletDynamics BulletSoftBody)