cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(HumanoidRobotics)

# FIND PACKAGES
find_package(Bullet CONFIG REQUIRED)

include_directories(include)
include_directories(${BULLET_INCLUDE_DIRS})

file(GLOB_RECURSE SOURCES "src/*.cc")
file(GLOB_RECURSE HEADERS "include/*.h")

add_executable(HumanoidRobotics src/main.cc ${HEADERS} ${SOURCES})

set_target_properties(HumanoidRobotics PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    COMPILE_DEFINITIONS "USE_GUI"
)
# add_library(HumanoidRoboticsLib SHARED ${SOURCES})

# install(TARGETS HumanoidRoboticsLib DESTINATION libs)

target_link_libraries(HumanoidRobotics PRIVATE ${BULLET_LIBRARIES})