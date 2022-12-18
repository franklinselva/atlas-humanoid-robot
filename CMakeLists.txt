cmake_minimum_required(VERSION 3.25)
project(HumanoidRobotics VERSION 0.0.1 LANGUAGES CXX DESCRIPTION "Humanoid Robotics Project")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CXX_FLAGS "-Wall")
set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

# FIND PACKAGES
find_package(Bullet CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

# INCLUDE DIRECTORIES
file(GLOB_RECURSE SOURCES "src/*.cc")
file(GLOB_RECURSE HEADERS "include/*.h")

# ADD EXECUTABLE
add_executable(HumanoidRobotics src/main.cc ${HEADERS} ${SOURCES})
target_include_directories(HumanoidRobotics PRIVATE include)
target_link_libraries(HumanoidRobotics PRIVATE ${BULLET_LIBRARIES} spdlog::spdlog)

set_target_properties(HumanoidRobotics PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    COMPILE_DEFINITIONS "USE_GUI"
)

# ADD LIBRARY
# add_library(HumanoidRoboticsLib SHARED ${SOURCES})

# install(TARGETS HumanoidRoboticsLib DESTINATION libs)
