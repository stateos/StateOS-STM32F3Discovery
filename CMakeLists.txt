cmake_minimum_required(VERSION 3.8)

set(CMAKE_TOOLCHAIN_FILE "common/stateos/cmake/config-stm32f3discovery-gcc.cmake")
set(CMAKE_BUILD_TYPE     "Release")

project(test)

find_package(startup REQUIRED)
find_package(device  REQUIRED)
find_package(stateos REQUIRED)

add_executable(test
	src/main.c
)

target_include_directories(test
	PRIVATE
	include
)

target_link_libraries(test
	PRIVATE
	startup
	device::nosys
	stateos::kernel
)

setup_target(test)
