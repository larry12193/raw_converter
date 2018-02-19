cmake_minimum_required(VERSION 2.8.3)
project(raw_converter)
add_compile_options(-std=c++11)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

find_package( Boost REQUIRED COMPONENTS thread system program_options filesystem)
find_package( OpenCV REQUIRED )

include_directories(include
                    ${Boost_INCLUDE_DIRS})

# Define sources and executable
set(EXECUTABLE_NAME "raw_converter")
add_executable(${EXECUTABLE_NAME} src/raw_converter.cpp)

target_link_libraries(${EXECUTABLE_NAME}
                      pthread
                      ${Boost_LIBRARIES}
                      ${OpenCV_LIBS})

# Install target
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
