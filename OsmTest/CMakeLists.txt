
cmake_minimum_required(VERSION 3.8)
set(SOURCE_FILES demo.cpp )

add_executable(DemoMap ${SOURCE_FILES} ${HEADER_FILES})

set_target_properties(DemoMap PROPERTIES CXX_STANDARD 11)

target_include_directories(DemoMap PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/include)

target_link_libraries(DemoMap libosmscout.so )
