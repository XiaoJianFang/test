
cmake_minimum_required(VERSION 3.8)


add_executable(DemoMap LookupPOI.cpp)
add_executable(DemoMap2 LookupText.cpp)

set_target_properties(DemoMap PROPERTIES CXX_STANDARD 11)
set_target_properties(DemoMap2 PROPERTIES CXX_STANDARD 11)


target_link_libraries(DemoMap libosmscout.so )
target_link_libraries(DemoMap2 libosmscout.so )


