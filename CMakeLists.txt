add_library(motors INTERFACE)

target_sources(motors INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/trinamic.c
)

target_include_directories(motors INTERFACE ${CMAKE_CURRENT_LIST_DIR})
