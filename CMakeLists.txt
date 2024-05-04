cmake_minimum_required(VERSION 3.20)

project("software_spi" C CXX ASM)

# Export compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/st-project.cmake)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})