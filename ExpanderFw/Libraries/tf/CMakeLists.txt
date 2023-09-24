cmake_minimum_required(VERSION 3.20)
project(tf C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(${PROJECT_NAME}_SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/TinyFrame.c
    ${CMAKE_CURRENT_LIST_DIR}/utilities/payload_builder.c
    ${CMAKE_CURRENT_LIST_DIR}/utilities/payload_parser.c
)

include_directories(demo/simple)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Set the include directories for the hal_driver library
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/.
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/utilities
    ${CMAKE_SOURCE_DIR}/Libraries
)