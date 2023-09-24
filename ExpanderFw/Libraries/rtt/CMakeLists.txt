cmake_minimum_required(VERSION 3.20)
project(rtt C ASM)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add the source files for rtt
set(${PROJECT_NAME}_SOURCES  
    ${CMAKE_CURRENT_LIST_DIR}/RTT/SEGGER_RTT.c
    ${CMAKE_CURRENT_LIST_DIR}/RTT/SEGGER_RTT_printf.c
)

# Create the static library target
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Set the include directories for the rtt library
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/.
    ${CMAKE_CURRENT_LIST_DIR}/Config
)
