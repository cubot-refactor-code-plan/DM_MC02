cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DM_MC02)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# TinyUSB class drivers are selected at compile time. Exactly one transport is
# built into a firmware image so CDC and HID can never run concurrently.
set(DM_MC02_USB_CLASS "HID" CACHE STRING "USB device class: CDC or HID")
set_property(CACHE DM_MC02_USB_CLASS PROPERTY STRINGS CDC HID)
string(TOUPPER "${DM_MC02_USB_CLASS}" DM_MC02_USB_CLASS_NORMALIZED)
if(DM_MC02_USB_CLASS_NORMALIZED STREQUAL "CDC")
    add_compile_definitions(DM_MC02_USB_CLASS_CDC=1 DM_MC02_USB_CLASS_HID=0)
elseif(DM_MC02_USB_CLASS_NORMALIZED STREQUAL "HID")
    add_compile_definitions(DM_MC02_USB_CLASS_CDC=0 DM_MC02_USB_CLASS_HID=1)
else()
    message(FATAL_ERROR "DM_MC02_USB_CLASS must be CDC or HID")
endif()
message(STATUS "USB device class: ${DM_MC02_USB_CLASS_NORMALIZED}")

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)



##### User #####

# Add User's folders to main
add_subdirectory(User)

# Add stm32cubemx to User_lib
target_link_libraries(User_lib PRIVATE stm32cubemx)

# Add User_lib to main
target_link_libraries(${CMAKE_PROJECT_NAME}
    User_lib
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Keep an internal-only image and a raw XIP payload for the two-bank flash flow.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex --remove-section=.usb_xip
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}_internal.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary --only-section=.usb_xip
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}_usb_xip.bin
    COMMENT "Generating internal Flash image and external USB XIP payload"
)

##### User #####

