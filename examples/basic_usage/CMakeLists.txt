# The following lines of boilerplate have to be in your project's
# CMakeLists.txt. In this file we just import the global project.cmake
# file which takes care of the rest.

cmake_minimum_required(VERSION 3.16)

# Set default target to ESP32-S3
if(NOT DEFINED IDF_TARGET)
    set(IDF_TARGET esp32s3)
endif()

# Set the components to include the components from the parent directory
set(EXTRA_COMPONENT_DIRS "../../")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rv3032_basic_usage)
