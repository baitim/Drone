cmake_minimum_required(VERSION 3.16)
project(simple)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/../esp-idf-lib-master/components)