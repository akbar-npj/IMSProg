cmake_minimum_required(VERSION 3.10.0)
project(IMSProg LANGUAGES CXX C)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    add_link_options("-Wl,-z,max-page-size=65536")
endif()


add_subdirectory("IMSProg_programmer")
add_subdirectory("IMSProg_editor")
add_subdirectory("IMSProg_database_update")
