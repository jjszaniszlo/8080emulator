cmake_minimum_required(VERSION 3.24.0)

project(6052emu)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)
set(CMAKE_INSTALL_PREFIX ..)
set(LOG_ENABLED true)

add_executable(6052emu src/main.c
               src/logger.c
               src/loggerconf.c
               src/cpu.c)

include_directories(6052emu include /opt/homebrew/include)

install(
        TARGETS 6052emu 
        RUNTIME DESTINATION bin
)
