#=============================================================================#
# Author: QueezyTheGreat                                                      #
# Date:   26.04.2011                                                          #
#                                                                             #
# Description: Arduino CMake example                                          #
#                                                                             #
#=============================================================================#
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake) # Arduino Toolchain

set(ARDUINO_C_FLAGS      "-ffunction-sections -fdata-sections -save-temps -fverbose-asm")
set(ARDUINO_CXX_FLAGS    "${ARDUINO_C_FLAGS} -fno-exceptions")
set(ARDUINO_LINKER_FLAGS "-Wl,--gc-sections -Wl,-M,arduino.map")

cmake_minimum_required(VERSION 2.8)
#====================================================================#
#  Setup Project                                                     #
#====================================================================#
project(Asserv3RouesI2C C CXX)

generate_arduino_firmware(master
        BOARD uno
        PORT /dev/ttyUSB0
        SRCS master/master.cpp master/ESP8266.cpp
)

generate_arduino_firmware(slave1
        BOARD nano328
        PORT /dev/ttyUSB0
        SRCS slaves/slave1.cpp
)

generate_arduino_firmware(slave2
        BOARD nano328
        PORT /dev/ttyUSB0
        SRCS slaves/slave2.cpp
)

generate_arduino_firmware(slave3
        BOARD nano328
        PORT /dev/ttyUSB0
        SRCS slaves/slave3.cpp
)
