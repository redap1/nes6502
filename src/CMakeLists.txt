add_library(nes_core
    CPU/nes6502.cpp
    CPU/nes6502.h
    Bus/Bus.h
    Bus/Bus.cpp
)

target_include_directories(nes_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})