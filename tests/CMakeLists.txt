include(FetchContent)
FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.8.1
)

FetchContent_MakeAvailable(Catch2)

add_executable(cpu_tests cpu_tests.cpp)
target_link_libraries(cpu_tests PRIVATE nes_core Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(cpu_tests)