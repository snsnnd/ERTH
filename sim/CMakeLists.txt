cmake_minimum_required(VERSION 3.16)
project(erth_sim LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(NNCAR_WARN -Wall -Wextra -Wpedantic -Werror)

add_library(erth_plant STATIC src/erth_sim.c)
target_include_directories(erth_plant PUBLIC include)
target_compile_options(erth_plant PRIVATE ${NNCAR_WARN})
target_link_libraries(erth_plant PUBLIC m)

add_executable(erth_sim src/main.c)
target_compile_options(erth_sim PRIVATE ${NNCAR_WARN})
target_link_libraries(erth_sim PRIVATE erth_plant)

enable_testing()
add_test(NAME erth_baseline COMMAND erth_sim --check)
