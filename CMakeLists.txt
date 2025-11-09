cmake_minimum_required(VERSION 3.20)
project(freecell_sui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -pedantic -O2)

add_executable(
        fc-sui
        card.cc
        card-storage.cc
        evaluation-type.cc
        fc-sui.cc
        game.cc
        mem_watch.cc
        memusage.cc
        move.cc
        search-interface.cc
        strategies-provided.cc
        sui-solution.cc
        test.cc
        test-main.cc
)

# headers are in the root
target_include_directories(fc-sui PRIVATE ${CMAKE_SOURCE_DIR})