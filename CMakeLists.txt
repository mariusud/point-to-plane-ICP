cmake_minimum_required(VERSION 3.19...3.25)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.27)
  message(FATAL_ERROR
    "SymForce does not support CMake >=3.27: https://github.com/symforce-org/symforce/issues/356"
    )
endif()

project(symforce_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# Set the installation directory path
set(CMAKE_INSTALL_PREFIX "/home/marius/Development/symforce")

include(AddThirdParty.cmake)
include(FetchContent)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Eigen REQUIRED)
find_package(Symforce REQUIRED)



# Add the executable target
add_executable(run_dynamic_size src/run_dynamic_size.cc)

# Add dependencies
add_dependencies(
    run_dynamic_size
    Eigen::Eigen
    fmt::fmt
    spdlog::spdlog
    symforce::gen
    symforce::cholesky
    symforce::opt
)

# Link libraries
target_link_libraries(
    run_dynamic_size
    Eigen::Eigen
    fmt::fmt
    spdlog::spdlog
    symforce::gen
    symforce::cholesky
    symforce::opt
)

# Install target
install(TARGETS run_dynamic_size DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)