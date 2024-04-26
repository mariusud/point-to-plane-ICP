# fmtlib
find_package(fmt 8.1 REQUIRED)
get_target_property(FMT_INTERFACE_INCLUDES fmt::fmt
                    INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "FMT_INTERFACE_INCLUDES: ${FMT_INTERFACE_INCLUDES}")

# spdlog
find_package(spdlog 1.9.0 REQUIRED)
get_target_property(SPDLOG_INTERFACE_INCLUDES spdlog::spdlog
                    INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "SPDLOG_INTERFACE_INCLUDES: ${SPDLOG_INTERFACE_INCLUDES}")

# PCL
find_package(VTK REQUIRED)
find_package(PCL)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

