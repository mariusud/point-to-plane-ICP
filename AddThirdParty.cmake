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


# Open3D
# find_package(Open3D REQUIRED)
# get_target_property(OPEN3D_INTERFACE_INCLUDES Open3D::Open3D
                    # INTERFACE_INCLUDE_DIRECTORIES)
# message(STATUS "OPEN3D_INTERFACE_INCLUDES: ${OPEN3D_INTERFACE_INCLUDES}")

# ------------------------------------------------------------------------------
# PCL
# ------------------------------------------------------------------------------

find_package(VTK REQUIRED)
find_package(PCL)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# if(NOT PCL_FOUND)
# message(STATUS "PCL not found, adding with FetchContent")
# function(add_PCL)
    # FetchContent_Declare(
    # PCL
    # URL https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.0-rc1/source.tar.gz
    # )
# 
    # FetchContent_MakeAvailable(PCL)
# endfunction()
# 
# add_PCL()
# else()
# message(STATUS "PCL found at ${PCL_DIR}")
# endif()