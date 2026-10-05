find_package(Eigen3 CONFIG REQUIRED)

add_library(cfd_solvers STATIC
    src/kernels_scalar.cpp
    src/kernels_threaded.cpp
    src/navier_stokes_3d.cpp
    src/navier_stokes_2d.cpp
    src/voxelizer.cpp
    src/orientation.cpp
    src/scenario_presets_2d.cpp
    src/image_scenario_2d.cpp
    src/performance_presets_3d.cpp
    src/materials.cpp
)

target_include_directories(cfd_solvers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cfd_solvers PUBLIC cfd_core cfd_mesh Eigen3::Eigen)
find_package(Threads REQUIRED)
target_link_libraries(cfd_solvers PRIVATE Threads::Threads)

cfd_apply_warnings(cfd_solvers)
cfd_apply_static_runtime(cfd_solvers)
