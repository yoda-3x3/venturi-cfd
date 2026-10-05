add_executable(cfd_tests
    ${CMAKE_SOURCE_DIR}/vendor/catch2/catch_amalgamated.cpp
    core/test_connected_components.cpp
    core/test_hashing.cpp
    solvers/test_kernels_scalar.cpp
    solvers/test_kernels_threaded.cpp
    solvers/test_navier_stokes_3d.cpp
    solvers/test_navier_stokes_2d.cpp
    solvers/test_navier_stokes_2d_regression.cpp
    solvers/test_image_scenario_2d.cpp
    solvers/test_voxelizer.cpp
    solvers/test_orientation.cpp
    solvers/test_scenario_presets.cpp
    solvers/test_materials.cpp
    mesh/test_readers.cpp
    mesh/test_mesh_bvh.cpp
    mesh/test_convex_hull2d.cpp
    mesh/test_cap_boundaries.cpp
    mesh/test_primitives.cpp
    io/test_openfoam_writer.cpp
    io/test_vtk_writer.cpp
    io/test_run_cache.cpp
    io/test_results_cache.cpp
    pipeline/test_run_2d.cpp
    pipeline/test_run_3d.cpp
    pipeline/test_preview.cpp
)

target_include_directories(cfd_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/vendor/catch2
)

target_compile_definitions(cfd_tests PRIVATE
    CFD_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mesh/fixtures"
)

target_link_libraries(cfd_tests PRIVATE cfd_core cfd_mesh cfd_solvers cfd_io cfd_pipeline)

cfd_apply_static_runtime(cfd_tests)
cfd_copy_embree_dlls(cfd_tests)

add_test(NAME cfd_tests COMMAND cfd_tests)
