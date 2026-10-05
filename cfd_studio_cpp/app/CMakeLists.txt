find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets OpenGL)
find_package(WebP CONFIG REQUIRED)

qt_standard_project_setup()
set(CMAKE_AUTORCC ON)

qt_add_executable(cfd_studio
    src/main.cpp
    src/main_window.cpp
    src/theme.cpp
    src/tab_2d.cpp
    src/tab_3d.cpp
    src/sim2d_worker.cpp
    src/sim3d_worker.cpp
    src/paraview_launcher.cpp
    src/orientation_dialog.cpp
    src/results_viewer_dialog.cpp
    src/splash_screen.cpp
    src/colormap.cpp
    src/image_scenario_loader.cpp
    src/widgets/plot_widget.cpp
    src/widgets/mesh_preview_widget.cpp
    src/widgets/results_viewer_widget.cpp
    src/widgets/color_legend_widget.cpp
    src/widgets/direction_arrow_widget.cpp
    resources/app_icon.qrc
    resources/app_icon.rc
)

# GUI subsystem (-mwindows): without this MinGW links a console-subsystem
# exe, so Windows opens a terminal window alongside the app on every launch.
set_target_properties(cfd_studio PROPERTIES WIN32_EXECUTABLE TRUE)

target_include_directories(cfd_studio PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(cfd_studio PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    cfd_pipeline
    WebP::webp
)

cfd_apply_warnings(cfd_studio)
# Deliberately NOT cfd_apply_static_runtime(cfd_studio) -- see that
# function's own comment in cmake/StaticRuntime.cmake. Qt is a set of
# dynamic DLLs on this toolchain; cfd_studio ships alongside them
# (via windeployqt, Phase 7) instead of statically linking like every
# other target in this project.
cfd_copy_embree_dlls(cfd_studio)
cfd_deploy_qt(cfd_studio)
