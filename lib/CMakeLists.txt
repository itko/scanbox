# add this path to include path of library files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# list here your source files (*.cpp) of your module
set(source_files
        classifier/knn.cpp
        clustering/kmeans.cpp
        clustering/mean_shift.cpp
        clustering/mean_shift_cpu_optimized.cpp
        clustering/single_cluster.cpp
        clustering/label_clustering.cpp
        generic/cluster.cpp
        generic/cluster_chain.cpp
        generic/cluster_chain_accumulator.cpp
        generic/cluster_descriptor.cpp
        generic/disparity_map.cpp
        generic/explode.cpp
        generic/frame.cpp
        generic/frame_window.cpp
        generic/point_cloud.cpp
        generic/read_csv.cpp
        generic/read_png.cpp
        generic/resolve_symlink.cpp
        generic/trajectory.cpp
        generic/center.cpp
        generic/write_point_cloud.cpp
        generic/write_png.cpp
        generic/random_sample.cpp
        descripting/cog.cpp
        matching/nearest_neighbour.cpp
        matching/matching.cpp
        frame_window_filtering/disparity_gaussian_blur.cpp
        frame_window_filtering/disparity_bilateral.cpp
        frame_window_filtering/disparity_median.cpp
        frame_window_filtering/disparity_morphology.cpp
        frame_window_filtering/background_subtraction.cpp
        frame_window_filtering/hog_labeling.cpp
        frame_window_filtering/strict_labeling.cpp
        point_cloud_filtering/statistical_outlier_removal.cpp
        point_cloud_filtering/subsample.cpp
        registration/disparity_registration.cpp
        registration/disparity_registration_cpu_optimized.cpp
        trajectory_builder/cog_trajectory_builder.cpp
        color/color.cpp
        )

# list here your testing files (*.test.cc) of your module
set(test_files
        test_root.cc
        generic/explode.test.cc
        generic/erase_indices.test.cc
        generic/random_sample.test.cc
        generic/point_cloud.test.cc
        generic/read_csv.test.cc
        generic/read_png.test.cc
        clustering/mean_shift.test.cc
        clustering/single_cluster.test.cc
        spatial/brute_force.test.cc
        spatial/cube_iterator.test.cc
        spatial/cubic_neighborhood.test.cc
        spatial/statistical_outlier_detection.test.cc
        spatial/uniform_grid.test.cc
        spatial/flann.test.cc
        trajectory_builder/cog_trajectory_builder.test.cc
        )

add_library(libmousetrack ${source_files})

# add this path to include path of target
target_include_directories(libmousetrack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package( Boost COMPONENTS
    system
    filesystem
    log
    unit_test_framework
    REQUIRED )

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

find_package( Flann REQUIRED )

find_package( OpenCV REQUIRED )

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( PNG REQUIRED )
add_definitions(${PNG_DEFINITIONS})
include_directories(${PNG_INCLUDE_DIRS})

target_link_libraries(libmousetrack
    ${OpenCV_LIBS}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_LOG_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${PNG_LIBRARIES}
    )

add_executable(tests ${test_files})
target_link_libraries(tests
    libmousetrack
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

# create a target for the headers, we don't actually need the result
# but this way, IDE's also show header files in their project overviews
file(GLOB_RECURSE header_files "*.h")
add_custom_target(headers SOURCES ${header_files})
