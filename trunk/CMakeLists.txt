cmake_minimum_required(VERSION 2.4.6)

set(CMAKE_CXX_FLAGS "-O3 -g -Wall")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(${LIBRARY_OUTPUT_PATH})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/usr/local/include/opencv2)
include_directories(/usr/local/include/opencv)
include_directories(/usr/local/include/g2o)
include_directories(src)
link_directories(/usr/local/lib)

include_directories(src/graph)
add_library(graph src/graph/VertexCamera.cpp src/graph/VertexPlane.cpp src/graph/VertexPoint3D.cpp src/graph/EdgeCameraPlane.cpp)


include_directories(src/FeatureDescriptor)
add_library(FeatureDescriptor src/FeatureDescriptor/FeatureDescriptor.cpp src/FeatureDescriptor/SurfFeatureDescriptor64.cpp src/FeatureDescriptor/SurfFeatureDescriptor128.cpp src/FeatureDescriptor/OrbFeatureDescriptor.cpp src/FeatureDescriptor/FPFHFeatureDescriptor.cpp src/FeatureDescriptor/IntegerHistogramFeatureDescriptor.cpp src/FeatureDescriptor/FloatHistogramFeatureDescriptor.cpp)


include_directories(src/mygeometry)
add_library(mygeometry src/mygeometry/Point.cpp src/mygeometry/Line.cpp src/mygeometry/Plane.cpp src/mygeometry/KeyPoint.cpp src/mygeometry/KeyPointSet.cpp)
target_link_libraries(mygeometry FeatureDescriptor)

include_directories(src/FeatureExtractor)
add_library(FeatureExtractor src/FeatureExtractor/FeatureExtractor.cpp src/FeatureExtractor/OrbExtractor.cpp src/FeatureExtractor/OwnFeatureExtractor.cpp  src/FeatureExtractor/SurfExtractor.cpp)#src/FeatureExtractor/FPFHExtractor.cpp
target_link_libraries(FeatureExtractor mygeometry)
#target_link_libraries(FeatureExtractor FeatureDescriptor)
target_link_libraries(FeatureExtractor ${PCL_LIBRARIES})
target_link_libraries(FeatureExtractor opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

include_directories(src/RGBDSegmentation)
add_library(RGBDSegmentation src/RGBDSegmentation/RGBDSegmentation.cpp)

include_directories(src/core)
add_library(core src/core/RGBDFrame.cpp src/core/Transformation.cpp)
target_link_libraries(core RGBDSegmentation)
target_link_libraries(core FeatureExtractor)

include_directories(src/FrameMatcher)
add_library(FrameMatcher src/FrameMatcher/FrameMatcher.cpp src/FrameMatcher/AICK.cpp src/FrameMatcher/BasicIcpMatcher.cpp src/FrameMatcher/BasicGIcpMatcher.cpp src/FrameMatcher/NDTMatcher.cpp src/FrameMatcher/RansacPCLCopyMatcher.cpp src/FrameMatcher/RansacPCLCopySlowMatcher.cpp)
target_link_libraries(FrameMatcher ${PCL_LIBRARIES})
target_link_libraries(FrameMatcher core)
#target_link_libraries(FrameMatcher mygeometry)
#target_link_libraries(FrameMatcher FeatureDescriptor)
#target_link_libraries(FrameMatcher FeatureExtractor)
target_link_libraries(FrameMatcher opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

include_directories(src/Map)
add_library(map3d src/Map/Map3D.cpp src/Map/Map3Dbase.cpp src/Map/Map3DbaseGraph.cpp src/Map/Map3Dbow.cpp)
target_link_libraries(map3d ${PCL_LIBRARIES})
target_link_libraries(map3d core)
target_link_libraries(map3d mygeometry)
target_link_libraries(map3d FrameMatcher)
target_link_libraries(map3d graph)

##########################################################################################################################################################################
########################################################################       APPS       ################################################################################
##########################################################################################################################################################################

add_executable(kinect_at_home_test src/kinect_at_home.cpp)
target_link_libraries(kinect_at_home_test ${PCL_LIBRARIES})
target_link_libraries(kinect_at_home_test ${OpenCV_LIBS})
target_link_libraries(kinect_at_home_test map3d)
#target_link_libraries(kinect_at_home_test core)
#target_link_libraries(kinect_at_home_test graph)
#target_link_libraries(kinect_at_home_test mygeometry)
#target_link_libraries(kinect_at_home_test FeatureDescriptor)
#target_link_libraries(kinect_at_home_test FeatureExtractor)
#target_link_libraries(kinect_at_home_test FrameMatcher)
#target_link_libraries(kinect_at_home_test RGBDSegmentation)
target_link_libraries(kinect_at_home_test g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(kinect_at_home_test opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)












