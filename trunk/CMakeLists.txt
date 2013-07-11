cmake_minimum_required(VERSION 2.4.6)

set(CMAKE_CXX_FLAGS "-O4 -Wall")
#set(CMAKE_CXX_FLAGS "-O3 -g -Wall")
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
include_directories(EXTERNAL/g2o/trunk)
include_directories(EXTERNAL/g2o/trunk/build)
#include_directories(EXTERNAL/g2o/trunk/g2o/build/g2o/EXTERNAL/csparse/cs.h)
include_directories(EXTERNAL/g2o/trunk/EXTERNAL/csparse)
#include_directories(EXTERNAL/g2o/include/g2o)
#include_directories(EXTERNAL/g2o/build/g2o/)
#EXTERNAL/g2o/build/g2o
#include_directories(EXTERNAL/g2o/build/g2o/EXTERNAL/csparse)
#include_directories(EXTERNAL/g2o/include/)
link_directories(EXTERNAL/g2o/trunk/lib)
#link_directories(EXTERNAL/g2o/build/g2o/lib)
#link_directories(EXTERNAL/g2o/lib)
#link_directories(/usr/local/lib)

include_directories(src/graph)
add_library(graph src/graph/VertexCamera.cpp src/graph/vertex_point_xyz.cpp src/graph/EdgeCameraPlane.cpp)


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
add_library(RGBDSegmentation src/RGBDSegmentation/RGBDSegmentation.cpp src/RGBDSegmentation/RGBDSegmentationBase.cpp src/RGBDSegmentation/RGBDSegmentationTest.cpp src/RGBDSegmentation/RGBDSegmentationDummy.cpp src/RGBDSegmentation/RGBDSegmentationPCL.cpp src/RGBDSegmentation/RGBDSegmentationScaleSearch.cpp)

include_directories(src/core)
add_library(core src/core/RGBDFrame.cpp src/core/Transformation.cpp)
target_link_libraries(core RGBDSegmentation)
target_link_libraries(core FeatureExtractor)

include_directories(src/graphcutting)
add_library(graphcutting src/graphcutting/GraphForCut.cpp)

include_directories(src/TransformationFilter)
add_library(TransformationFilter src/TransformationFilter/TransformationFilter.cpp src/TransformationFilter/FeatureDistanceFilter.cpp src/TransformationFilter/PixelDistanceFilter.cpp src/TransformationFilter/DistanceNetFilter.cpp src/TransformationFilter/RansacFilter.cpp src/TransformationFilter/BowFilter.cpp src/TransformationFilter/UpdateWeightFilter.cpp src/TransformationFilter/UpdateWeightFilterv2.cpp )
target_link_libraries(TransformationFilter ${PCL_LIBRARIES})
target_link_libraries(TransformationFilter core)
#target_link_libraries(TransformationFilter mygeometry)
#target_link_libraries(TransformationFilter FeatureDescriptor)
#target_link_libraries(TransformationFilter FeatureExtractor)
target_link_libraries(TransformationFilter opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

include_directories(src/FrameMatcher)
add_library(FrameMatcher src/FrameMatcher/FrameMatcher.cpp src/FrameMatcher/AICK.cpp src/FrameMatcher/AICKv2.cpp src/FrameMatcher/BowAICKv2.cpp src/FrameMatcher/dnet.cpp src/FrameMatcher/DistanceNetMatcher.cpp src/FrameMatcher/DistanceNetMatcherv2.cpp src/FrameMatcher/DistanceNetMatcherv3.cpp src/FrameMatcher/DistanceNetMatcherv4.cpp src/FrameMatcher/DistanceNetMatcherv5.cpp src/FrameMatcher/BasicIcpMatcher.cpp src/FrameMatcher/BasicGIcpMatcher.cpp src/FrameMatcher/NDTMatcher.cpp src/FrameMatcher/RansacPCLCopyMatcher.cpp src/FrameMatcher/RansacPCLCopySlowMatcher.cpp src/FrameMatcher/MultiLayerMatcher.cpp src/FrameMatcher/MultiFilterMatcher.cpp src/FrameMatcher/FilterMatcher.cpp src/FrameMatcher/GraphCutMatcherv1.cpp)
target_link_libraries(FrameMatcher ${PCL_LIBRARIES})
target_link_libraries(FrameMatcher core)
target_link_libraries(FrameMatcher TransformationFilter)
target_link_libraries(FrameMatcher graphcutting)
#target_link_libraries(FrameMatcher mygeometry)
#target_link_libraries(FrameMatcher FeatureDescriptor)
#target_link_libraries(FrameMatcher FeatureExtractor)
target_link_libraries(FrameMatcher opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)


include_directories(src/Map)
add_library(map3d src/Map/Map3D.cpp src/Map/Map3Dbase.cpp src/Map/Map3DbaseGraph.cpp src/Map/Map3DPlanesGraph.cpp src/Map/Map3DPlanesGraphv2.cpp src/Map/Map3DPlanesGraphv3.cpp src/Map/Map3DPlanesGraphv4.cpp src/Map/Map3Dbow.cpp)
target_link_libraries(map3d ${PCL_LIBRARIES})
target_link_libraries(map3d core)
target_link_libraries(map3d mygeometry)
target_link_libraries(map3d FrameMatcher)
target_link_libraries(map3d graph)
target_link_libraries(map3d graphcutting)

##########################################################################################################################################################################
########################################################################       APPS       ################################################################################
##########################################################################################################################################################################

add_executable(kinect_at_home_test src/kinect_at_home.cpp)
target_link_libraries(kinect_at_home_test ${PCL_LIBRARIES})
target_link_libraries(kinect_at_home_test ${OpenCV_LIBS})
target_link_libraries(kinect_at_home_test map3d)
target_link_libraries(kinect_at_home_test g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(kinect_at_home_test opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

##################

add_executable(kinect_at_home_bow_setup src/kinect_at_home_bow_setup.cpp)
target_link_libraries(kinect_at_home_bow_setup ${PCL_LIBRARIES})
target_link_libraries(kinect_at_home_bow_setup ${OpenCV_LIBS})
target_link_libraries(kinect_at_home_bow_setup map3d)
target_link_libraries(kinect_at_home_bow_setup g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(kinect_at_home_bow_setup opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

##################

add_executable(kinect_at_home_basic src/kinect_at_home_basic.cpp)
target_link_libraries(kinect_at_home_basic ${PCL_LIBRARIES})
target_link_libraries(kinect_at_home_basic ${OpenCV_LIBS})
target_link_libraries(kinect_at_home_basic map3d)
target_link_libraries(kinect_at_home_basic g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(kinect_at_home_bow_setup opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

##################

add_executable(evaluate_registration src/evaluate_registration.cpp)
target_link_libraries(evaluate_registration ${PCL_LIBRARIES} ${OpenCV_LIBS} map3d)
target_link_libraries(evaluate_registration g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(evaluate_registration opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

##################

add_executable(evaluate_registrationv2 src/evaluate_registrationv2.cpp)
target_link_libraries(evaluate_registrationv2 ${PCL_LIBRARIES} ${OpenCV_LIBS} map3d)
target_link_libraries(evaluate_registrationv2 g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(evaluate_registrationv2 opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

##################

add_executable(evaluate_registrationv3 src/evaluate_registrationv3.cpp)
target_link_libraries(evaluate_registrationv3 ${PCL_LIBRARIES} ${OpenCV_LIBS} map3d)
target_link_libraries(evaluate_registrationv3 g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(evaluate_registrationv3 opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

##################

add_executable(evaluate_registrationv4 src/evaluate_registrationv4.cpp)
target_link_libraries(evaluate_registrationv4 ${PCL_LIBRARIES} ${OpenCV_LIBS} map3d)
target_link_libraries(evaluate_registrationv4 g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(evaluate_registrationv4 opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

###################

add_executable(imu_grabber src/imu_grabber.cpp)
target_link_libraries(imu_grabber phidget21)
target_link_libraries(imu_grabber ${PCL_LIBRARIES} ${OpenCV_LIBS} map3d)
target_link_libraries(imu_grabber g2o_cli g2o_core g2o_incremental g2o_interactive g2o_interface)
target_link_libraries(imu_grabber g2o_math_groups g2o_parser g2o_solver_cholmod g2o_solver_csparse) 
target_link_libraries(imu_grabber g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp)
target_link_libraries(imu_grabber g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
target_link_libraries(imu_grabber opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)


###################

add_executable(evaluate_registration_surf src/evaluate_registration_surf.cpp)
target_link_libraries(evaluate_registration_surf ${PCL_LIBRARIES} ${OpenCV_LIBS} map3d)

###################

add_executable(evaluate_registration_orb src/evaluate_registration_orb.cpp)
target_link_libraries(evaluate_registration_orb ${PCL_LIBRARIES} ${OpenCV_LIBS} map3d)
###################

add_executable(evaluate_registration_surf_weight src/evaluate_registration_surf_weight.cpp)
target_link_libraries(evaluate_registration_surf_weight ${PCL_LIBRARIES} ${OpenCV_LIBS} map3d)

###################
