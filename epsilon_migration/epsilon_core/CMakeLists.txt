cmake_minimum_required(VERSION 3.8)
project(epsilon_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3 -Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(OpenMP REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)
find_package(BLAS REQUIRED)

find_library(GLOG_LIBRARY NAMES glog REQUIRED)
find_library(DW_LIBRARY NAMES dw)

set(OOQP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/core/common/thirdparty/ooqp)
find_path(OOQP_INCLUDE_DIR
  NAMES QpGenData.h
  HINTS ${OOQP_ROOT}/include/ooqp
  REQUIRED
)

set(OOQP_LIBRARIES)
foreach(OOQP_LIB_NAME ooqpgensparse ooqpsparse ooqpgondzio ooqpbase ma27)
  find_library(OOQP_LIB_${OOQP_LIB_NAME}
    NAMES ${OOQP_LIB_NAME}
    HINTS ${OOQP_ROOT}/lib
    REQUIRED
  )
  list(APPEND OOQP_LIBRARIES ${OOQP_LIB_${OOQP_LIB_NAME}})
endforeach()

find_library(GFORTRAN_LIBRARY
  NAMES gfortran libgfortran.so.5 libgfortran.so.4
  HINTS /usr/lib/gcc/x86_64-linux-gnu/11 /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  REQUIRED
)
list(APPEND OOQP_LIBRARIES ${BLAS_LIBRARIES} ${GFORTRAN_LIBRARY})

protobuf_generate_cpp(EUDM_PROTO_SRCS EUDM_PROTO_HDRS
  util/eudm_planner/proto/eudm_config.proto
)
protobuf_generate_cpp(SSC_PROTO_SRCS SSC_PROTO_HDRS
  util/ssc_planner/proto/ssc_config.proto
)

set(EPSILON_CORE_SOURCES
  core/common/src/common/basics/colormap.cc
  core/common/src/common/basics/semantics.cc
  core/common/src/common/basics/shapes.cc
  core/common/src/common/basics/tool_func.cc
  core/common/src/common/circle_arc/circle_arc.cc
  core/common/src/common/circle_arc/circle_arc_branch.cc
  core/common/src/common/idm/intelligent_driver_model.cc
  core/common/src/common/lane/lane.cc
  core/common/src/common/lane/lane_generator.cc
  core/common/src/common/math/calculations.cc
  core/common/src/common/mobil/mobil_behavior_prediction.cc
  core/common/src/common/mobil/mobil_model.cc
  core/common/src/common/primitive/frenet_primitive.cc
  core/common/src/common/rss/rss_checker.cc
  core/common/src/common/solver/ooqp_interface.cc
  core/common/src/common/solver/qp_solver.cc
  core/common/src/common/spline/lookup_table.cc
  core/common/src/common/spline/spline_generator.cc
  core/common/src/common/state/free_state.cc
  core/common/src/common/state/state_transformer.cc
  core/vehicle_model/src/vehicle_model/controllers/ctx_idm_velocity_controller.cc
  core/vehicle_model/src/vehicle_model/controllers/idm_velocity_controller.cc
  core/vehicle_model/src/vehicle_model/controllers/pid_controller.cc
  core/vehicle_model/src/vehicle_model/controllers/pure_pursuit_controller.cc
  core/vehicle_model/src/vehicle_model/ctx_idm_model.cc
  core/vehicle_model/src/vehicle_model/ideal_steer_model.cc
  core/vehicle_model/src/vehicle_model/idm_model.cc
  core/vehicle_model/src/vehicle_model/vehicle_model.cc
  core/semantic_map_manager/src/config_loader.cc
  core/semantic_map_manager/src/data_renderer.cc
  core/semantic_map_manager/src/semantic_map_manager.cc
  core/semantic_map_manager/src/traffic_signal_manager.cc
  core/route_planner/src/route_planner/route_planner.cc
  core/behavior_planner/src/behavior_planner/behavior_planner.cc
  core/behavior_planner/src/behavior_planner/map_adapter.cc
  util/eudm_planner/src/eudm_planner/dcp_tree.cc
  util/eudm_planner/src/eudm_planner/eudm_manager.cc
  util/eudm_planner/src/eudm_planner/eudm_planner.cc
  util/eudm_planner/src/eudm_planner/map_adapter.cc
  util/ssc_planner/src/ssc_planner/map_adapter.cc
  util/ssc_planner/src/ssc_planner/ssc_map.cc
  util/ssc_planner/src/ssc_planner/ssc_planner.cc
  ${EUDM_PROTO_SRCS}
  ${SSC_PROTO_SRCS}
)

add_library(epsilon_core STATIC ${EPSILON_CORE_SOURCES})

target_include_directories(epsilon_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/common/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/common/thirdparty>
  $<BUILD_INTERFACE:${OOQP_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/vehicle_model/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/vehicle_model/thirdparty>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/forward_simulator/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/motion_predictor/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/semantic_map_manager/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/semantic_map_manager/thirdparty>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/route_planner/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/behavior_planner/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/util/eudm_planner/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/util/ssc_planner/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(epsilon_core PUBLIC
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  ${OpenCV_LIBS}
  ${Protobuf_LIBRARIES}
  ${OOQP_LIBRARIES}
  ${GLOG_LIBRARY}
  Threads::Threads
)

if(DW_LIBRARY)
  target_link_libraries(epsilon_core PUBLIC ${DW_LIBRARY})
endif()

install(TARGETS epsilon_core
  EXPORT export_epsilon_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  core/common/inc/
  core/vehicle_model/inc/
  core/forward_simulator/inc/
  core/motion_predictor/inc/
  core/semantic_map_manager/inc/
  core/route_planner/inc/
  core/behavior_planner/inc/
  util/eudm_planner/inc/
  util/ssc_planner/inc/
  DESTINATION include
)

install(DIRECTORY
  core/common/thirdparty/
  core/vehicle_model/thirdparty/
  core/semantic_map_manager/thirdparty/
  DESTINATION include
)

install(FILES
  ${EUDM_PROTO_HDRS}
  ${SSC_PROTO_HDRS}
  DESTINATION include
)

install(DIRECTORY
  config/
  util/eudm_planner/config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING
    PATTERN "*.pb.txt"
    PATTERN "*.json"
)

install(DIRECTORY
  util/ssc_planner/config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.pb.txt"
)

ament_export_targets(export_epsilon_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 OpenCV Protobuf)
ament_package()
