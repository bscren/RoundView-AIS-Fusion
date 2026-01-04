cmake_minimum_required(VERSION 3.8)
project(marnav_vis_cpp)

# 添加C++17标准配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ========== ONNX Runtime路径(1.18.1 C++ 版本) ==========
set(ONNXRUNTIME_DIR "/home/tl/onnxruntime-linux-x64-gpu-1.18.1")

# 编译选项（启用警告）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包（ROS 2相关）
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# 查找message_filters包（用于时间同步）
find_package(message_filters REQUIRED)

# 查找marnav_interfaces自定义消息包
find_package(marnav_interfaces REQUIRED)

# 查找OpenCV
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui)

# 查找GeographicLib库
find_package(GeographicLib REQUIRED)

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 配置头文件搜索路径
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)



# 可执行文件配置
add_executable(deep_sorvf_node
  src/DeepSORVF_ros.cpp
  src/AIS_utils.cpp
  src/VIS_utils.cpp
  src/yolo_onnx.cpp
)
# 包含目录设置
target_include_directories(deep_sorvf_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
# 依赖项链接（ROS 2包）
ament_target_dependencies(deep_sorvf_node
  rclcpp
  ament_index_cpp
  sensor_msgs
  cv_bridge
  yaml-cpp
  message_filters
  marnav_interfaces
)
target_link_directories(deep_sorvf_node
  PRIVATE
  ${ONNXRUNTIME_DIR}/lib
)
# 链接库（OpenCV和GeographicLib）
target_link_libraries(deep_sorvf_node
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  GeographicLib::GeographicLib
  onnxruntime
)


# YOLOv11 图像推理节点
add_executable(yolo_onnx_node 
  src/yolo_onnx.cpp
  src/yolo_example.cpp
)
target_include_directories(yolo_onnx_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_DIR}/include
)
target_link_directories(yolo_onnx_node
  PRIVATE
  ${ONNXRUNTIME_DIR}/lib
)
target_link_libraries(yolo_onnx_node
  ${OpenCV_LIBRARIES}
  onnxruntime
)

set_target_properties(yolo_onnx_node PROPERTIES
  INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_WITH_INSTALL_RPATH TRUE
)

# CKPT ONNX 特征提取节点
add_executable(ckpt_onnx_node 
  src/ckpt_onnx.cpp
)
target_include_directories(ckpt_onnx_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_DIR}/include
)
target_link_directories(ckpt_onnx_node
  PRIVATE
  ${ONNXRUNTIME_DIR}/lib
)
target_link_libraries(ckpt_onnx_node
  ${OpenCV_LIBRARIES}
  onnxruntime
)

set_target_properties(ckpt_onnx_node PROPERTIES
  INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_WITH_INSTALL_RPATH TRUE
)

# 安装可执行文件
install(TARGETS
  deep_sorvf_node
  yolo_onnx_node
  ckpt_onnx_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装config目录（如果存在）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# 安装launch目录（如果存在）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

