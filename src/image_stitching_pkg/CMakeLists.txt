cmake_minimum_required(VERSION 3.8)
project(image_stitching_pkg)
# 在project()之后添加
# 添加C++17标准配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)  # 强制生成调试信息
add_compile_options(-g -O0)  # -g生成调试符号，-O0关闭优化（避免调试时变量被优化）

# 编译选项（启用警告）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包（ROS 2相关）
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# 查找nlohmann_json包
find_package(nlohmann_json REQUIRED)

# 显式查找 marnav_interfaces 和 mycustface自定义消息包
find_package(mycustface REQUIRED)
find_package(marnav_interfaces REQUIRED)

# # 使用pkg-config查找OpenCV 4.8.0（替代find_package，解决cmake配置文件缺失问题）
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(OpenCV REQUIRED opencv4)  # 依赖pkg-config的opencv4配置
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui stitching xfeatures2d)

# 配置头文件搜索路径（关键步骤）
include_directories(
  include  # 基础目录
  include/image_stitching_pkg  # 补充头文件所在的子目录（关键）
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)  # 注意：删除无效的 ${catkin_INCLUDE_DIRS}（ROS 2 不使用 catkin）


# 调试信息：确认OpenCV配置
# message(STATUS "Found OpenCV via pkg-config: ${OpenCV_VERSION}")
# message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV libraries: ${OpenCV_LIBRARIES}")


# ---------------------- 添加CUDA配置 ----------------------
# 手动指定CUDA路径（根据你的实际路径修改）
set(CUDA_PATH "/usr/local/cuda-11.8")
set(CUDA_INCLUDE_DIRS "${CUDA_PATH}/targets/x86_64-linux/include")
set(CUDA_LIB_DIR "${CUDA_PATH}/targets/x86_64-linux/lib")

# 验证CUDA路径是否存在
if(NOT EXISTS ${CUDA_INCLUDE_DIRS})
  message(FATAL_ERROR "CUDA include directory not found: ${CUDA_INCLUDE_DIRS}")
endif()
if(NOT EXISTS ${CUDA_LIB_DIR})
  message(FATAL_ERROR "CUDA library directory not found: ${CUDA_LIB_DIR}")
endif()

# 添加CUDA头文件目录到所有目标
# include_directories(${CUDA_INCLUDE_DIRS})
# 添加CUDA库文件目录
link_directories(${CUDA_LIB_DIR})

# 声明需要链接的CUDA库（核心库：cudart）
set(CUDA_LIBS cudart)
# ----------------------------------------------------------


# 可执行文件配置
## 1. 离线拼接测试节点
add_executable(image_stitching_node 
  src/module_test_stitchfunction.cpp
)  # 补充闭合括号

## 2. RTSP实时拼接节点
add_executable( stitch
  src/module_test_rtsp_stitch.cpp
)  # 补充闭合括号

## 3. RTSP实时显示视频流节点（JH_rtsp.cpp）
add_executable(JH_rtsp
  src/JH_rtsp.cpp
)  # 补充闭合括号

## 4.封装JH_ROS版本
add_executable(JH_ROS_stitch
  src/JH_ros2_stitch_node.cpp
  src/JH_stitcher.cpp
)


# 包含目录设置（添加OpenCV头文件路径）
target_include_directories(image_stitching_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/image_stitching_pkg>  # 补充子目录
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)  # 补充闭合括号

target_include_directories(stitch
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)  # 补充闭合括号

target_include_directories(JH_rtsp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)  # 补充闭合括号



# 依赖项链接（ROS 2包）
ament_target_dependencies(image_stitching_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  yaml-cpp
)  # 补充闭合括号

ament_target_dependencies(stitch
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  mycustface  # 添加自定义消息包依赖
)  # 移除OpenCV，避免与pkg-config冲突

ament_target_dependencies(JH_rtsp
  rclcpp
  ament_index_cpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  yaml-cpp
)  # 补充闭合括号

ament_target_dependencies(JH_ROS_stitch
  rclcpp
  ament_index_cpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  mycustface
  marnav_interfaces
  nlohmann_json
  yaml-cpp
)


# 链接库（OpenCV及其他系统库，使用pkg-config获取的库列表）
target_link_libraries(image_stitching_node
  ${OpenCV_LIBRARIES}  # 替代${OpenCV_LIBS}，适配pkg-config
  yaml-cpp
  ${CUDA_LIBS}
)  # 补充闭合括号

target_link_libraries(stitch
  ${OpenCV_LIBRARIES}
)  # 补充闭合括号

target_link_libraries(JH_rtsp
  ${OpenCV_LIBRARIES}
  yaml-cpp
)  # 补充闭合括号

target_link_libraries(JH_ROS_stitch
  ${OpenCV_LIBRARIES}
  yaml-cpp
)  # 补充闭合括号



# 安装可执行文件
install(TARGETS
  image_stitching_node
  stitch
  JH_rtsp
  JH_ROS_stitch
  DESTINATION lib/${PROJECT_NAME}
)  # 补充闭合括号

# 单独安装launch目录（存放.launch.py文件）
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装config目录（存放配置文件）
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 测试配置（保持默认）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()