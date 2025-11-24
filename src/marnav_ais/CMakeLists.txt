cmake_minimum_required(VERSION 3.8)
project(marnav_ais)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(marnav_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find_package(LibSerial REQUIRED)
# 改为手动查找 libserial 的头文件和库
# 1. 查找 libserial 头文件目录
find_path(LIBSERIAL_INCLUDE_DIR
  NAMES libserial/SerialPort.h  # 核心头文件
  PATHS /usr/include  # 常见的系统头文件路径
)
# 2. 查找 libserial 库文件
find_library(LIBSERIAL_LIBRARY
  NAMES serial  # 库文件实际名为 libserial.so，因此这里用 "serial"
  PATHS /usr/lib /usr/local/lib  # 常见的系统库路径
)
# 3. 检查是否找到
if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIBRARY)
  message(FATAL_ERROR "libserial 库未找到，请确认已安装 libserial-dev")
endif()
# 4. 添加头文件路径
include_directories(${LIBSERIAL_INCLUDE_DIR})



# ========== 1. 编译 marnav 库 ==========
# 禁用 marnav 的示例 examples 编译（避免 Qt 依赖）
set(ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
# 可选：如果还有其他不需要的模块（如工具、测试），也可以一起禁用
set(ENABLE_TOOLS OFF CACHE BOOL "" FORCE)
set(ENABLE_TESTS OFF CACHE BOOL "" FORCE)
# 然后添加 marnav 子目录,将 marnav 作为子项目编译（利用其自带的 CMakeLists.txt）
add_subdirectory(extern/marnav)


# ========== 2. 包含 marnav 头文件路径 ==========
include_directories(
    include  # 你自己的头文件目录（若有）
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/marnav/include  # marnav 的头文件路径
)


# ========== 3. 链接 marnav 库到你的目标（AIS 解析节点） ==========
add_executable(ais_parser_node src/ais_parser_node.cpp)  

# 5. 链接 libserial 库（在 target_link_libraries 中使用 ${LIBSERIAL_LIBRARY}）
# 链接 marnav 库（marnav 编译后会生成名为 `marnav` 的库）
target_link_libraries(ais_parser_node  
    marnav  
    rclcpp::rclcpp  # 链接 rclcpp 库
    ${LIBSERIAL_LIBRARY}  # 链接找到的 libserial 库
    GeographicLib::GeographicLib  # 链接 GeographicLib 库
)  

# 链接 ROS2 依赖（如 rclcpp）
ament_target_dependencies(ais_parser_node  
    rclcpp  
    std_msgs  # 若需发布消息，可添加
    marnav_interfaces
)  

# ========== 4. 安装可执行文件（让 ROS2 能找到） ==========
install(TARGETS  
    ais_parser_node  
    DESTINATION lib/${PROJECT_NAME}  
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
