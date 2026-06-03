cmake_minimum_required(VERSION 3.8)
project(oit_robot_utils)

# -----------------------------------------------------------------------------
# 1. 依存パッケージの検索
# -----------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -----------------------------------------------------------------------------
# 2. メッセージ・インターフェースの生成
# -----------------------------------------------------------------------------
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Custom.msg"
#   DEPENDENCIES std_msgs
# )

# -----------------------------------------------------------------------------
# 3. C++ 実行ファイルのビルド設定
# -----------------------------------------------------------------------------
# include_directories(include)

# set(OIT_ROBOT_UTILS_SRCS
#   "src/program.cpp"
# )

# -----------------------------------------------------------------------------
# 4. インストール設定 (バイナリ、スクリプト、リソース)
# -----------------------------------------------------------------------------

# C++ ノードのインストール
# install(TARGETS
#   test01
#   DESTINATION lib/${PROJECT_NAME}
# )

# Pythonライブラリ (oit_cybergear) のインストール
ament_python_install_package(${PROJECT_NAME})

# 実行用スクリプト (scripts/*.py, *.sh) のインストール
file(GLOB ALL_SCRIPTS "scripts/*.sh" "scripts/*.py")
if(ALL_SCRIPTS)
  install(PROGRAMS
    ${ALL_SCRIPTS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# 設定ファイル・ローンチファイルのインストール
# install(DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )

# -----------------------------------------------------------------------------
# 5. エクスポートとパッケージ化
# -----------------------------------------------------------------------------

# 他のパッケージがこのメッセージや依存関係を利用できるようにする
ament_export_dependencies(rclcpp std_msgs angles rosidl_default_runtime)

ament_package()
