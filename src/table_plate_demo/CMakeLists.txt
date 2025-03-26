cmake_minimum_required(VERSION 3.8)
project(table_plate_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package() 