cmake_minimum_required(VERSION 3.5)
project(coordinate_transform_lib)

# Укажите необходимые зависимости
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(coordinate_transform_lib STATIC
    src/coordinate_transform_lib.cpp  # Убедитесь, что путь правильный
)
# Укажите зависимости для библиотеки
ament_target_dependencies(coordinate_transform_lib
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  yaml-cpp
  tf2_geometry_msgs
)

# Укажите, где находятся заголовочные файлы
include_directories(
    include
)

# Укажите, куда помещать заголовочные файлы
target_include_directories(coordinate_transform_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Установка
install(TARGETS coordinate_transform_lib
    EXPORT coordinate_transform_lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(coordinate_transform_lib)
ament_export_include_directories(include)
ament_export_libraries(coordinate_transform_lib)

ament_package()