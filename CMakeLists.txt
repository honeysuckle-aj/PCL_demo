cmake_minimum_required(VERSION 3.28)
project(demo)
set(CMAKE_CXX_STANDARD 14)

# 设置输出根目录为build
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build)
# 设置可执行程序输出到build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)


# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
# 设置依赖库链接目录
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})

add_executable(demo main.cpp)
target_link_libraries(demo ${PCL_LIBRARIES})

add_custom_target(run
  DEPENDS demo                
  COMMAND $<TARGET_FILE:demo> 
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}  # 确保在构建目录执行
  COMMENT "Running the demo..."                  # 输出提示信息
)