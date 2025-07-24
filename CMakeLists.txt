cmake_minimum_required(VERSION 3.10)
project(myproject)

find_package(catkin REQUIRED COMPENENTS
    roscpp_traits rostime
    roscpp
    roslib
)

# 声明这是一个catkin包，生成必要的配置文件
catkin_package()

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")

add_subdirectory(message_traits)
add_subdirectory(message_serialization)
add_subdirectory(sample_protos)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(pb_talker pb_talker.cpp)

# 这行代码的意思是：为 pb_talker 目标添加依赖项
# pb_talker - 主目标（可能是一个可执行文件或库）
# copy_traits_files - 第一个依赖目标
# copy_serialization_files - 第二个依赖目标
add_dependencies(pb_talker copy_traits_files copy_serialization_files)

target_link_libraries(pb_talker
    ${roscpp_LIBRARIES}
    pb_proto
    roscpp_proto_serialization
)

add_executable(listener listener.cpp)
target_link_libraries(listener ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})

install(TARGETS talker listener
    DESTINATION bin
)

message("!!!!!!!!!!")
message("pb = ${CMAKE_CURRENT_SOURCE_DIR}")
message("${CATKIN_DEVEL_PREFIX}")
message("${CATKIN_PACKAGE_ETC_DESTINATION}")

# 这行代码的意思是：创建一个符号链接（软链接）
# execute_process - CMake命令，用于执行外部命令
# cmake -E create_symlink - 使用CMake的跨平台工具创建符号链接
# ${CATKIN_DEVEL_PREFIX} - 源路径（链接指向的目标）
# ${CMAKE_CURRENT_SOURCE_DIR}/devel - 链接路径（创建的链接位置）
execute_process(COMMAND cmake -E create_symlink ${CATKIN_DEVEL_PREFIX}
    ${CMAKE_CCURRENT_SOURCE_DIR}/devel
)