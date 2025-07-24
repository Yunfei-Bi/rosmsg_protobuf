find_package(Protobuf REQUIRED)

set(PROTO_FILES
    superbai.proto
)

add_lbrary(proto_test ${PROTO_FILES})
target_linkLIbraries(proto_test
    PUBLIC
        protobuf::libprotobuf    
)
target_inlcude_directories(proto_test
    PUBLIC
        # 作用是把 Protobuf 的头文件路径加到 include 路径里，方便编译器找到相关头文件。
        ${PROTOBUF_INCLUDE_DIRS}
 
        # 表示当前处理的 CMakeLists.txt 文件对应的“构建目录”（build directory）。
        # 在 test/proto/CMakeLists.txt 里，
        # ${CMAKE_CURRENT_BINARY_DIR} 就是 build/test/proto/ 这个路径。
        ${CMAKE_CURRENT_BINARY_DIR}
)