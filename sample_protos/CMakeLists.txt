find_package(Protobuf REQUIRED)

set(PROTO_FILES
    publish_info.proto
)

add_library(pb_proto ${PROTO_FILES})
target_link_libraries(pb_proto
    PUBLIC
        protobuf::libprotobuf
)
target_include_directories(pb_proto
    PUBLIC
        ${PROTOBUF_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}
)

protobuf_generate(TARGET pb_proto
    LANGUAGE cpp
)