package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "cc_geometry_proto",
    deps = [
        ":geometry_proto",
    ],
)

proto_library(
    name = "geometry_proto",
    srcs = ["geometry.proto"],
)

cc_proto_library(
    name = "cc_transform_proto",
    deps = [
        ":transform_proto",
    ],
)

proto_library(
    name = "transform_proto",
    srcs = ["transform.proto"],
)

cc_proto_library(
    name = "cc_display_proto",
    deps = [
        ":display_proto",
    ],
)

proto_library(
    name = "display_proto",
    srcs = ["display.proto"],
    deps = [
        ":geometry_proto",
    ],
)

cc_proto_library(
    name = "cc_status_proto",
    deps = [
        ":status_proto",
    ],
)

proto_library(
    name = "status_proto",
    srcs = ["status.proto"],
)
