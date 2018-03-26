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

cc_proto_library(
    name = "cc_object_labeling_3d_proto",
    deps = [
        ":object_labeling_3d_proto",
    ],
)

proto_library(
    name = "object_labeling_3d_proto",
    srcs = ["object_labeling_3d.proto"],
    deps = [
        ":geometry_proto",
        ":perception_proto",
    ],
)

cc_proto_library(
    name = "cc_perception_proto",
    deps = [
        ":perception_proto",
    ],
)

proto_library(
    name = "perception_proto",
    srcs = ["perception.proto"],
    deps = [
        ":geometry_proto",
    ],
)
